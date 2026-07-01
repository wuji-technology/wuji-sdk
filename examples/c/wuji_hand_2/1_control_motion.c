/*
 * Wuji SDK C — Wuji Hand 2: MIT sweep (control).
 *
 * Auto-detect a Wuji Hand 2, configure MIT impedance control (effort limit +
 * per-joint kp/kd), enable motors, wait for them to report Enabled, then sweep
 * all *online* joints around zero through a fixed frequency table. Each joint
 * follows pos = A·sin(ωt) with velocity feedforward A·ω·cos(ωt), ramped in over
 * the first quarter of each segment, returning to zero between segments.
 *
 * The Wuji Hand 2 runs MIT (impedance) control by default, so there is no
 * control-mode call: we just set the effort limit + kp/kd and enable.
 *
 * !! WARNING: this MOVES the hand through ±~45° sweeps. Keep it clear. !!
 *
 * The C publish call sends one 20-joint command frame:
 *   WujiJointCommand cmds[20];   // each {position, velocity, effort}
 *   wuji_joint_command_publisher_send(pub, cmds);
 * Use {0, 0, 0} for joints you don't want to command.
 *
 * Build: point CMake at your extracted SDK tarball (see ../README.md):
 *          cmake -S . -B build \
 *            -DWUJI_SDK_INCLUDE_DIR=/path/to/sdk/include \
 *            -DWUJI_SDK_LIB=/path/to/sdk/lib/libwuji_sdk_c.so
 *          cmake --build build
 * Run:   ./build/1_control_motion        (Ctrl+C to stop early)
 */
#define _DEFAULT_SOURCE /* usleep(), M_PI */
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <math.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <stdatomic.h>

#include "wuji_sdk.h"

#define TOTAL_JOINTS      WUJI_HAND_2_JOINT_COUNT
#define JOINTS_PER_FINGER 4
#define PUB_HZ            200

/* OD 0x6001 ext_state (status_word & 0x3): 0=Init 1=Ready 2=Enabled 3=Stopped. */
#define EXT_STATE_ENABLED 2u
#define EXT_STATE(sw)     ((sw) & 0x3u)

/* Motion parameters shared with the SDK's sweep examples. */
static const struct { float freq; int cycles; } SWEEP_TABLE[] = {
    {0.125f, 1}, {0.25f, 1}, {0.5f, 2}, {1.0f, 4}, {2.0f, 8},
};
static const float PI_F          = 3.14159265358979f;
static const float AMPLITUDE     = 3.14159265358979f / 4.0f;     /* PI/4         */
static const float AMPLITUDE_J2  = 25.0f * 3.14159265358979f / 180.0f; /* 25 deg  */
static const float KP            = 3.0f;
static const float KD            = 0.05f;
static const float EFFORT_LIMIT  = 1.5f; /* Amps */
static const float ENABLE_TIMEOUT_S = 5.0f;
static const float ZERO_RETURN_S    = 2.0f;
static const float RAMP_FRACTION    = 0.25f;

static volatile sig_atomic_t g_stop = 0;
static void on_sigint(int sig) { (void)sig; g_stop = 1; }

static uint64_t now_us(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000ULL + (uint64_t)ts.tv_nsec / 1000ULL;
}

/* Enable-wait state, written by the joint_diagnostics worker thread. The
 * realtime diagnostics frame carries only the online joints, each with a
 * status_word; once every joint in a frame reports Enabled we are good to go. */
typedef struct { _Atomic int all_enabled; } enable_ctx_t;

static void on_diag(WujiFrameKind kind, const WujiJointDiagnosticsFrame *f, void *ud) {
    enable_ctx_t *c = (enable_ctx_t *)ud;
    if (kind != WUJI_FRAME_KIND_OK || !f || f->joints_len == 0) return;
    int all = 1;
    for (size_t i = 0; i < f->joints_len; i++)
        if (EXT_STATE(f->joints[i].status_word) != EXT_STATE_ENABLED) all = 0;
    if (all) atomic_store_explicit(&c->all_enabled, 1, memory_order_relaxed);
}

/* Fail-fast for the control setup calls: on any non-OK status print the SDK
 * last-error and bail to the `cleanup` label. Used only before the hand is
 * enabled, so no disable is needed on these paths. */
#define CHECK(expr, what) do { \
    if ((expr) != WUJI_STATUS_OK) { \
        fprintf(stderr, "%s: %s\n", (what), wuji_last_error()); \
        goto cleanup; \
    } \
} while (0)

int main(void) {
    signal(SIGINT, on_sigint);

    WujiInitOptions opts = { .log_level = 3 };
    if (wuji_init(&opts) != WUJI_STATUS_OK) {
        fprintf(stderr, "wuji_init failed: %s\n", wuji_last_error());
        return 1;
    }

    /* Discover and connect to the first hand found. */
    WujiDiscovered *list = NULL;
    size_t n_dev = 0;
    if (wuji_scan(&list, &n_dev) != WUJI_STATUS_OK) {
        fprintf(stderr, "wuji_scan failed: %s\n", wuji_last_error());
        wuji_shutdown();
        return 1;
    }
    if (n_dev == 0) {
        printf("No devices found\n");
        wuji_discovered_free(list, n_dev);
        wuji_shutdown();
        return 0;
    }
    WujiConnectTarget tgt = { .kind = WUJI_CONNECT_TARGET_KIND_SN, .value = list[0].serial_number };
    struct WujiDevice *dev = NULL;
    WujiStatus st = wuji_connect(&tgt, "wuji_hand_2", NULL, &dev);
    char sn[64];
    snprintf(sn, sizeof(sn), "%s", list[0].serial_number);
    wuji_discovered_free(list, n_dev);
    if (st != WUJI_STATUS_OK) {
        fprintf(stderr, "wuji_connect failed: %s\n", wuji_last_error());
        wuji_shutdown();
        return 1;
    }

    uint8_t n_online = 0;
    CHECK(wuji_hand_2_online_joints_count(dev, &n_online), "online_joints_count");
    printf("Connected: %s (%u joints online)\n", sn, n_online);
    if (n_online == 0) {
        printf("No joints online, aborting.\n");
        goto cleanup;
    }

    /* Online bitmap (indexed by joint 0..19): which joints to command. We read
     * it from the per-joint effort-limit query — bit i set means joint i is
     * present, so we only drive joints that exist. */
    float limits[TOTAL_JOINTS];
    uint32_t online_mask = 0;
    if (wuji_hand_2_get_all_effort_limit(dev, limits, &online_mask) != WUJI_STATUS_OK) {
        fprintf(stderr, "get_all_effort_limit: %s\n", wuji_last_error());
        goto cleanup;
    }

    /* Configure control: effort limit, per-joint kp/kd, then enable.
     * (MIT is the device default — no control-mode call.) */
    float kp[TOTAL_JOINTS], kd[TOTAL_JOINTS];
    for (int j = 0; j < TOTAL_JOINTS; j++) { kp[j] = KP; kd[j] = KD; }
    CHECK(wuji_hand_2_set_all_effort_limit(dev, EFFORT_LIMIT), "set_all_effort_limit");
    CHECK(wuji_hand_2_set_all_mit_params(dev, kp, kd), "set_all_mit_params");
    CHECK(wuji_hand_2_enable(dev, NULL), "enable"); /* NULL mask = whole hand */

    /* ---- action: MIT sweep ---- */

    /* Wait for all online joints to reach Enabled. We subscribe to
     * joint_diagnostics and watch for an all-enabled frame. */
    enable_ctx_t en = { 0 };
    struct WujiSub *diag_sub = NULL;
    if (wuji_hand_2_subscribe_joint_diagnostics(dev, on_diag, &en, &diag_sub) != WUJI_STATUS_OK) {
        fprintf(stderr, "subscribe joint_diagnostics: %s\n", wuji_last_error());
        wuji_hand_2_disable(dev, NULL);
        goto cleanup;
    }
    uint64_t deadline = now_us() + (uint64_t)(ENABLE_TIMEOUT_S * 1000000.0f);
    while (!atomic_load_explicit(&en.all_enabled, memory_order_relaxed)
           && now_us() < deadline && !g_stop)
        usleep(50000); /* 50 ms */
    wuji_sub_close(diag_sub);
    if (!atomic_load_explicit(&en.all_enabled, memory_order_relaxed)) {
        fprintf(stderr, "Enable timeout!\n");
        wuji_hand_2_disable(dev, NULL);
        goto cleanup;
    }
    printf("All motors enabled.\n");

    struct WujiJointCommandPublisher *pub = NULL;
    if (wuji_hand_2_joint_command_publish(dev, &pub) != WUJI_STATUS_OK) {
        fprintf(stderr, "open publisher: %s\n", wuji_last_error());
        wuji_hand_2_disable(dev, NULL);
        goto cleanup;
    }

    const uint64_t dt_us = 1000000ULL / PUB_HZ;
    WujiJointCommand zeros[TOTAL_JOINTS] = {{0}};
    const int n_segments = (int)(sizeof(SWEEP_TABLE) / sizeof(SWEEP_TABLE[0]));

    for (int s = 0; s < n_segments && !g_stop; s++) {
        float freq = SWEEP_TABLE[s].freq;
        int cycles = SWEEP_TABLE[s].cycles;
        float dur = (float)cycles / freq;
        float omega = 2.0f * PI_F * freq;
        printf("  %.3f Hz x %d = %.1fs\n", freq, cycles, dur);

        uint64_t t0 = now_us();
        long n = 0;
        while (!g_stop) {
            float t = (float)(now_us() - t0) / 1000000.0f;
            if (t >= dur) break;
            float ramp = t / (dur * RAMP_FRACTION);
            if (ramp > 1.0f) ramp = 1.0f;

            WujiJointCommand cmds[TOTAL_JOINTS] = {{0}};
            for (int i = 0; i < TOTAL_JOINTS; i++) {
                if (!WUJI_JOINT_ONLINE(online_mask, i)) continue;
                float amp = (i % JOINTS_PER_FINGER == 2) ? AMPLITUDE_J2 : AMPLITUDE;
                cmds[i].position = amp * sinf(omega * t) * ramp;
                cmds[i].velocity = amp * omega * cosf(omega * t) * ramp;
                cmds[i].effort   = 0.0f;
            }
            if (wuji_joint_command_publisher_send(pub, cmds) != WUJI_STATUS_OK) {
                fprintf(stderr, "send: %s\n", wuji_last_error());
                goto done;
            }
            n++;
            uint64_t target = t0 + dt_us * (uint64_t)n;
            uint64_t now = now_us();
            if (target > now) usleep((useconds_t)(target - now));
        }

        /* Zero-return between segments: hold all-zero commands for ZERO_RETURN_S. */
        uint64_t tz = now_us();
        while ((now_us() - tz) < (uint64_t)(ZERO_RETURN_S * 1000000.0f) && !g_stop) {
            if (wuji_joint_command_publisher_send(pub, zeros) != WUJI_STATUS_OK) {
                fprintf(stderr, "send (zero-return): %s\n", wuji_last_error());
                goto done;
            }
            usleep((useconds_t)dt_us);
        }
    }

done:
    wuji_joint_command_publisher_close(pub); /* releases the PUB stream */
    wuji_hand_2_disable(dev, NULL);
    printf("Sweep complete.\n");

cleanup:
    wuji_dev_disconnect(dev);
    wuji_dev_release(dev);
    wuji_shutdown();
    return 0;
}
