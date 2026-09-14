/*
 * Wuji SDK C — Wuji Hand 2: subscribe to joint state.
 *
 * Scans for a hand, connects to the first one found, and subscribes to the
 * broadcast joint_states stream (~1 kHz). Each frame is a variable-length,
 * self-describing aggregate: `joints` holds only the joints the firmware
 * reported this cycle (up to 20), each tagged by its firmware `nid`.
 * `position` is the joint-side angle in radians; `velocity` (rad/s) and
 * `effort` (the joint torque proxy) complete each entry.
 *
 * Build: point CMake at your extracted SDK tarball (see ../README.md):
 *          cmake -S . -B build \
 *            -DWUJI_SDK_INCLUDE_DIR=/path/to/sdk/include \
 *            -DWUJI_SDK_LIB=/path/to/sdk/lib/libwuji_sdk_c.so
 *          cmake --build build
 * Run:   ./build/0_subscribe_joint_states        (Ctrl+C to stop)
 *
 * THREADING: the callback fires on a dedicated SDK worker thread, NOT main.
 * The `frame` pointer is valid only for the duration of the callback (its heap
 * `joints` array is freed right after the callback returns) — never stash it.
 */
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <stdatomic.h>

#include "wuji_sdk.h"

static volatile sig_atomic_t g_stop = 0;
static void on_sigint(int sig) { (void)sig; g_stop = 1; }

/* Per-subscription state. `frames` is written by the SDK worker thread and read
 * by main, so it is atomic; `last_print_ms` is touched only by the (single)
 * worker thread. */
typedef struct {
    _Atomic uint64_t frames;        /* total OK frames seen */
    uint64_t         last_print_ms; /* monotonic ms of the last printed frame */
} ctx_t;

static uint64_t now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL;
}

static void on_joint_state(WujiFrameKind kind, const WujiJointStateFrame *f, void *ud) {
    ctx_t *ctx = (ctx_t *)ud;

    /* Terminal / non-data frames (f is non-NULL only when kind == OK). */
    if (kind == WUJI_FRAME_KIND_LAG)   { fprintf(stderr, "[joint_states] lagged\n");                 return; }
    if (kind == WUJI_FRAME_KIND_END)   { fprintf(stderr, "[joint_states] stream ended\n");           return; }
    if (kind == WUJI_FRAME_KIND_ERROR) { fprintf(stderr, "[joint_states] error: %s\n", wuji_last_error()); return; }
    if (kind != WUJI_FRAME_KIND_OK || !f) return;

    uint64_t fr = atomic_fetch_add_explicit(&ctx->frames, 1, memory_order_relaxed) + 1;

    /* Throttle printing to ~1 Hz (the broadcast is much faster). */
    uint64_t now = now_ms();
    if (now - ctx->last_print_ms < 1000) return;
    ctx->last_print_ms = now;

    /* The frame carries a FrameHeader (seq + firmware timestamp_us + frame_id,
     * where frame_id is the wrist frame "l_wrist"/"r_wrist"). */
    printf("seq=%u  frame_id=%s  ts=%lluus  num_joints=%u  (frame #%lu)\n",
           f->header.seq, f->header.frame_id, (unsigned long long)f->header.timestamp_us,
           f->num_joints, (unsigned long)fr);
    printf("  %-4s %12s %12s %10s\n", "nid", "position", "velocity", "effort");
    for (size_t i = 0; i < f->joints_len; i++) {
        const WujiJointStateEntry *e = &f->joints[i];
        printf("  %-4u %12.4f %12.4f %10.4f\n",
               e->nid, e->position, e->velocity, e->effort);
    }
    fflush(stdout); /* flush live output when piped (tee/grep) */
}

int main(void) {
    signal(SIGINT, on_sigint);

    printf("Wuji SDK version: %s\n", wuji_version());

    WujiInitOptions opts = { .log_level = 3 };
    if (wuji_init(&opts) != WUJI_STATUS_OK) {
        fprintf(stderr, "wuji_init failed: %s\n", wuji_last_error());
        return 1;
    }

    /* Discover and connect to the first hand found. */
    WujiDiscovered *list = NULL;
    size_t n = 0;
    if (wuji_scan(&list, &n) != WUJI_STATUS_OK) {
        fprintf(stderr, "wuji_scan failed: %s\n", wuji_last_error());
        wuji_shutdown();
        return 1;
    }
    if (n == 0) {
        printf("No devices found\n");
        wuji_discovered_free(list, n);
        wuji_shutdown();
        return 0;
    }
    size_t idx = n;
    for (size_t i = 0; i < n; i++) {
        printf("  SN=%s  Type=%s  Address=%s\n", list[i].serial_number, list[i].model, list[i].address);
        if (idx == n && list[i].device_id == WUJI_DEVICE_TYPE_WUJI_HAND_2) idx = i;
    }
    if (idx == n) {
        printf("No Wuji Hand 2 found among the scanned devices\n");
        wuji_discovered_free(list, n);
        wuji_shutdown();
        return 0;
    }
    printf("Connecting to %s (%s)\n", list[idx].serial_number, list[idx].address);

    WujiConnectTarget tgt = { .kind = WUJI_CONNECT_TARGET_KIND_SN, .value = list[idx].serial_number };
    WujiConnectOptions connect_opts = wuji_connect_options_default();
    connect_opts.timeout_ms = 1000;
    connect_opts.retry_count = 3;
    connect_opts.enable_bridge = true;
    struct WujiDevice *dev = NULL;
    WujiStatus st = wuji_connect(&tgt, "hand2", &connect_opts, &dev);
    wuji_discovered_free(list, n);
    if (st != WUJI_STATUS_OK) {
        fprintf(stderr, "wuji_connect failed: %s\n", wuji_last_error());
        wuji_shutdown();
        return 1;
    }

    ctx_t ctx = { 0, 0 };
    struct WujiSub *sub = NULL;
    if (wuji_hand_2_subscribe_joint_states(dev, on_joint_state, &ctx, &sub) != WUJI_STATUS_OK) {
        fprintf(stderr, "subscribe joint_states: %s\n", wuji_last_error());
        goto cleanup;
    }
    printf("Subscribed to joint_states. Ctrl+C to stop.\n");
    fflush(stdout);

    while (!g_stop) sleep(1);
    printf("\nStopping (%lu frames)...\n",
           (unsigned long)atomic_load_explicit(&ctx.frames, memory_order_relaxed));

    wuji_sub_close(sub); /* stop the worker before releasing the device */
cleanup:
    wuji_dev_disconnect(dev);
    wuji_dev_release(dev);
    wuji_shutdown();
    return 0;
}
