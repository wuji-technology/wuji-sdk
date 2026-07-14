/*
 * Wuji Hand — subscribe to the joint_states stream.
 *
 * Auto-detects and connects to a Wuji Hand, then prints a once-per-second
 * report: frame rate + all 20 joint positions (finger-major, one row per
 * finger). Mirrors examples/python/wuji_hand/0.subscribe.py.
 *
 * Build: see ../README.md.   Run: ./build/0_subscribe   (Ctrl+C to stop)
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <stdatomic.h>

#include "wuji_sdk.h"

/* WujiHand has 20 joints arranged finger-major (thumb first). */
#define FINGERS 5
#define JOINTS_PER_FINGER 4
#define JOINT_COUNT (FINGERS * JOINTS_PER_FINGER)

#define REPORT_MS 1000 /* print one report per second, like the Python example */

static volatile sig_atomic_t g_stop = 0;
static void on_sigint(int sig) { (void)sig; g_stop = 1; }

static uint64_t now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL;
}

/* All fields below are touched only by the (single) subscription worker
 * thread, except `total` which main never reads — no synchronization needed
 * beyond the atomic counter. */
typedef struct {
    _Atomic uint64_t total;   /* total OK frames seen */
    uint64_t window_frames;   /* frames since the last report */
    uint64_t window_start_ms; /* monotonic ms when the window opened */
} ctx_t;

static void on_joint_states(WujiFrameKind kind, const WujiHandJointStates *f, void *ud) {
    ctx_t *ctx = (ctx_t *)ud;

    /* Terminal / non-data frames (f is non-NULL only when kind == OK). */
    if (kind == WUJI_FRAME_KIND_LAG)   { fprintf(stderr, "[joint_states] lagged\n");                       return; }
    if (kind == WUJI_FRAME_KIND_END)   { fprintf(stderr, "[joint_states] stream ended\n");                 return; }
    if (kind == WUJI_FRAME_KIND_ERROR) { fprintf(stderr, "[joint_states] error: %s\n", wuji_last_error()); return; }
    if (kind != WUJI_FRAME_KIND_OK || !f) return;

    uint64_t total = atomic_fetch_add_explicit(&ctx->total, 1, memory_order_relaxed) + 1;
    ctx->window_frames++;

    uint64_t now = now_ms();
    uint64_t elapsed = now - ctx->window_start_ms;
    if (elapsed < REPORT_MS) return;

    double fps = (double)ctx->window_frames * 1000.0 / (double)elapsed;
    printf("[wuji_hand][JointStates] fps=%5.1f total=%llu\n", fps, (unsigned long long)total);

    /* f->position holds all 20 joints in finger-major order (finger1..5 x
     * joint1..4); print one row per finger — same layout as the Python example. */
    if (f->position_len == JOINT_COUNT) {
        for (int fi = 0; fi < FINGERS; fi++) {
            const double *row = &f->position[fi * JOINTS_PER_FINGER];
            printf("  finger%d: [%+.3f, %+.3f, %+.3f, %+.3f]\n",
                   fi + 1, row[0], row[1], row[2], row[3]);
        }
    } else {
        printf("  (unexpected position_len=%zu, want %d)\n", f->position_len, JOINT_COUNT);
    }
    fflush(stdout); /* flush live output when piped (tee/grep) */

    ctx->window_frames = 0;
    ctx->window_start_ms = now;
}

int main(void) {
    signal(SIGINT, on_sigint);

    printf("Wuji SDK version: %s\n", wuji_version());

    WujiInitOptions opts = { .log_level = 3 };
    if (wuji_init(&opts) != WUJI_STATUS_OK) {
        fprintf(stderr, "wuji_init failed: %s\n", wuji_last_error());
        return 1;
    }

    /* Scan and connect to the first Wuji Hand (selected by device_id). */
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
    size_t idx = n_dev;
    for (size_t i = 0; i < n_dev; i++) {
        printf("  SN=%s  Type=%s  Address=%s\n", list[i].serial_number, list[i].model, list[i].address);
        if (idx == n_dev && list[i].device_id == WUJI_DEVICE_TYPE_WUJI_HAND) idx = i;
    }
    if (idx == n_dev) {
        printf("No Wuji Hand found among the scanned devices\n");
        wuji_discovered_free(list, n_dev);
        wuji_shutdown();
        return 0;
    }
    char sn[64];
    snprintf(sn, sizeof(sn), "%s", list[idx].serial_number);
    wuji_discovered_free(list, n_dev);

    /* Connect by USB serial number. */
    struct WujiDevice *dev = NULL;
    WujiStatus st = wuji_hand_connect_sn(sn, NULL, &dev);
    if (st != WUJI_STATUS_OK) {
        fprintf(stderr, "wuji_hand_connect_sn(\"%s\") failed: %s\n", sn, wuji_last_error());
        wuji_shutdown();
        return 1;
    }

    /* Raw handedness encoding: 0=Right, 1=Left. */
    uint8_t handedness = 0xFF;
    (void)wuji_hand_get_handedness(dev, &handedness);
    printf("Connected: %s (%s)\n", sn,
           handedness == 0 ? "right" : handedness == 1 ? "left" : "unknown");

    int exit_code = 0;
    ctx_t ctx = { .total = 0, .window_frames = 0, .window_start_ms = now_ms() };
    struct WujiSub *sub = NULL;
    st = wuji_hand_subscribe_joint_states(dev, on_joint_states, &ctx, &sub);
    if (st != WUJI_STATUS_OK) {
        fprintf(stderr, "wuji_hand_subscribe_joint_states: %s\n", wuji_last_error());
        exit_code = 1;
        goto cleanup;
    }
    printf("Subscribed to joint_states. Ctrl+C to stop.\n\n");

    while (!g_stop) usleep(50000); /* 50 ms; reports print from the callback */
    printf("\n(interrupted)\n");

    wuji_sub_close(sub);

cleanup:
    wuji_dev_disconnect(dev);
    wuji_dev_release(dev);
    wuji_shutdown();
    return exit_code;
}
