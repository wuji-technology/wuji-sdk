/*
 * Wuji Hand — tactile glove status + pressure summary.
 *
 * Auto-detects and connects to a Wuji Hand, then each subscription reports its
 * own stream: the status callback logs detach events, the pressure callback
 * prints a summary every 0.5 s while frames flow. The tactile glove pairs at
 * connect time only — plug it in before connecting; after a re-plug,
 * disconnect and reconnect the hand. Mirrors
 * examples/python/wuji_hand/3.tactile_status.py.
 *
 * Build: see ../README.md.   Run: ./build/3_tactile_status   (Ctrl+C to stop)
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <signal.h>
#include <math.h>
#include <time.h>
#include <unistd.h>
#include <stdatomic.h>

#include "wuji_sdk.h"

#define REPORT_MS 500 /* one summary line every 0.5s, like the Python example */

static volatile sig_atomic_t g_stop = 0;
static void on_sigint(int sig) { (void)sig; g_stop = 1; }

static uint64_t now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL;
}

/* Latest glove state, written by the status subscription worker and read by
 * main for the initial attach report. -1 = no frame yet. */
static _Atomic int g_state = -1;

/* The status stream delivers the current state on first subscription, then
 * pushes a frame only when the state changes. Pairing happens at connect time
 * only, so the state never returns to attached mid-session — this callback is
 * effectively a detach event log. The initial frame (prev == -1) is stored
 * silently; main reports it in the "Connected:" line. */
static void on_status(WujiFrameKind kind, const WujiTactileGloveStatus *s, void *ud) {
    (void)ud;
    if (kind == WUJI_FRAME_KIND_LAG)   { fprintf(stderr, "[tactile_status] lagged\n");                       return; }
    if (kind == WUJI_FRAME_KIND_END)   { fprintf(stderr, "[tactile_status] stream ended\n");                 return; }
    if (kind == WUJI_FRAME_KIND_ERROR) { fprintf(stderr, "[tactile_status] error: %s\n", wuji_last_error()); return; }
    if (kind != WUJI_FRAME_KIND_OK || !s) return;
    int prev = atomic_exchange_explicit(&g_state, (int)s->state, memory_order_relaxed);
    if (prev >= 0 && prev != (int)s->state) {
        printf("[wuji_hand][Tactile] attached=%s (state=%u)%s\n",
               s->state == 1 ? "true" : "false", s->state,
               s->state == 1 ? "" : " — to re-pair, disconnect and reconnect the hand");
        fflush(stdout);
    }
}

/* Pressure summary printed directly from the frame worker thread, throttled to
 * one line per REPORT_MS — mirrors the Python pressure_summary() line. */
typedef struct {
    uint64_t last_print_ms; /* touched only by the (single) frame worker */
} frame_ctx_t;

static void on_pressure_frame(WujiFrameKind kind, const WujiTactileGloveFrame *f, void *ud) {
    frame_ctx_t *ctx = (frame_ctx_t *)ud;
    if (kind == WUJI_FRAME_KIND_LAG)   { fprintf(stderr, "[tactile] lagged\n");                       return; }
    if (kind == WUJI_FRAME_KIND_END)   { fprintf(stderr, "[tactile] stream ended\n");                 return; }
    if (kind == WUJI_FRAME_KIND_ERROR) { fprintf(stderr, "[tactile] error: %s\n", wuji_last_error()); return; }
    if (kind != WUJI_FRAME_KIND_OK || !f) return;

    uint64_t now = now_ms();
    if (now - ctx->last_print_ms < REPORT_MS) return;
    ctx->last_print_ms = now;

    /* Same summary as the Python example: max / mean / first 8 over the
     * FINITE taxels only (frames contain NaN for masked/invalid taxels). */
    float max_v = 0.0f;
    double sum = 0.0;
    size_t n_finite = 0;
    float first8[8] = {0};
    for (size_t i = 0; i < f->pressure_len; i++) {
        float v = f->pressure[i];
        if (!isfinite(v)) continue;
        if (n_finite < 8) first8[n_finite] = v;
        if (n_finite == 0 || v > max_v) max_v = v;
        sum += v;
        n_finite++;
    }
    if (n_finite == 0) {
        printf("[wuji_hand][Tactile] seq=%u pressure=[]\n", f->sequence);
    } else {
        printf("[wuji_hand][Tactile] seq=%u max=%.3f mean=%.3f "
               "first8=[%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n",
               f->sequence,
               (double)max_v, sum / (double)n_finite,
               (double)first8[0], (double)first8[1],
               (double)first8[2], (double)first8[3],
               (double)first8[4], (double)first8[5],
               (double)first8[6], (double)first8[7]);
    }
    fflush(stdout);
}

int main(void) {
    int exit_code = 0;
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

    struct WujiSub *status_sub = NULL;
    struct WujiSub *frame_sub = NULL;
    frame_ctx_t frame_ctx = { .last_print_ms = 0 };

    st = wuji_hand_subscribe_tactile_status(dev, on_status, NULL, &status_sub);
    if (st != WUJI_STATUS_OK) {
        fprintf(stderr, "subscribe_tactile_status: %s\n", wuji_last_error());
        exit_code = 1;
        goto cleanup;
    }

    /* Wait up to 1s for the initial status (delivered on first subscription). */
    {
        uint64_t deadline = now_ms() + 1000;
        while (now_ms() < deadline && atomic_load(&g_state) < 0) usleep(20000);
    }
    bool attached = atomic_load(&g_state) == 1;

    /* Raw handedness encoding: 0=Right, 1=Left. */
    uint8_t handedness = 0xFF;
    (void)wuji_hand_get_handedness(dev, &handedness);
    printf("Connected: %s (%s, tactile_attached=%s)\n", sn,
           handedness == 0 ? "right" : handedness == 1 ? "left" : "unknown",
           attached ? "yes" : "no");

    if (!attached) {
        /* Pairing happens at connect time only: plugging the glove in later
         * will NOT attach it mid-session. */
        printf("No tactile glove is attached. Plug it in, then disconnect and reconnect.\n");
        goto close_subs;
    }

    st = wuji_hand_subscribe_tactile_pressure_frame(dev, on_pressure_frame, &frame_ctx, &frame_sub);
    if (st != WUJI_STATUS_OK) {
        fprintf(stderr, "subscribe_tactile_pressure_frame: %s\n", wuji_last_error());
        exit_code = 1;
        goto close_subs;
    }
    printf("Subscribed to tactile_status and tactile pressure frames. Ctrl+C to stop.\n\n");

    /* All reporting happens in the two callbacks; main just waits for Ctrl+C. */
    while (!g_stop) usleep(20000);
    printf("\n(interrupted)\n");

close_subs:
    wuji_sub_close(frame_sub);  /* NULL-safe */
    wuji_sub_close(status_sub);

cleanup:
    wuji_dev_disconnect(dev);
    wuji_dev_release(dev);
    wuji_shutdown();
    return exit_code;
}
