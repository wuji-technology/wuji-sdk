/*
 * Wuji SDK C — Wuji Hand 2: fingertip sensor info + data.
 *
 * The fingertip sensor is self-describing. This example:
 *   - reads each finger's info with ONE high-level call,
 *     wuji_hand_2_get_fingertip_info() -> WujiFingertipSensorInfo. The SDK
 *     drives the chunked info read internally; the caller never sees the
 *     chunked-read transport. `info.format` is a JSON string describing the
 *     data-frame layout (a point array).
 *   - subscribes to the five per-finger data streams (~100 Hz,
 *     WujiFingertipSensorData). Each frame's `data` is the raw point-array
 *     payload; decode it per `info.format`. This example prints the frame
 *     envelope (seq / info_digest / byte length) rather than the full decode.
 *
 * Build: point CMake at your extracted SDK tarball (see ../README.md):
 *          cmake -S . -B build \
 *            -DWUJI_SDK_INCLUDE_DIR=/path/to/sdk/include \
 *            -DWUJI_SDK_LIB=/path/to/sdk/lib/libwuji_sdk_c.so
 *          cmake --build build
 * Run:   ./build/3_fingertip        (Ctrl+C to stop)
 *
 * THREADING: each callback fires on a dedicated SDK worker thread, NOT main.
 * The `frame` pointer is valid only for the duration of the callback (its heap
 * `data` buffer is freed right after the callback returns) — never stash it.
 */
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>

#include "wuji_sdk.h"

static volatile sig_atomic_t g_stop = 0;
static void on_sigint(int sig) { (void)sig; g_stop = 1; }

/* Per-finger callback state: the finger name and the last print time (touched
 * only by that finger's single worker thread, so no atomics needed). */
typedef struct {
    const char *name;
    uint64_t    last_print_ms;
} finger_ctx_t;

static uint64_t now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL;
}

static void on_fingertip_data(WujiFrameKind kind, const WujiFingertipSensorData *f, void *ud) {
    finger_ctx_t *ctx = (finger_ctx_t *)ud;

    if (kind == WUJI_FRAME_KIND_LAG)   { fprintf(stderr, "[%s] lagged\n", ctx->name); return; }
    if (kind == WUJI_FRAME_KIND_END)   { fprintf(stderr, "[%s] stream ended\n", ctx->name); return; }
    if (kind == WUJI_FRAME_KIND_ERROR) { fprintf(stderr, "[%s] error: %s\n", ctx->name, wuji_last_error()); return; }
    if (kind != WUJI_FRAME_KIND_OK || !f) return;

    /* Throttle printing to ~1 Hz (the stream is ~100 Hz). */
    uint64_t now = now_ms();
    if (now - ctx->last_print_ms < 1000) return;
    ctx->last_print_ms = now;

    /* Decode f->data (f->data_len bytes) per the finger's info.format; here we
     * just show the frame envelope. info_digest binds the frame to a specific
     * info revision (re-GET info if it changes). */
    printf("[%-6s] seq=%u  info_digest=0x%08x  data=%zuB\n",
           ctx->name, f->header.seq, f->info_digest, f->data_len);
    fflush(stdout);
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
    size_t idx = n;
    for (size_t i = 0; i < n; i++)
        if (idx == n && list[i].device_id == WUJI_DEVICE_TYPE_WUJI_HAND_2) idx = i;
    if (idx == n) {
        printf("No Wuji Hand 2 found\n");
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

    /* Read each finger's self-describing info (high-level; chunked read hidden). */
    const char *names[5] = { "thumb", "index", "middle", "ring", "pinky" };
    for (uint8_t i = 0; i < 5; i++) {
        WujiFingertipSensorInfo info;
        if (wuji_hand_2_get_fingertip_info(dev, i, &info) != WUJI_STATUS_OK) {
            fprintf(stderr, "get_fingertip_info(%s): %s\n", names[i], wuji_last_error());
            goto cleanup;
        }
        printf("  %-6s frame_id=%s  device_type=0x%04x  rate=%.0fHz  format=%zuB\n",
               names[i], info.header.frame_id, info.device_type, info.rate_hz, strlen(info.format));
    }

    /* Subscribe to the five per-finger typed data streams. */
    finger_ctx_t ctxs[5] = {
        { names[0], 0 }, { names[1], 0 }, { names[2], 0 }, { names[3], 0 }, { names[4], 0 },
    };
    struct WujiSub *subs[5] = { 0 };
    st  = wuji_hand_2_subscribe_fingertip_thumb_data(dev, on_fingertip_data, &ctxs[0], &subs[0]);
    st |= wuji_hand_2_subscribe_fingertip_index_data(dev, on_fingertip_data, &ctxs[1], &subs[1]);
    st |= wuji_hand_2_subscribe_fingertip_middle_data(dev, on_fingertip_data, &ctxs[2], &subs[2]);
    st |= wuji_hand_2_subscribe_fingertip_ring_data(dev, on_fingertip_data, &ctxs[3], &subs[3]);
    st |= wuji_hand_2_subscribe_fingertip_pinky_data(dev, on_fingertip_data, &ctxs[4], &subs[4]);
    if (st != WUJI_STATUS_OK) {
        fprintf(stderr, "subscribe fingertip data: %s\n", wuji_last_error());
        goto cleanup_subs;
    }
    printf("Subscribed to 5 fingertip data streams. Ctrl+C to stop.\n");
    fflush(stdout);

    while (!g_stop) sleep(1);
    printf("\nStopping...\n");

cleanup_subs:
    for (int i = 0; i < 5; i++)
        if (subs[i]) wuji_sub_close(subs[i]); /* stop workers before releasing the device */
cleanup:
    wuji_dev_disconnect(dev);
    wuji_dev_release(dev);
    wuji_shutdown();
    return 0;
}
