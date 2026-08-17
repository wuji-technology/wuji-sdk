/*
 * Wuji SDK C — stream rate control (wuji_sub_set_rate).
 *
 * C port of public/examples/python/wuji_hand_2/4.set_stream_rate.py.
 *
 * `wuji_sub_set_rate(sub, hz, &actual_hz)` lowers the device push rate of a
 * subscribed stream and reports the applied rate via `actual_hz` (it may be
 * quantized to a device-supported rate). Pass 0 to restore the device
 * default.
 *
 * Rate-adjustable streams on the Wuji Hand 2: joint_states,
 * joint_diagnostics, imu, fingertip/{thumb,index,middle,ring,pinky}/data.
 * joint_states and joint_diagnostics share the same device stream, so changing
 * the rate through either subscription changes both.
 *
 * The rate belongs to the stream, not to your handle: it applies to every
 * subscriber of that stream, and resets to the device default when the stream
 * is released or the device reconnects.
 *
 * Build: point CMake at your extracted SDK tarball (see ../README.md):
 *          cmake -S . -B build \
 *            -DWUJI_SDK_INCLUDE_DIR=/path/to/sdk/include \
 *            -DWUJI_SDK_LIB=/path/to/sdk/lib/libwuji_sdk_c.so
 *          cmake --build build
 * Run:   ./build/4_set_stream_rate
 */
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <unistd.h>

#include "wuji_sdk.h"

#define REQUESTED_HZ 100

static void on_joint_states(WujiFrameKind kind, const WujiJointStateFrame *frame, void *ud) {
    (void)ud;
    if (kind == WUJI_FRAME_KIND_OK && frame)
        printf("joint_states frame: %zu joints\n", frame->joints_len);
}

int main(void) {
    WujiInitOptions opts = { .log_level = 3 };
    if (wuji_init(&opts) != WUJI_STATUS_OK) {
        fprintf(stderr, "wuji_init failed: %s\n", wuji_last_error());
        return 1;
    }

    WujiDiscovered *list = NULL;
    size_t n = 0;
    if (wuji_scan(&list, &n) != WUJI_STATUS_OK || n == 0) {
        fprintf(stderr, "No devices found\n");
        wuji_discovered_free(list, n);
        wuji_shutdown();
        return 1;
    }

    /* Pick the first Wuji Hand 2 among the scanned devices. */
    int idx = -1;
    for (size_t i = 0; i < n; i++) {
        if (list[i].device_id == WUJI_DEVICE_TYPE_WUJI_HAND_2) { idx = (int)i; break; }
    }
    if (idx < 0) {
        fprintf(stderr, "No Wuji Hand 2 found\n");
        wuji_discovered_free(list, n);
        wuji_shutdown();
        return 1;
    }

    WujiConnectTarget tgt = { .kind = WUJI_CONNECT_TARGET_KIND_SN, .value = list[idx].serial_number };
    struct WujiDevice *dev = NULL;
    WujiStatus st = wuji_connect(&tgt, "hand", NULL, &dev);
    wuji_discovered_free(list, n);
    if (st != WUJI_STATUS_OK) {
        fprintf(stderr, "wuji_connect failed: %s\n", wuji_last_error());
        wuji_shutdown();
        return 1;
    }

    /* Keep the handle open for as long as you want its rate to stay in effect. */
    struct WujiSub *sub = NULL;
    if (wuji_hand_2_subscribe_joint_states(dev, on_joint_states, NULL, &sub) != WUJI_STATUS_OK) {
        fprintf(stderr, "subscribe failed: %s\n", wuji_last_error());
        wuji_dev_disconnect(dev);
        wuji_dev_release(dev);
        wuji_shutdown();
        return 1;
    }
    /* To try another root, subscribe to imu and call wuji_sub_set_rate on it. */

    uint16_t actual = 0;
    st = wuji_sub_set_rate(sub, REQUESTED_HZ, &actual);
    if (st != WUJI_STATUS_OK) {
        fprintf(stderr, "wuji_sub_set_rate failed: %s\n", wuji_last_error());
    } else {
        printf("joint_states: requested %d Hz, device applied %u Hz\n", REQUESTED_HZ, actual);
    }
    sleep(2); /* frames arrive on an SDK worker thread */

    uint16_t restored = 0;
    if (wuji_sub_set_rate(sub, 0, &restored) == WUJI_STATUS_OK)
        printf("restored to the device default: %u Hz\n", restored);

    wuji_sub_close(sub);
    wuji_dev_disconnect(dev);
    wuji_dev_release(dev);
    wuji_shutdown();
    return 0;
}
