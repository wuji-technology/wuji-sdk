/*
 * Purpose:
 *   Set a subscription stream rate.
 * Prerequisites:
 *   Install the C SDK and connect a reachable Wuji Glove.
 * Usage:
 *   Build with CMake as described in ../README.md, selecting 6_set_stream_rate.
 *   ./build/glove/6_set_stream_rate
 *   ./build/glove/6_set_stream_rate --sn DEVICE_SN
 * Expected Behavior:
 *   Request 30 Hz for emf_poses, print five frames within 5 seconds, then restore the default rate.
 *   The applied rate affects every subscriber of this stream.
 *   Exit 0 on completion or Ctrl-C, 1 on execution or cleanup failure,
 *   and 2 on invalid arguments. Errors go to stderr.
 */
#define _POSIX_C_SOURCE 200809L

#include "wuji_sdk.h"
#include <time.h>

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static volatile sig_atomic_t stop_requested = 0;

static void on_sigint(int signum) {
    (void)signum;
    stop_requested = 1;
}

static void report_sdk_error(const char *operation) {
    const char *error = wuji_last_error();
    fprintf(stderr, "%s failed: %s\n", operation, error ? error : "Unknown SDK error");
}

static WujiStatus connect_device(const char *sn, WujiDevice **device) {
    if (sn != NULL) {
        WujiConnectTarget target = {.kind = WUJI_CONNECT_TARGET_KIND_SN, .value = sn};
        return wuji_connect(&target, "example", NULL, device);
    }
    return wuji_auto_connect("example", device);
}

#include <stdatomic.h>
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

static void sleep_ms(unsigned ms) {
#ifdef _WIN32
    Sleep(ms);
#else
    struct timespec duration = {.tv_sec = ms / 1000, .tv_nsec = (long)(ms % 1000) * 1000000L};
    nanosleep(&duration, NULL);
#endif
}

static double now_seconds(void) {
#ifdef _WIN32
    return (double)GetTickCount64() / 1000.0;
#else
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (double)now.tv_sec + (double)now.tv_nsec / 1000000000.0;
#endif
}

static atomic_int stream_failed = 0;
static atomic_int closing = 0;

static int frame_ok(WujiFrameKind kind, const void *frame) {
    if (kind == WUJI_FRAME_KIND_END && atomic_load(&closing)) return 0;
    if (kind == WUJI_FRAME_KIND_LAG) {
        const char *error = wuji_last_error();
        fprintf(stderr, "Warning: %s\n", error ? error : "subscription frames were dropped");
        return 0;
    }
    if (kind == WUJI_FRAME_KIND_OK && frame != NULL) return 1;
    fprintf(stderr, "subscription failed (%d): %s\n", kind,
            wuji_last_error() ? wuji_last_error() : "stream ended or data was lost");
    atomic_store(&stream_failed, 1);
    return 0;
}

static atomic_int frames = 0;
static atomic_int sampling = 0;
static void on_poses(WujiFrameKind kind, const WujiEmfPoseArray *frame, void *data) {
    (void)data;
    if (!frame_ok(kind, frame)) return;
    if (!atomic_load(&sampling)) return;
    if (atomic_fetch_add(&frames, 1) < 5) printf("emf_poses: %zu poses\n", frame->poses_len);
}

#define CHECK(call) do { \
    if (stop_requested) goto cleanup; \
    if ((call) != WUJI_STATUS_OK) { \
        report_sdk_error(#call); \
        result = 1; goto cleanup; \
    } \
} while (0)

int main(int argc, char **argv) {
    const char *sn = NULL;

    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--sn") == 0 && i + 1 < argc && argv[i + 1][0] != '-') {
            sn = argv[++i];
        } else if (strncmp(argv[i], "--sn=", 5) == 0) {
            sn = argv[i] + 5;
        } else {
            fprintf(stderr, "Invalid arguments. Usage: %s [--sn DEVICE_SN]\n", argv[0]);
            return 2;
        }
        if (sn != NULL && sn[0] == '\0') {
            fprintf(stderr, "--sn must not be empty\n");
            return 2;
        }
    }


    if (signal(SIGINT, on_sigint) == SIG_ERR) {
        fprintf(stderr, "install Ctrl-C handler failed: %s\n", strerror(errno));
        return 1;
    }

    int result = 0;
    WujiSub *sub = NULL;
    int rate_changed = 0;
    uint16_t actual = 0;
    int initialized = 0;
    WujiDevice *device = NULL;
    char *serial_number = NULL;
    WujiDeviceType device_type;
    int32_t length;

    if (wuji_init(NULL) != WUJI_STATUS_OK) {
        report_sdk_error("wuji_init");
        result = 1;
        goto cleanup;
    }
    initialized = 1;
    if (stop_requested) goto cleanup;
    if (connect_device(sn, &device) != WUJI_STATUS_OK) {
        report_sdk_error("connect_device");
        result = 1;
        goto cleanup;
    }
    if (stop_requested) goto cleanup;

    if (wuji_dev_device_type(device, &device_type) != WUJI_STATUS_OK) {
        report_sdk_error("wuji_dev_device_type");
        result = 1;
        goto cleanup;
    }
    if (device_type != WUJI_DEVICE_TYPE_WUJI_GLOVE) {
        fprintf(stderr, "Error: expected Wuji Glove, got device type %d\n", (int)device_type);
        result = 1;
        goto cleanup;
    }
    length = wuji_dev_serial_number(device, NULL, 0);
    if (length < 0) {
        report_sdk_error("wuji_dev_serial_number");
        result = 1;
        goto cleanup;
    }
    serial_number = (char *)malloc((size_t)length + 1);
    if (serial_number == NULL) {
        fprintf(stderr, "Error: out of memory\n");
        result = 1;
        goto cleanup;
    }
    if (wuji_dev_serial_number(device, serial_number, (size_t)length + 1) < 0) {
        report_sdk_error("wuji_dev_serial_number");
        result = 1;
        goto cleanup;
    }
    printf("Connected: %s\n", serial_number);

    CHECK(wuji_glove_subscribe_emf_poses(device, on_poses, NULL, &sub));
    CHECK(wuji_sub_set_rate(sub, 30, &actual));
    rate_changed = 1;
    atomic_store(&sampling, 1);
    printf("emf_poses: requested 30 Hz, device applied %u Hz\n", actual);
    double deadline = now_seconds() + 5.0;
    while (!stop_requested && !atomic_load(&stream_failed) && atomic_load(&frames) < 5) {
        if (now_seconds() >= deadline) {
            fprintf(stderr, "receive emf_poses failed: timed out waiting for five frames\n");
            result = 1;
            goto cleanup;
        }
        sleep_ms(10);
    }
cleanup:
    if (sub != NULL) {
        if (rate_changed) {
            if (wuji_sub_set_rate(sub, 0, &actual) != WUJI_STATUS_OK) {
                report_sdk_error("wuji_sub_set_rate"); result = 1;
            } else printf("Restored default rate: %u Hz\n", actual);
        }
        atomic_store(&closing, 1);
        wuji_sub_close(sub);
    }
    if (atomic_load(&stream_failed)) result = 1;
    free(serial_number);
    if (device != NULL) {
        if (wuji_dev_disconnect(device) != WUJI_STATUS_OK) {
            report_sdk_error("wuji_dev_disconnect");
            result = 1;
        }
        wuji_dev_release(device);
    }
    if (initialized) wuji_shutdown();
    return result;
}
