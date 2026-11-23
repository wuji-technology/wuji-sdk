/*
 * Purpose:
 *   Subscribe to Wuji Hand 2 joint states and IMU data with callbacks.
 * Prerequisites:
 *   Install the C SDK and make one Wuji Hand 2 reachable. Use --sn if several
 *   devices are discoverable.
 * Usage:
 *   Build with CMake as described in ../README.md, selecting target 1_subscribe_callback.
 *   ./build/1_subscribe_callback
 *   ./build/1_subscribe_callback --sn DEVICE_SN
 * Expected Behavior:
 *   Print joint NID/position pairs and IMU acceleration/angular velocity until Ctrl-C.
 *   No motor control. Exit 0 on completion or Ctrl-C, 1 on execution or cleanup
 *   failure, and 2 on invalid arguments. Errors go to stderr.
 *   The callback runs on an SDK worker thread.
 */

#define _POSIX_C_SOURCE 200809L

#include "wuji_sdk.h"
#include <time.h>

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <stdatomic.h>
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

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

static void wait_tick(void) {
#ifdef _WIN32
    Sleep(10);
#else
    nanosleep(&(struct timespec){.tv_nsec = 10000000L}, NULL);
#endif
}

typedef struct {
    atomic_int failed;
    atomic_int closing;
} StreamState;

static void on_joint_states(WujiFrameKind kind, const WujiJointStateFrame *frame, void *data) {
    StreamState *state = data;
    if (kind == WUJI_FRAME_KIND_END && atomic_load(&state->closing)) return;
    if (kind == WUJI_FRAME_KIND_LAG) {
        const char *error = wuji_last_error();
        fprintf(stderr, "Warning: %s\n", error ? error : "subscription frames were dropped");
        return;
    }
    if (kind != WUJI_FRAME_KIND_OK || frame == NULL) {
        const char *error = wuji_last_error();
        fprintf(stderr, "joint_states stream failed (%d): %s\n", kind,
                error ? error : "stream ended or data was lost");
        atomic_store(&state->failed, 1);
        return;
    }
    printf("seq=%u joints=[", frame->header.seq);
    for (size_t i = 0; i < frame->joints_len; ++i) {
        printf("%s(%u, %.3f)", i ? ", " : "", frame->joints[i].nid, frame->joints[i].position);
    }
    printf("]\n");
}

static void on_imu(WujiFrameKind kind, const WujiImuData *frame, void *data) {
    StreamState *state = data;
    if (kind == WUJI_FRAME_KIND_END && atomic_load(&state->closing)) return;
    if (kind == WUJI_FRAME_KIND_LAG) {
        const char *error = wuji_last_error();
        fprintf(stderr, "Warning: %s\n", error ? error : "subscription frames were dropped");
        return;
    }
    if (kind != WUJI_FRAME_KIND_OK || frame == NULL) {
        const char *error = wuji_last_error();
        fprintf(stderr, "imu stream failed (%d): %s\n", kind,
                error ? error : "stream ended or data was lost");
        atomic_store(&state->failed, 1);
        return;
    }
    printf("IMU seq=%u acceleration=[%.3f, %.3f, %.3f] angular_velocity=[%.3f, %.3f, %.3f]\n",
           frame->header.seq,
           frame->linear_acceleration.x, frame->linear_acceleration.y, frame->linear_acceleration.z,
           frame->angular_velocity.x, frame->angular_velocity.y, frame->angular_velocity.z);
}

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
        if (sn[0] == '\0') {
            fprintf(stderr, "--sn must not be empty\n");
            return 2;
        }
    }

    if (signal(SIGINT, on_sigint) == SIG_ERR) {
        fprintf(stderr, "install Ctrl-C handler failed: %s\n", strerror(errno));
        return 1;
    }

    int result = 0;
    WujiDevice *device = NULL;
    WujiSub *sub = NULL;
    WujiSub *imu_sub = NULL;
    StreamState state = {0};
    if (wuji_init(NULL) != WUJI_STATUS_OK) {
        report_sdk_error("wuji_init");
        return 1;
    }
    if (stop_requested) goto cleanup;
    if (connect_device(sn, &device) != WUJI_STATUS_OK) {
        report_sdk_error("connect_device");
        result = 1;
        goto cleanup;
    }
    if (stop_requested) goto cleanup;
    WujiDeviceType device_type;
    if (wuji_dev_device_type(device, &device_type) != WUJI_STATUS_OK) {
        report_sdk_error("wuji_dev_device_type");
        result = 1;
        goto cleanup;
    }
    if (device_type != WUJI_DEVICE_TYPE_WUJI_HAND_2) {
        fprintf(stderr, "Error: expected Wuji Hand 2, got device type %d\n", (int)device_type);
        result = 1;
        goto cleanup;
    }
    char serial_number[64];
    if (wuji_dev_serial_number(device, serial_number, sizeof(serial_number)) < 0) {
        report_sdk_error("wuji_dev_serial_number");
        result = 1;
        goto cleanup;
    }
    printf("Connected: %s\n", serial_number);
    if (stop_requested) goto cleanup;
    if (wuji_hand_2_subscribe_joint_states(device, on_joint_states, &state, &sub) != WUJI_STATUS_OK) {
        report_sdk_error("wuji_hand_2_subscribe_joint_states");
        result = 1;
        goto cleanup;
    }
    if (stop_requested) goto cleanup;
    if (wuji_hand_2_subscribe_imu(device, on_imu, &state, &imu_sub) != WUJI_STATUS_OK) {
        report_sdk_error("wuji_hand_2_subscribe_imu"); result = 1; goto cleanup;
    }
    while (!stop_requested && !atomic_load(&state.failed)) wait_tick();

cleanup:
    atomic_store(&state.closing, 1);
    if (imu_sub != NULL) wuji_sub_close(imu_sub);
    if (sub != NULL) wuji_sub_close(sub);
    if (atomic_load(&state.failed)) result = 1;
    if (device != NULL) {
        if (wuji_dev_disconnect(device) != WUJI_STATUS_OK) {
            report_sdk_error("wuji_dev_disconnect");
            result = 1;
        }
        wuji_dev_release(device);
    }
    wuji_shutdown();
    return result;
}
