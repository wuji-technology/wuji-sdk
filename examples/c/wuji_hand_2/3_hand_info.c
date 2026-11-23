/*
 * Purpose:
 *   Read hand identity, joint effort limits, and one diagnostics snapshot.
 * Prerequisites:
 *   Install the C SDK.
 *   A reachable Wuji Hand 2. Automatic connection requires exactly one
 *   discovered device; use --sn when several are present.
 * Usage:
 *   Build with CMake as described in ../README.md, selecting target 3_hand_info.
 *   ./build/3_hand_info
 *   ./build/3_hand_info --sn DEVICE_SN
 * Expected Behavior:
 *   Print handedness, IP, online count, per-joint limits, and diagnostics with fault descriptions.
 *   This example does not move the hand.
 *   Exit 0 on completion or Ctrl-C with successful cleanup, 1 on an operation
 *   or cleanup failure, and 2 on invalid arguments. Errors go to stderr.
 */

#define _POSIX_C_SOURCE 200809L

#include "wuji_sdk.h"

#include <errno.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdatomic.h>
#include <time.h>
#include <unistd.h>

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

typedef struct {
    atomic_int ready;
    atomic_int failed;
    atomic_int closing;
    WujiJointDiagnosticsEntry joints[WUJI_HAND_2_JOINT_COUNT];
    size_t count;
} Snapshot;

static void on_diagnostics(WujiFrameKind kind, const WujiJointDiagnosticsFrame *frame, void *data) {
    Snapshot *snapshot = data;
    if (kind == WUJI_FRAME_KIND_END && atomic_load(&snapshot->closing)) return;
    if (kind == WUJI_FRAME_KIND_LAG) {
        const char *error = wuji_last_error();
        fprintf(stderr, "Warning: %s\n", error ? error : "subscription frames were dropped");
        return;
    }
    if (kind != WUJI_FRAME_KIND_OK || frame == NULL || frame->joints_len > WUJI_HAND_2_JOINT_COUNT) {
        fprintf(stderr, "diagnostics stream failed (%d): %s\n", kind,
                wuji_last_error() ? wuji_last_error() : "stream ended or data was lost");
        atomic_store(&snapshot->failed, 1);
        return;
    }
    if (atomic_load(&snapshot->ready)) return;
    snapshot->count = frame->joints_len;
    for (size_t i = 0; i < frame->joints_len; ++i) snapshot->joints[i] = frame->joints[i];
    atomic_store(&snapshot->ready, 1);
}

static double now_seconds(void) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (double)now.tv_sec + (double)now.tv_nsec / 1000000000.0;
}

#define CHECK(call) do { \
    if (stop_requested) goto cleanup; \
    if ((call) != WUJI_STATUS_OK) { \
        report_sdk_error(#call); result = 1; goto cleanup; \
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
    int initialized = 0;
    WujiDevice *device = NULL;
    char *serial_number = NULL;
    char *ip = NULL;
    WujiSub *sub = NULL;
    Snapshot snapshot = {0};
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
    if (device_type != WUJI_DEVICE_TYPE_WUJI_HAND_2) {
        fprintf(stderr, "Error: expected Wuji Hand 2, got device type %d\n", (int)device_type);
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
    WujiHandedness handedness;
    CHECK(wuji_hand_2_get_handedness(device, &handedness));
    printf("Handedness: %s\n", handedness == WUJI_HANDEDNESS_LEFT ? "left" : "right");
    size_t needed = 0;
    if (wuji_hand_2_get_ip(device, NULL, 0, &needed) != WUJI_STATUS_ERR_BUFFER_TOO_SMALL) {
        report_sdk_error("wuji_hand_2_get_ip"); result = 1; goto cleanup;
    }
    ip = malloc(needed);
    if (ip == NULL) {
        fprintf(stderr, "Error: out of memory\n"); result = 1; goto cleanup;
    }
    CHECK(wuji_hand_2_get_ip(device, ip, needed, NULL));
    printf("IP: %s\n", ip);
    uint8_t count;
    CHECK(wuji_hand_2_online_joints_count(device, &count));
    printf("Online joints: %u\n", count);
    printf("effort_limit units: A in MIT mode; N m in Position mode.\n");
    float limits[WUJI_HAND_2_JOINT_COUNT];
    uint32_t online;
    CHECK(wuji_hand_2_get_all_effort_limit(device, limits, &online));
    for (int i = 0; i < WUJI_HAND_2_JOINT_COUNT; ++i) {
        /* Only online slots contain a valid limit. */
        if (WUJI_JOINT_ONLINE(online, i)) printf("Joint %d: %.3f\n", i, (double)limits[i]);
        else printf("Joint %d: offline\n", i);
    }
    CHECK(wuji_hand_2_subscribe_joint_diagnostics(device, on_diagnostics, &snapshot, &sub));
    double deadline = now_seconds() + 5.0;
    while (!stop_requested && !atomic_load(&snapshot.ready) &&
           !atomic_load(&snapshot.failed) && now_seconds() < deadline) nanosleep(&(struct timespec){.tv_nsec = 10000000L}, NULL);
    if (atomic_load(&snapshot.failed)) { result = 1; goto cleanup; }
    if (stop_requested) goto cleanup;
    if (!atomic_load(&snapshot.ready)) {
        fprintf(stderr, "Error: timed out waiting for joint diagnostics\n");
        result = 1; goto cleanup;
    }
    for (size_t i = 0; i < snapshot.count; ++i) {
        const WujiJointDiagnosticsEntry *joint = &snapshot.joints[i];
        WujiJointStatus status;
        CHECK(wuji_hand_2_decode_joint_status(joint->status_word, &status));
        printf("NID %u: ext_state=%u, enabled=%d, position_limit_active=%d, "
               "velocity_limit_active=%d, current_limit_active=%d, "
               "current=%.2f A, voltage=%.1f V, temperature=%.1f C, error=0x%04X\n",
               joint->nid, status.ext_state, status.enabled, status.position_limit_active,
               status.velocity_limit_active, status.current_limit_active,
               (double)joint->current, (double)joint->vbus_v_fb,
               (double)joint->mcu_temp_c_fb, joint->error_code_current);
        if (joint->error_code_current != 0) {
            WujiErrorInfo info;
            WujiStatus lookup = wuji_hand_2_describe_error(joint->error_code_current, &info);
            if (lookup == WUJI_STATUS_ERR_NOT_FOUND) {
                printf("  Unknown fault code\n");
            } else if (lookup != WUJI_STATUS_OK) {
                report_sdk_error("wuji_hand_2_describe_error"); result = 1; goto cleanup;
            } else {
                printf("  %s: %s, %s\n", info.name, info.severity, info.clear_policy);
                printf("  Description: %s\n  Cause: %s\n  Resolution: %s\n",
                       info.desc, info.cause, info.resolution);
            }
        }
    }

cleanup:
    if (sub != NULL) {
        atomic_store(&snapshot.closing, 1);
        wuji_sub_close(sub);
    }
    if (atomic_load(&snapshot.failed)) result = 1;
    free(ip);
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
