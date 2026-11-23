/*
 * Purpose:
 *   Publish a cosine sweep in position mode.
 * Prerequisites:
 *   Install the C SDK and use firmware 3.0.0 or later. All 20 joints must be
 *   online. Keep the hand clear of people and objects.
 * Usage:
 *   Build with CMake as described in ../README.md, selecting target 8_position_mode.
 *   ./build/8_position_mode
 *   ./build/8_position_mode --sn DEVICE_SN
 * Expected Behavior:
 *   The SDK may report a thumb abduction stall warning during this motion.
 *   Disable, select position mode, scale each joint's effort limit to 80%
 *   of its current value, verify it, set the command cutoff to 40 Hz, and enable.
 *   Ramp to zero in 1 second at 1 kHz, then move all joints from zero to
 *   0.35 rad and back twice at 120 Hz. Disable, restore the original effort
 *   limits, and disconnect on completion, error, or Ctrl-C.
 *   Exit 0 on completion or Ctrl-C, 1 on execution or cleanup failure,
 *   and 2 on invalid arguments. Errors go to stderr.
 */

#define _POSIX_C_SOURCE 200809L

#include "wuji_sdk.h"

#include <errno.h>
#include <math.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define JOINT_COUNT WUJI_HAND_2_JOINT_COUNT
#define PUB_HZ 120
#define RAMP_HZ 1000
#define AMPLITUDE 0.35f
#define PERIOD_SECONDS 4
#define CYCLES 2
#define COMMAND_CUTOFF_HZ 40.0f
#define EFFORT_LIMIT_SCALE 0.8f

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

static void sleep_seconds(double seconds) {
    struct timespec duration = {
        .tv_sec = (time_t)seconds,
        .tv_nsec = (long)((seconds - (time_t)seconds) * 1000000000.0)
    };
    nanosleep(&duration, NULL);
}

static double now_seconds(void) {
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (double)now.tv_sec + (double)now.tv_nsec / 1000000000.0;
}

typedef struct {
    atomic_int ready;
    atomic_int failed;
    atomic_int closing;
    float positions[JOINT_COUNT];
} Feedback;

static int frame_ok(WujiFrameKind kind, const void *frame, Feedback *feedback) {
    if (kind == WUJI_FRAME_KIND_END && atomic_load(&feedback->closing)) return 0;
    if (kind == WUJI_FRAME_KIND_LAG) {
        const char *error = wuji_last_error();
        fprintf(stderr, "Warning: %s\n", error ? error : "subscription frames were dropped");
        return 0;
    }
    if (kind == WUJI_FRAME_KIND_OK && frame != NULL) return 1;
    const char *error = wuji_last_error();
    fprintf(stderr, "feedback stream failed (%d): %s\n", kind,
            error ? error : "stream ended or data was lost");
    atomic_store(&feedback->failed, 1);
    return 0;
}

static void on_diagnostics(WujiFrameKind kind, const WujiJointDiagnosticsFrame *frame, void *data) {
    Feedback *feedback = data;
    if (!frame_ok(kind, frame, feedback)) return;
    atomic_store(&feedback->ready, 0);
    if (frame->joints_len != JOINT_COUNT) return;
    for (size_t i = 0; i < frame->joints_len; ++i) {
        WujiJointStatus status;
        if (wuji_hand_2_decode_joint_status(frame->joints[i].status_word, &status) != WUJI_STATUS_OK) {
            report_sdk_error("wuji_hand_2_decode_joint_status");
            atomic_store(&feedback->failed, 1);
            return;
        }
        if (!status.enabled) return;
    }
    atomic_store(&feedback->ready, 1);
}

static void on_positions(WujiFrameKind kind, const WujiJointStateFrame *frame, void *data) {
    Feedback *feedback = data;
    if (!frame_ok(kind, frame, feedback) || atomic_load(&feedback->ready)) return;
    if (frame->joints_len != JOINT_COUNT) return;
    unsigned seen = 0;
    for (size_t i = 0; i < frame->joints_len; ++i) {
        uint8_t index;
        if (wuji_hand_2_nid_to_joint_index(frame->joints[i].nid, &index) != WUJI_STATUS_OK) {
            report_sdk_error("wuji_hand_2_nid_to_joint_index");
            atomic_store(&feedback->failed, 1);
            return;
        }
        if ((seen & (1u << index)) || !isfinite(frame->joints[i].position)) {
            fprintf(stderr, "Error: duplicate joint or invalid position in joint_states\n");
            atomic_store(&feedback->failed, 1);
            return;
        }
        seen |= 1u << index;
        feedback->positions[index] = frame->joints[i].position;
    }
    atomic_store(&feedback->ready, 1);
}

static int wait_feedback(WujiSub *sub, Feedback *feedback, const char *expected) {
    double deadline = now_seconds() + 5.0;
    while (!stop_requested && !atomic_load(&feedback->ready) &&
           !atomic_load(&feedback->failed) && now_seconds() < deadline) {
        sleep_seconds(0.01);
    }
    atomic_store(&feedback->closing, 1);
    wuji_sub_close(sub);
    if (atomic_load(&feedback->failed)) return 1;
    if (stop_requested) return 0;
    if (!atomic_load(&feedback->ready)) {
        fprintf(stderr, "Error: timed out waiting for %s\n", expected);
        return 1;
    }
    return 0;
}

static int wait_enabled(WujiDevice *device) {
    Feedback feedback = {0};
    WujiSub *sub = NULL;
    if (wuji_hand_2_subscribe_joint_diagnostics(device, on_diagnostics, &feedback, &sub) != WUJI_STATUS_OK) {
        report_sdk_error("wuji_hand_2_subscribe_joint_diagnostics");
        return 1;
    }
    return wait_feedback(sub, &feedback, "all joints enabled");
}

static int read_positions(WujiDevice *device, float *positions) {
    Feedback feedback = {0};
    WujiSub *sub = NULL;
    if (wuji_hand_2_subscribe_joint_states(device, on_positions, &feedback, &sub) != WUJI_STATUS_OK) {
        report_sdk_error("wuji_hand_2_subscribe_joint_states");
        return 1;
    }
    int result = wait_feedback(sub, &feedback, "positions from all 20 joints");
    if (result == 0 && !stop_requested) memcpy(positions, feedback.positions, sizeof(feedback.positions));
    return result;
}

static int read_effort_limits(WujiDevice *device, float limits[JOINT_COUNT]) {
    uint32_t online = 0;
    if (wuji_hand_2_get_all_effort_limit(device, limits, &online) != WUJI_STATUS_OK) {
        report_sdk_error("wuji_hand_2_get_all_effort_limit");
        return 1;
    }
    if (online != (1u << JOINT_COUNT) - 1u) {
        fprintf(stderr, "Error: effort_limit requires all 20 joints online\n");
        return 1;
    }
    for (int i = 0; i < JOINT_COUNT; ++i) {
        if (!isfinite(limits[i]) || limits[i] < 0.0f) {
            fprintf(stderr, "Error: invalid effort_limit for joint %d\n", i);
            return 1;
        }
    }
    return 0;
}

static int write_effort_limits(WujiDevice *device, const float limits[JOINT_COUNT]) {
    if (wuji_hand_2_set_all_effort_limit_per_joint(device, limits) != WUJI_STATUS_OK) {
        report_sdk_error("wuji_hand_2_set_all_effort_limit_per_joint");
        return 1;
    }
    float reported[JOINT_COUNT];
    if (read_effort_limits(device, reported)) return 1;
    for (int i = 0; i < JOINT_COUNT; ++i) {
        float tolerance = fmaxf(1e-6f, 1e-5f * fmaxf(fabsf(limits[i]), fabsf(reported[i])));
        if (fabsf(limits[i] - reported[i]) > tolerance) {
            fprintf(stderr, "Error: effort_limit readback mismatch for joint %d\n", i);
            return 1;
        }
    }
    return 0;
}

#define CHECK(call) do { \
    if (stop_requested) goto cleanup; \
    if ((call) != WUJI_STATUS_OK) { \
        report_sdk_error(#call); \
        result = 1; \
        goto cleanup; \
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
    int hand_checked = 0;
    int restore_limits = 0;
    float original_limits[JOINT_COUNT];
    WujiDevice *device = NULL;
    WujiJointCommandPublisher *publisher = NULL;
    if (wuji_init(NULL) != WUJI_STATUS_OK) {
        report_sdk_error("wuji_init");
        return 1;
    }
    if (stop_requested) goto cleanup;
    CHECK(connect_device(sn, &device));
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
    hand_checked = 1;
    char serial_number[64];
    if (wuji_dev_serial_number(device, serial_number, sizeof(serial_number)) < 0) {
        report_sdk_error("wuji_dev_serial_number");
        result = 1;
        goto cleanup;
    }
    printf("Connected: %s\n", serial_number);
    // Disable joints before switching control mode.
    CHECK(wuji_hand_2_disable(device, NULL));
    if (stop_requested) goto cleanup;
    CHECK(wuji_hand_2_set_control_mode(device, WUJI_CONTROL_MODE_POSITION));
    result = read_effort_limits(device, original_limits);
    if (result || stop_requested) goto cleanup;
    float scaled_limits[JOINT_COUNT];
    for (int i = 0; i < JOINT_COUNT; ++i) scaled_limits[i] = original_limits[i] * EFFORT_LIMIT_SCALE;
    restore_limits = 1;
    result = write_effort_limits(device, scaled_limits);
    if (result || stop_requested) goto cleanup;
    CHECK(wuji_hand_2_set_position_command_low_pass_cutoff_frequency(device, COMMAND_CUTOFF_HZ));
    if (stop_requested) goto cleanup;
    CHECK(wuji_hand_2_enable(device, NULL));
    result = wait_enabled(device);
    if (result || stop_requested) goto cleanup;
    float start[JOINT_COUNT];
    result = read_positions(device, start);
    if (result || stop_requested) goto cleanup;
    CHECK(wuji_hand_2_joint_command_publish(device, &publisher));
    double started = now_seconds();
    for (int step = 1; step <= RAMP_HZ && !stop_requested; ++step) {
        float scale = (float)(0.5 * (1.0 + cos(3.141592653589793 * step / RAMP_HZ)));
        WujiJointCommand commands[JOINT_COUNT] = {{0}};
        for (int i = 0; i < JOINT_COUNT; ++i) commands[i].position = start[i] * scale;
        CHECK(wuji_joint_command_publisher_send(publisher, commands));
        double wait = started + (double)step / RAMP_HZ - now_seconds();
        if (wait > 0) sleep_seconds(wait);
    }
    started = now_seconds();
    const int steps = PERIOD_SECONDS * PUB_HZ * CYCLES;
    for (int step = 0; step <= steps && !stop_requested; ++step) {
        WujiJointCommand commands[JOINT_COUNT] = {{0}};
        float position = (float)(AMPLITUDE * 0.5 *
            (1.0 - cos(2.0 * 3.141592653589793 * step / (PERIOD_SECONDS * PUB_HZ))));
        for (int i = 0; i < JOINT_COUNT; ++i) commands[i].position = position;
        CHECK(wuji_joint_command_publisher_send(publisher, commands));
        double wait = started + (double)(step + 1) / PUB_HZ - now_seconds();
        if (step < steps && wait > 0) sleep_seconds(wait);
    }

cleanup:
    if (publisher != NULL) wuji_joint_command_publisher_close(publisher);
    if (hand_checked) {
        if (wuji_hand_2_disable(device, NULL) != WUJI_STATUS_OK) {
            report_sdk_error("wuji_hand_2_disable");
            if (restore_limits) {
                fprintf(stderr, "Warning: original effort limits may not be restored. "
                        "Check and restore them, or power-cycle the device to restore default limits, "
                        "before running this example again.\n");
            }
            result = 1;
        } else if (restore_limits && write_effort_limits(device, original_limits)) {
            fprintf(stderr, "Warning: original effort limits may not be restored. "
                    "Check and restore them, or power-cycle the device to restore default limits, "
                    "before running this example again.\n");
            result = 1;
        }
    }
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
