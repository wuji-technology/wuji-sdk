/*
 * Purpose:
 *   Publish joint commands to smoothly return Wuji Hand 2 to zero and hold.
 * Prerequisites:
 *   Install the C SDK. All 20 joints must be online. Keep the hand clear of
 *   people and objects; this example moves every joint.
 * Usage:
 *   Build with CMake as described in ../README.md, selecting target 2_publish.
 *   ./build/2_publish
 *   ./build/2_publish --sn DEVICE_SN
 * Expected Behavior:
 *   Disable, select MIT_V2 mode, set physical gains and per-joint torque limits,
 *   then enable. Publish at 200 Hz: return to zero in 1.5 seconds and hold
 *   for 5 seconds. Disable and disconnect on completion, error, or Ctrl-C.
 *   Exit 0 on completion or Ctrl-C, 1 on execution or cleanup failure,
 *   and 2 on invalid arguments. Errors go to stderr.
 * Documentation:
 *   https://docs.wuji.tech/docs/en/wuji-hand/latest/sdk-practice/publish-command
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
#define PUB_HZ 200
#define HOME_SECONDS 1.5
#define HOLD_SECONDS 5.0
// MIT_V2 starting gains: Kp 0.5..2.0 N*m/rad, Kd 0.01..0.05 N*m*s/rad.
// Retune for the joint and load; these are recommendations, not device limits.
#define KP 1.0f
#define KD 0.02f
static const float TORQUE_LIMITS_NM[JOINT_COUNT] = {
    0.25f, 0.15f, 0.12f, 0.08f,
    0.25f, 0.08f, 0.08f, 0.08f,
    0.25f, 0.08f, 0.08f, 0.08f,
    0.25f, 0.08f, 0.08f, 0.08f,
    0.25f, 0.08f, 0.08f, 0.08f,
};

static int verify_mit_v2_parameters(struct WujiDevice *hand) {
    float limits[JOINT_COUNT], kp[JOINT_COUNT], kd[JOINT_COUNT];
    const uint32_t all_online = (UINT32_C(1) << JOINT_COUNT) - 1u;
    uint32_t limit_online = 0, gain_online = 0;
    if (wuji_hand_2_get_all_effort_limit(hand, limits, &limit_online) != WUJI_STATUS_OK
        || wuji_hand_2_get_all_mit_params(hand, kp, kd, &gain_online) != WUJI_STATUS_OK) {
        fprintf(stderr, "read MIT_V2 parameters: %s\n", wuji_last_error());
        return 0;
    }
    if (limit_online != all_online || gain_online != all_online) {
        fprintf(stderr, "MIT_V2 requires %d readable parameter entries\n", JOINT_COUNT);
        return 0;
    }
    for (int i = 0; i < JOINT_COUNT; ++i) {
        if (!isfinite(limits[i]) || !isfinite(kp[i]) || !isfinite(kd[i])
            || fabsf(limits[i] - TORQUE_LIMITS_NM[i]) > 1e-5f
            || fabsf(kp[i] - KP) > 1e-5f || fabsf(kd[i] - KD) > 1e-5f) {
            fprintf(stderr, "MIT_V2 parameter mismatch at joint %d\n", i);
            return 0;
        }
    }
    return 1;
}

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

#define CHECK(call) do { \
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
        if (strspn(sn, " \t\r\n") == strlen(sn)) {
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
    char serial_number[64];
    if (wuji_dev_serial_number(device, serial_number, sizeof(serial_number)) < 0) {
        report_sdk_error("wuji_dev_serial_number");
        result = 1;
        goto cleanup;
    }
    if (sn != NULL && strcmp(serial_number, sn) != 0) {
        fprintf(stderr, "connected serial number differs from --sn\n");
        result = 1;
        goto cleanup;
    }
    hand_checked = 1;
    uint8_t online = 0;
    CHECK(wuji_hand_2_online_joints_count(device, &online));
    if (online != JOINT_COUNT) {
        fprintf(stderr, "MIT_V2 requires 20/20 online joints\n");
        result = 1;
        goto cleanup;
    }
    printf("Connected: %s\n", serial_number);
    // Disable joints before switching control mode.
    CHECK(wuji_hand_2_disable(device, NULL));
    if (stop_requested) goto cleanup;
    CHECK(wuji_hand_2_set_control_mode(device, WUJI_CONTROL_MODE_MIT_V2));
    float kp[JOINT_COUNT], kd[JOINT_COUNT];
    for (int i = 0; i < JOINT_COUNT; ++i) { kp[i] = KP; kd[i] = KD; }
    CHECK(wuji_hand_2_set_all_mit_params(device, kp, kd));
    CHECK(wuji_hand_2_set_all_effort_limit_per_joint(device, TORQUE_LIMITS_NM));
    if (stop_requested) goto cleanup;
    if (!verify_mit_v2_parameters(device)) { result = 1; goto cleanup; }
    if (stop_requested) goto cleanup;
    CHECK(wuji_hand_2_enable(device, NULL));
    result = wait_enabled(device);
    if (result || stop_requested) goto cleanup;
    float start[JOINT_COUNT];
    result = read_positions(device, start);
    if (result || stop_requested) goto cleanup;
    CHECK(wuji_hand_2_joint_command_publish(device, &publisher));
    const int home_steps = (int)(HOME_SECONDS * PUB_HZ);
    const int hold_steps = (int)(HOLD_SECONDS * PUB_HZ);
    double started = now_seconds();
    for (int step = 1; step <= home_steps + hold_steps && !stop_requested; ++step) {
        float scale = step < home_steps ? 1.0f - (float)step / home_steps : 0.0f;
        WujiJointCommand commands[JOINT_COUNT] = {{0}};
        for (int i = 0; i < JOINT_COUNT; ++i) commands[i].position = start[i] * scale;
        CHECK(wuji_joint_command_publisher_send(publisher, commands));
        double wait = started + (double)step / PUB_HZ - now_seconds();
        if (wait > 0) sleep_seconds(wait);
    }

cleanup:
    if (publisher != NULL) wuji_joint_command_publisher_close(publisher);
    if (hand_checked && wuji_hand_2_disable(device, NULL) != WUJI_STATUS_OK) {
        report_sdk_error("wuji_hand_2_disable");
        result = 1;
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
