/*
 * Purpose:
 *   Replay the recorded thumb-opposition trajectory in position mode.
 * Prerequisites:
 *   Install the C SDK and use firmware 3.0.0 or later. All 20 joints must be
 *   online. Keep the hand clear of people and objects. Keep the data directory beside the executable.
 * Usage:
 *   Build with CMake as described in ../../README.md, selecting 6_opposition.
 *   ./build/6_opposition/6_opposition
 *   ./build/6_opposition/6_opposition --sn DEVICE_SN
 * Expected Behavior:
 *   The SDK may report a thumb abduction stall warning during this motion.
 *   Select the replay by device handedness. Disable, select position mode,
 *   set the command cutoff to 40 Hz, and enable. Ramp to the first recorded
 *   pose in 1 second, then publish every recorded pose once at 1 kHz.
 *   Disable and disconnect on completion, error, or Ctrl-C. Exit 0 on
 *   completion or Ctrl-C, 1 on execution or cleanup failure, and 2 on
 *   invalid arguments. Errors go to stderr.
 */

#define _POSIX_C_SOURCE 200809L

#include "wuji_sdk.h"

#include <errno.h>
#include <math.h>
#include <limits.h>
#include <stdlib.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#ifdef __APPLE__
#include <mach-o/dyld.h>
#endif

#define JOINT_COUNT WUJI_HAND_2_JOINT_COUNT
#define PUB_HZ 1000
#define COMMAND_CUTOFF_HZ 40.0f
#define REPLAY_HEADER_SIZE 16
#define REPLAY_FRAME_SIZE 84

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

static uint32_t read_u32_le(const uint8_t *bytes) {
    return (uint32_t)bytes[0] | ((uint32_t)bytes[1] << 8) |
           ((uint32_t)bytes[2] << 16) | ((uint32_t)bytes[3] << 24);
}

static int replay_path(WujiHandedness side, char path[PATH_MAX]) {
    char executable[PATH_MAX];
#ifdef __APPLE__
    char unresolved[PATH_MAX];
    uint32_t size = sizeof(unresolved);
    if (_NSGetExecutablePath(unresolved, &size) != 0 ||
        realpath(unresolved, executable) == NULL) return 1;
#else
    ssize_t length = readlink("/proc/self/exe", executable, sizeof(executable) - 1);
    if (length <= 0 || (size_t)length >= sizeof(executable) - 1) return 1;
    executable[length] = '\0';
#endif
    char *separator = strrchr(executable, '/');
    if (separator == NULL) return 1;
    *separator = '\0';
    int length_out = snprintf(path, PATH_MAX, "%s/data/%s.replay", executable,
                              side == WUJI_HANDEDNESS_LEFT ? "left" : "right");
    return length_out < 0 || length_out >= PATH_MAX;
}

static int read_header(FILE *stream, WujiHandedness side, uint32_t *count) {
    uint8_t header[REPLAY_HEADER_SIZE];
    if (fread(header, 1, sizeof(header), stream) != sizeof(header) ||
        memcmp(header, "WJH2RPL\0", 8) != 0 || header[8] != 1 || header[9] != 0 ||
        header[10] != (side == WUJI_HANDEDNESS_LEFT ? 2 : 1) ||
        header[11] != JOINT_COUNT) {
        fprintf(stderr, "Error: invalid replay header\n");
        return 1;
    }
    *count = read_u32_le(header + 12);
    if (*count == 0 || fseek(stream, 0, SEEK_END) != 0) {
        fprintf(stderr, "Error: empty or unreadable replay\n");
        return 1;
    }
    long size = ftell(stream);
    uint64_t expected = REPLAY_HEADER_SIZE + (uint64_t)*count * REPLAY_FRAME_SIZE;
    if (size < 0 || (uint64_t)size != expected || fseek(stream, REPLAY_HEADER_SIZE, SEEK_SET) != 0) {
        fprintf(stderr, "Error: replay file length does not match frame count\n");
        return 1;
    }
    return 0;
}

static int read_pose(FILE *stream, float positions[JOINT_COUNT]) {
    uint8_t frame[REPLAY_FRAME_SIZE];
    if (fread(frame, 1, sizeof(frame), stream) != sizeof(frame)) {
        fprintf(stderr, "Error: short replay frame\n");
        return 1;
    }
    for (int i = 0; i < JOINT_COUNT; ++i) {
        uint32_t bits = read_u32_le(frame + 4 + i * 4);
        memcpy(&positions[i], &bits, sizeof(bits));
        if (!isfinite(positions[i])) {
            fprintf(stderr, "Error: invalid position in replay\n");
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
    FILE *stream = NULL;
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
    WujiHandedness side;
    CHECK(wuji_hand_2_get_handedness(device, &side));
    if (side != WUJI_HANDEDNESS_LEFT && side != WUJI_HANDEDNESS_RIGHT) {
        fprintf(stderr, "Error: unsupported handedness\n"); result = 1; goto cleanup;
    }
    char path[PATH_MAX];
    if (replay_path(side, path)) {
        fprintf(stderr, "Error: cannot resolve replay path\n"); result = 1; goto cleanup;
    }
    stream = fopen(path, "rb");
    if (stream == NULL) {
        fprintf(stderr, "open replay failed: %s\n", strerror(errno)); result = 1; goto cleanup;
    }
    uint32_t count;
    result = read_header(stream, side, &count);
    if (result || stop_requested) goto cleanup;
    float first[JOINT_COUNT];
    result = read_pose(stream, first);
    if (result || stop_requested) goto cleanup;
    if (fseek(stream, REPLAY_HEADER_SIZE, SEEK_SET) != 0) {
        fprintf(stderr, "seek replay failed: %s\n", strerror(errno)); result = 1; goto cleanup;
    }
    // Disable joints before switching control mode.
    CHECK(wuji_hand_2_disable(device, NULL));
    if (stop_requested) goto cleanup;
    CHECK(wuji_hand_2_set_control_mode(device, WUJI_CONTROL_MODE_POSITION));
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
    for (int step = 1; step <= PUB_HZ && !stop_requested; ++step) {
        float ratio = (float)(0.5 * (1.0 - cos(3.141592653589793 * step / PUB_HZ)));
        WujiJointCommand commands[JOINT_COUNT] = {{0}};
        for (int i = 0; i < JOINT_COUNT; ++i)
            commands[i].position = start[i] + (first[i] - start[i]) * ratio;
        CHECK(wuji_joint_command_publisher_send(publisher, commands));
        double wait = started + (double)step / PUB_HZ - now_seconds();
        if (wait > 0) sleep_seconds(wait);
    }
    started = now_seconds();
    for (uint32_t step = 0; step < count && !stop_requested; ++step) {
        float positions[JOINT_COUNT];
        result = read_pose(stream, positions);
        if (result || stop_requested) goto cleanup;
        WujiJointCommand commands[JOINT_COUNT] = {{0}};
        for (int i = 0; i < JOINT_COUNT; ++i) commands[i].position = positions[i];
        CHECK(wuji_joint_command_publisher_send(publisher, commands));
        double wait = started + (double)(step + 1) / PUB_HZ - now_seconds();
        if (step + 1 < count && wait > 0) sleep_seconds(wait);
    }

cleanup:
    if (publisher != NULL) wuji_joint_command_publisher_close(publisher);
    if (hand_checked && wuji_hand_2_disable(device, NULL) != WUJI_STATUS_OK) {
        report_sdk_error("wuji_hand_2_disable");
        result = 1;
    }
    if (stream != NULL && fclose(stream) != 0) {
        fprintf(stderr, "close replay failed: %s\n", strerror(errno));
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
