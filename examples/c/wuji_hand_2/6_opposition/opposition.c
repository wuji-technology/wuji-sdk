#define _POSIX_C_SOURCE 200809L

/* Wuji SDK C - stream a recorded Hand 2 opposition trajectory at 1 kHz. */

#include <errno.h>
#include <limits.h>
#include <math.h>
#include <signal.h>
#include <stdatomic.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "wuji_sdk.h"

#define JOINT_COUNT 20u
#define REPLAY_HEADER_SIZE 16u
#define REPLAY_FRAME_SIZE 84u
#define REPLAY_INTERVAL_NS 1000000L
#define REPLAY_VERSION 1u
#define RAMP_DURATION_NS 1000000000L
#define RAMP_STEPS (RAMP_DURATION_NS / REPLAY_INTERVAL_NS)
#define CAPTURE_ATTEMPTS 400u
#define CAPTURE_WAIT_NS 5000000L
#define PI_VALUE 3.14159265358979323846

typedef enum {
    REPLAY_SIDE_RIGHT = 1,
    REPLAY_SIDE_LEFT = 2,
} replay_side_t;

typedef enum {
    REPLAY_ERROR_NONE = 0,
    REPLAY_ERROR_OPEN,
    REPLAY_ERROR_HEADER,
    REPLAY_ERROR_VERSION,
    REPLAY_ERROR_HANDEDNESS,
    REPLAY_ERROR_JOINT_COUNT,
    REPLAY_ERROR_FILE_LENGTH,
    REPLAY_ERROR_READ,
} replay_error_t;

typedef struct {
    FILE *stream;
    uint32_t frame_count;
    replay_error_t error;
} replay_reader_t;

typedef enum {
    PLAY_RESULT_OK = 0,
    PLAY_RESULT_INTERRUPTED = -2,
    PLAY_RESULT_READ_ERROR = -3,
    PLAY_RESULT_SEND_ERROR = -4,
    PLAY_RESULT_SLEEP_ERROR = -5,
} play_result_t;

static const uint8_t REPLAY_MAGIC[8] = {
    'W', 'J', 'H', '2', 'R', 'P', 'L', '\0',
};
static volatile sig_atomic_t g_stop = 0;

/* Request cooperative cleanup after Ctrl+C. */
static void on_sigint(int signal_number) {
    (void)signal_number;
    g_stop = 1;
}

/* Return a stable diagnostic when the SDK has no error text. */
static const char *sdk_error_detail(void) {
    const char *detail = wuji_last_error();

    return detail != NULL && detail[0] != '\0'
        ? detail : "SDK did not provide error details";
}

/* Report an SDK failure while its thread-local reason is still current. */
static void report_sdk_failure(const char *stage) {
    fprintf(stderr, "error: %s failed: %s\n", stage, sdk_error_detail());
}

/* Decode one little-endian uint16. */
static uint16_t read_u16_le(const uint8_t bytes[2]) {
    return (uint16_t)bytes[0] | (uint16_t)((uint16_t)bytes[1] << 8u);
}

/* Decode one little-endian uint32. */
static uint32_t read_u32_le(const uint8_t bytes[4]) {
    return (uint32_t)bytes[0]
        | ((uint32_t)bytes[1] << 8u)
        | ((uint32_t)bytes[2] << 16u)
        | ((uint32_t)bytes[3] << 24u);
}

/* Decode one little-endian float32 without alignment assumptions. */
static float read_f32_le(const uint8_t bytes[4]) {
    const uint32_t bits = read_u32_le(bytes);
    float value;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

/* Close an optional replay stream. */
static void close_replay(replay_reader_t *reader) {
    if (reader != NULL && reader->stream != NULL) {
        fclose(reader->stream);
        reader->stream = NULL;
    }
}

/* Describe a local replay validation failure without using SDK state. */
static const char *replay_error_detail(replay_error_t error) {
    switch (error) {
    case REPLAY_ERROR_OPEN:
        return "file could not be opened";
    case REPLAY_ERROR_HEADER:
        return "header or magic is invalid";
    case REPLAY_ERROR_VERSION:
        return "unsupported version";
    case REPLAY_ERROR_HANDEDNESS:
        return "handedness does not match the selected device";
    case REPLAY_ERROR_JOINT_COUNT:
        return "joint count is not 20";
    case REPLAY_ERROR_FILE_LENGTH:
        return "file length does not match frame count";
    case REPLAY_ERROR_READ:
        return "frame is short";
    case REPLAY_ERROR_NONE:
    default:
        return "unknown replay error";
    }
}

/* Open and validate one replay container by explicit path. */
static int open_replay_path(
    const char *path,
    replay_side_t expected_side,
    replay_reader_t *reader) {
    uint8_t header[REPLAY_HEADER_SIZE];
    uint32_t frame_count;
    uint64_t expected_size;
    long file_size;

    if (reader == NULL) return -1;
    memset(reader, 0, sizeof(*reader));
    if (path == NULL) {
        reader->error = REPLAY_ERROR_OPEN;
        return -1;
    }
    reader->stream = fopen(path, "rb");
    if (reader->stream == NULL) {
        reader->error = REPLAY_ERROR_OPEN;
        return -1;
    }
    if (fread(header, 1u, sizeof(header), reader->stream) != sizeof(header)) {
        reader->error = REPLAY_ERROR_HEADER;
        close_replay(reader);
        return -1;
    }
    if (memcmp(header, REPLAY_MAGIC, sizeof(REPLAY_MAGIC)) != 0) {
        reader->error = REPLAY_ERROR_HEADER;
        close_replay(reader);
        return -1;
    }
    if (read_u16_le(&header[8]) != REPLAY_VERSION) {
        reader->error = REPLAY_ERROR_VERSION;
        close_replay(reader);
        return -1;
    }
    if (header[10] != (uint8_t)expected_side) {
        reader->error = REPLAY_ERROR_HANDEDNESS;
        close_replay(reader);
        return -1;
    }
    if (header[11] != JOINT_COUNT) {
        reader->error = REPLAY_ERROR_JOINT_COUNT;
        close_replay(reader);
        return -1;
    }
    frame_count = read_u32_le(&header[12]);
    expected_size = REPLAY_HEADER_SIZE
        + (uint64_t)frame_count * REPLAY_FRAME_SIZE;
    if (fseek(reader->stream, 0L, SEEK_END) != 0) {
        reader->error = REPLAY_ERROR_FILE_LENGTH;
        close_replay(reader);
        return -1;
    }
    file_size = ftell(reader->stream);
    if (file_size < 0 || (uint64_t)file_size != expected_size
        || fseek(reader->stream, (long)REPLAY_HEADER_SIZE, SEEK_SET) != 0) {
        reader->error = REPLAY_ERROR_FILE_LENGTH;
        close_replay(reader);
        return -1;
    }
    reader->frame_count = frame_count;
    return 0;
}

/* Read one replay frame into a timestamp and flat-20 qpos buffer. */
static int read_replay_frame(
    replay_reader_t *reader,
    uint32_t *t_us,
    float qpos[JOINT_COUNT]) {
    uint8_t frame[REPLAY_FRAME_SIZE];
    size_t index;

    if (reader == NULL || reader->stream == NULL
        || t_us == NULL || qpos == NULL) {
        if (reader != NULL) reader->error = REPLAY_ERROR_READ;
        return -1;
    }
    if (fread(frame, 1u, sizeof(frame), reader->stream) != sizeof(frame)) {
        reader->error = REPLAY_ERROR_READ;
        return -1;
    }
    *t_us = read_u32_le(frame);
    for (index = 0u; index < JOINT_COUNT; ++index) {
        qpos[index] = read_f32_le(&frame[4u + index * 4u]);
    }
    return 0;
}

/* Resolve one replay beside the running executable. */
static int replay_path_for_side(
    replay_side_t side,
    char path[PATH_MAX]) {
    char executable[PATH_MAX];
    const char *filename = side == REPLAY_SIDE_RIGHT
        ? "right.replay" : "left.replay";
    ssize_t length;
    char *separator;
    int written;

    length = readlink("/proc/self/exe", executable, sizeof(executable) - 1u);
    if (length <= 0 || (size_t)length >= sizeof(executable)) return -1;
    executable[length] = '\0';
    separator = strrchr(executable, '/');
    if (separator == NULL) return -1;
    *separator = '\0';
    written = snprintf(path, PATH_MAX, "%s/data/%s", executable, filename);
    return written > 0 && written < PATH_MAX ? 0 : -1;
}

/* Sleep for one fixed replay interval. */
static int sleep_interval(void) {
    struct timespec duration = {0, REPLAY_INTERVAL_NS};
    while (nanosleep(&duration, &duration) != 0) {
        if (errno != EINTR || g_stop) return -1;
    }
    return 0;
}

/* Scan and connect the only discovered device. */
static int auto_connect_hand(struct WujiDevice **out_device) {
    struct WujiDiscovered *devices = NULL;
    struct WujiConnectTarget target = {0};
    size_t count = 0u;
    WujiStatus status;

    status = wuji_scan(&devices, &count);
    if (status != WUJI_STATUS_OK) {
        report_sdk_failure("scan");
        wuji_discovered_free(devices, count);
        return -1;
    }
    if (count == 0u) {
        fprintf(stderr,
                "error: no Hand 2 device found; check that the device is powered on and reachable on the network\n");
        wuji_discovered_free(devices, count);
        return -1;
    }
    if (count != 1u) {
        fprintf(stderr,
                "error: found %zu devices; this example requires exactly 1 device\n",
                count);
        wuji_discovered_free(devices, count);
        return -1;
    }
    target.kind = WUJI_CONNECT_TARGET_KIND_SN;
    target.value = devices[0].serial_number;
    status = wuji_connect(&target, "hand_2_opposition", NULL, out_device);
    if (status != WUJI_STATUS_OK) {
        report_sdk_failure("connect");
        wuji_discovered_free(devices, count);
        return -1;
    }
    wuji_discovered_free(devices, count);
    return 0;
}

/* Publish one flat-20 MIT position command. */
static int send_qpos(
    struct WujiJointCommandPublisher *publisher,
    const float qpos[JOINT_COUNT]) {
    struct WujiJointCommand commands[JOINT_COUNT];
    size_t index;

    for (index = 0u; index < JOINT_COUNT; ++index) {
        commands[index].position = qpos[index];
        commands[index].velocity = 0.0f;
        commands[index].effort = 0.0f;
    }
    return wuji_joint_command_publisher_send(publisher, commands)
        == WUJI_STATUS_OK ? 0 : -1;
}

/* Capture state: the worker thread fills flat-20 positions and sets ready. */
typedef struct {
    _Atomic int ready;
    float positions[JOINT_COUNT];
} capture_ctx_t;

/* Scatter one frame into flat-20 command order via the SDK nid mapping.
 * Reject frames that do not carry exactly the 20 joint nids — a tactile-slot
 * or out-of-range nid fails wuji_hand_2_nid_to_joint_index, and a duplicate
 * or missing joint leaves the seen-mask incomplete. */
static int frame_to_positions(const WujiJointStateFrame *frame,
                              float positions[JOINT_COUNT]) {
    uint32_t seen = 0u;
    size_t index;

    if (frame->joints_len != JOINT_COUNT) return -1;
    for (index = 0u; index < JOINT_COUNT; ++index) {
        uint8_t joint_index = 0u;
        if (wuji_hand_2_nid_to_joint_index(frame->joints[index].nid,
                                           &joint_index)
                != WUJI_STATUS_OK) {
            return -1;
        }
        if (seen & (1u << joint_index)) return -1;
        seen |= 1u << joint_index;
        positions[joint_index] = frame->joints[index].position;
    }
    return seen == ((1u << JOINT_COUNT) - 1u) ? 0 : -1;
}

/* Record one complete joint_states frame into the capture context. */
static void on_capture_frame(WujiFrameKind kind,
                             const WujiJointStateFrame *frame,
                             void *user_data) {
    capture_ctx_t *ctx = (capture_ctx_t *)user_data;

    if (kind != WUJI_FRAME_KIND_OK || frame == NULL) return;
    if (frame_to_positions(frame, ctx->positions) == 0) {
        atomic_store_explicit(&ctx->ready, 1, memory_order_release);
    }
}

/* Wait for one complete joint_states frame; -1 timeout, -2 stop, -3 subscribe. */
static int capture_start_positions(struct WujiDevice *device,
                                   float start[JOINT_COUNT]) {
    capture_ctx_t ctx;
    struct WujiSub *subscription = NULL;
    struct timespec wait = {0, CAPTURE_WAIT_NS};
    size_t attempt;
    int captured;

    memset(&ctx, 0, sizeof(ctx));
    if (wuji_hand_2_subscribe_joint_states(device, on_capture_frame, &ctx,
                                           &subscription)
        != WUJI_STATUS_OK) {
        return -3;
    }
    for (attempt = 0u; attempt < CAPTURE_ATTEMPTS && !g_stop; ++attempt) {
        if (atomic_load_explicit(&ctx.ready, memory_order_acquire)) break;
        nanosleep(&wait, NULL);
    }
    wuji_sub_close(subscription);
    if (g_stop) return -2;
    captured = atomic_load_explicit(&ctx.ready, memory_order_acquire);
    if (captured) memcpy(start, ctx.positions, sizeof(ctx.positions));
    return captured ? 0 : -1;
}

/* Ease from the captured pose to the opening frame with a cosine profile. */
static int send_ramp(struct WujiJointCommandPublisher *publisher,
                     const float start[JOINT_COUNT],
                     const float target[JOINT_COUNT]) {
    float qpos[JOINT_COUNT];
    size_t index;
    size_t step;

    for (step = 1u; step <= RAMP_STEPS; ++step) {
        const double ratio =
            0.5 * (1.0 - cos(PI_VALUE * (double)step / (double)RAMP_STEPS));
        if (g_stop) return -2;
        for (index = 0u; index < JOINT_COUNT; ++index) {
            qpos[index] =
                start[index] + (target[index] - start[index]) * (float)ratio;
        }
        if (send_qpos(publisher, qpos) != 0) return -1;
        if (step < RAMP_STEPS && sleep_interval() != 0) {
            return g_stop ? -2 : -3;
        }
    }
    return 0;
}

/* Stream every replay frame once at 1 kHz. */
static play_result_t play_replay(
    struct WujiJointCommandPublisher *publisher,
    replay_reader_t *reader) {
    float qpos[JOINT_COUNT];
    uint32_t t_us;
    uint32_t frame_index;

    if (g_stop) return PLAY_RESULT_INTERRUPTED;
    for (frame_index = 0u; frame_index < reader->frame_count; ++frame_index) {
        if (g_stop) return PLAY_RESULT_INTERRUPTED;
        if (read_replay_frame(reader, &t_us, qpos) != 0) {
            return PLAY_RESULT_READ_ERROR;
        }
        (void)t_us;
        if (send_qpos(publisher, qpos) != 0) {
            return PLAY_RESULT_SEND_ERROR;
        }
        if (frame_index + 1u < reader->frame_count
            && sleep_interval() != 0) {
            return g_stop ? PLAY_RESULT_INTERRUPTED : PLAY_RESULT_SLEEP_ERROR;
        }
    }
    return g_stop ? PLAY_RESULT_INTERRUPTED : PLAY_RESULT_OK;
}

/* Connect, select replay data, enable, stream, and always clean up. */
static int run_device(void) {
    struct WujiInitOptions init_options = {.log_level = 0};
    struct WujiDevice *device = NULL;
    struct WujiJointCommandPublisher *publisher = NULL;
    replay_reader_t reader = {0};
    enum WujiHandedness handedness;
    replay_side_t side;
    char replay_path[PATH_MAX];
    uint8_t online = 0u;
    int initialized = 0;
    int enabled = 0;
    int cleanup_failed = 0;
    int exit_code = 1;
    int play_result;
    float start[JOINT_COUNT];
    float first_qpos[JOINT_COUNT];
    uint32_t t_us;

    if (wuji_init(&init_options) != WUJI_STATUS_OK) {
        report_sdk_failure("initialization");
        goto cleanup;
    }
    initialized = 1;
    if (auto_connect_hand(&device) != 0) goto cleanup;
    if (wuji_hand_2_online_joints_count(device, &online) != WUJI_STATUS_OK) {
        report_sdk_failure("online-joint check");
        goto cleanup;
    }
    if (online != JOINT_COUNT) {
        fprintf(stderr,
                "error: online-joint check failed: Hand 2 requires 20/20 online joints (found %u)\n",
                (unsigned int)online);
        goto cleanup;
    }
    if (wuji_hand_2_get_handedness(device, &handedness)
            != WUJI_STATUS_OK) {
        report_sdk_failure("get-handedness");
        goto cleanup;
    }
    if (handedness == WUJI_HANDEDNESS_RIGHT) {
        side = REPLAY_SIDE_RIGHT;
    } else if (handedness == WUJI_HANDEDNESS_LEFT) {
        side = REPLAY_SIDE_LEFT;
    } else {
        fprintf(stderr, "error: handedness is unsupported; expected Left or Right\n");
        goto cleanup;
    }
    if (replay_path_for_side(side, replay_path) != 0) {
        fprintf(stderr, "error: replay path resolution failed for selected handedness\n");
        goto cleanup;
    }
    if (open_replay_path(replay_path, side, &reader) != 0) {
        if (reader.error == REPLAY_ERROR_OPEN) {
            fprintf(stderr, "error: replay open failed: %s\n", replay_path);
        } else {
            fprintf(stderr, "error: replay validation failed: %s (%s)\n",
                    replay_error_detail(reader.error), replay_path);
        }
        goto cleanup;
    }
    if (wuji_hand_2_enable(device, NULL) != WUJI_STATUS_OK) {
        report_sdk_failure("enable");
        goto cleanup;
    }
    enabled = 1;
    if (wuji_hand_2_joint_command_publish(device, &publisher)
            != WUJI_STATUS_OK) {
        report_sdk_failure("publisher creation");
        goto cleanup;
    }
    play_result = capture_start_positions(device, start);
    if (play_result == -2) {
        exit_code = 130;
        goto cleanup;
    }
    if (play_result == -3) {
        report_sdk_failure("subscribe joint_states");
        goto cleanup;
    }
    if (play_result != 0) {
        fprintf(stderr, "error: capture joint_states failed: no complete frame\n");
        goto cleanup;
    }
    if (reader.frame_count > 0u) {
        /* Peek the opening frame as the ramp target, then rewind. */
        if (read_replay_frame(&reader, &t_us, first_qpos) != 0) {
            fprintf(stderr, "error: replay read failed: %s\n",
                    replay_error_detail(reader.error));
            goto cleanup;
        }
        if (fseek(reader.stream, (long)REPLAY_HEADER_SIZE, SEEK_SET) != 0) {
            fprintf(stderr, "error: replay read failed: unable to rewind\n");
            goto cleanup;
        }
        play_result = send_ramp(publisher, start, first_qpos);
        if (play_result == -1) {
            report_sdk_failure("ramp send");
            goto cleanup;
        }
        if (play_result == -2) {
            exit_code = 130;
            goto cleanup;
        }
        if (play_result == -3) {
            fprintf(stderr, "error: ramp sleep failed\n");
            goto cleanup;
        }
    }
    play_result = play_replay(publisher, &reader);
    if (play_result == PLAY_RESULT_INTERRUPTED) {
        exit_code = 130;
    } else if (play_result == PLAY_RESULT_OK) {
        exit_code = 0;
    } else if (play_result == PLAY_RESULT_READ_ERROR) {
        fprintf(stderr, "error: replay read failed: %s\n",
                replay_error_detail(reader.error));
    } else if (play_result == PLAY_RESULT_SEND_ERROR) {
        report_sdk_failure("replay send");
    } else if (play_result == PLAY_RESULT_SLEEP_ERROR) {
        fprintf(stderr, "error: replay sleep failed\n");
    } else {
        fprintf(stderr, "error: replay interrupt failed\n");
    }

cleanup:
    if (device != NULL) {
        if (wuji_hand_2_disable(device, NULL) != WUJI_STATUS_OK) {
            report_sdk_failure("disable");
            if (enabled) cleanup_failed = 1;
        }
    }
    if (publisher != NULL) wuji_joint_command_publisher_close(publisher);
    close_replay(&reader);
    if (device != NULL) {
        if (wuji_dev_disconnect(device) != WUJI_STATUS_OK) {
            report_sdk_failure("disconnect");
            cleanup_failed = 1;
        }
        wuji_dev_release(device);
    }
    if (initialized) wuji_shutdown();
    if (exit_code == 0 && cleanup_failed) exit_code = 1;
    return exit_code;
}

/* Run the physical replay. */
int main(void) {
    g_stop = 0;
    signal(SIGINT, on_sigint);
    return run_device();
}
