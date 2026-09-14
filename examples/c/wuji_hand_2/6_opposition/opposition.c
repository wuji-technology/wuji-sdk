#define _POSIX_C_SOURCE 200809L

/* Wuji SDK C - stream a recorded Hand 2 opposition trajectory at 1 kHz. */

#include <errno.h>
#include <limits.h>
#include <signal.h>
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

typedef enum {
    REPLAY_SIDE_RIGHT = 1,
    REPLAY_SIDE_LEFT = 2,
} replay_side_t;

typedef struct {
    FILE *stream;
    uint32_t frame_count;
} replay_reader_t;

static const uint8_t REPLAY_MAGIC[8] = {
    'W', 'J', 'H', '2', 'R', 'P', 'L', '\0',
};
static volatile sig_atomic_t g_stop = 0;

/* Request cooperative cleanup after Ctrl+C. */
static void on_sigint(int signal_number) {
    (void)signal_number;
    g_stop = 1;
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

/* Open and validate one replay container by explicit path. */
static int open_replay_path(
    const char *path,
    replay_side_t expected_side,
    replay_reader_t *reader) {
    uint8_t header[REPLAY_HEADER_SIZE];
    uint32_t frame_count;
    uint64_t expected_size;
    long file_size;

    if (path == NULL || reader == NULL) return -1;
    memset(reader, 0, sizeof(*reader));
    reader->stream = fopen(path, "rb");
    if (reader->stream == NULL) return -1;
    if (fread(header, 1u, sizeof(header), reader->stream) != sizeof(header)) {
        close_replay(reader);
        return -1;
    }
    if (memcmp(header, REPLAY_MAGIC, sizeof(REPLAY_MAGIC)) != 0
        || read_u16_le(&header[8]) != REPLAY_VERSION
        || header[10] != (uint8_t)expected_side
        || header[11] != JOINT_COUNT) {
        close_replay(reader);
        return -1;
    }
    frame_count = read_u32_le(&header[12]);
    expected_size = REPLAY_HEADER_SIZE
        + (uint64_t)frame_count * REPLAY_FRAME_SIZE;
    if (fseek(reader->stream, 0L, SEEK_END) != 0) {
        close_replay(reader);
        return -1;
    }
    file_size = ftell(reader->stream);
    if (file_size < 0 || (uint64_t)file_size != expected_size
        || fseek(reader->stream, (long)REPLAY_HEADER_SIZE, SEEK_SET) != 0) {
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
        return -1;
    }
    if (fread(frame, 1u, sizeof(frame), reader->stream) != sizeof(frame)) {
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

    if (wuji_scan(&devices, &count) != WUJI_STATUS_OK || count != 1u) {
        wuji_discovered_free(devices, count);
        return -1;
    }
    target.kind = WUJI_CONNECT_TARGET_KIND_SN;
    target.value = devices[0].serial_number;
    status = wuji_connect(&target, "hand_2_opposition", NULL, out_device);
    wuji_discovered_free(devices, count);
    return status == WUJI_STATUS_OK ? 0 : -1;
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

/* Stream every replay frame once at 1 kHz. */
static int play_replay(
    struct WujiJointCommandPublisher *publisher,
    replay_reader_t *reader) {
    float qpos[JOINT_COUNT];
    uint32_t t_us;
    uint32_t frame_index;

    for (frame_index = 0u; frame_index < reader->frame_count; ++frame_index) {
        if (g_stop) return -2;
        if (read_replay_frame(reader, &t_us, qpos) != 0) return -1;
        (void)t_us;
        if (send_qpos(publisher, qpos) != 0) return -1;
        if (frame_index + 1u < reader->frame_count
            && sleep_interval() != 0) {
            return g_stop ? -2 : -1;
        }
    }
    return 0;
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
    int cleanup_failed = 0;
    int exit_code = 1;
    int play_result;

    if (wuji_init(&init_options) != WUJI_STATUS_OK) goto cleanup;
    initialized = 1;
    if (auto_connect_hand(&device) != 0) goto cleanup;
    if (wuji_hand_2_online_joints_count(device, &online)
            != WUJI_STATUS_OK || online != JOINT_COUNT) {
        fprintf(stderr, "error: Hand 2 requires 20/20 online joints\n");
        goto cleanup;
    }
    if (wuji_hand_2_get_handedness(device, &handedness)
            != WUJI_STATUS_OK) {
        goto cleanup;
    }
    if (handedness == WUJI_HANDEDNESS_RIGHT) {
        side = REPLAY_SIDE_RIGHT;
    } else if (handedness == WUJI_HANDEDNESS_LEFT) {
        side = REPLAY_SIDE_LEFT;
    } else {
        goto cleanup;
    }
    if (replay_path_for_side(side, replay_path) != 0
        || open_replay_path(replay_path, side, &reader) != 0) {
        fprintf(stderr, "error: replay file is invalid or missing\n");
        goto cleanup;
    }
    if (wuji_hand_2_enable(device, NULL) != WUJI_STATUS_OK) goto cleanup;
    if (wuji_hand_2_joint_command_publish(device, &publisher)
            != WUJI_STATUS_OK) {
        goto cleanup;
    }
    play_result = play_replay(publisher, &reader);
    if (play_result == -2) {
        exit_code = 130;
    } else if (play_result == 0) {
        exit_code = 0;
    }

cleanup:
    if (device != NULL
        && wuji_hand_2_disable(device, NULL) != WUJI_STATUS_OK) {
        cleanup_failed = 1;
    }
    wuji_joint_command_publisher_close(publisher);
    close_replay(&reader);
    if (device != NULL) {
        if (wuji_dev_disconnect(device) != WUJI_STATUS_OK) cleanup_failed = 1;
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
