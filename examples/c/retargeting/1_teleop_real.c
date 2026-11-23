/*
 * Purpose:
 *   Drive a Wuji Hand or Wuji Hand 2 from Wuji Glove keypoints.
 * Prerequisites:
 *   Install the C SDK. Use a hand and glove with matching handedness.
 *   Wuji Hand 2 needs firmware 3.0.0 or later.
 *   Keep the hand clear of people and objects.
 * Usage:
 *   Build with CMake as described in ../README.md, selecting 1_teleop_real.
 *   ./build/1_teleop_real
 *   ./build/1_teleop_real --hand-sn HAND_SN --glove-sn GLOVE_SN
 * Expected Behavior:
 *   Select an SDK user before connecting. Enter keeps the current user.
 *   Without serial numbers, select the last hand and glove found by scanning.
 *   Use a 1.5 A effort limit and 5 Hz LowPass controller for Wuji Hand, or
 *   position mode with a
 *   40 Hz command cutoff for Wuji Hand 2. Retarget at up to 120 Hz.
 *   Wuji Hand 2 first ramps from its current pose over 1 second.
 *   Stop with an error if no new glove skeleton frame arrives for 5 seconds.
 *   On exit, close streams, disable, disconnect, and restore the SDK user.
 *   Exit 0 on completion or Ctrl-C, 1 on execution or cleanup failure,
 *   and 2 on invalid arguments. Errors go to stderr.
 */
#define _POSIX_C_SOURCE 200809L

#include "wuji_sdk.h"
#include <errno.h>
#include <math.h>
#include <pthread.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#define JOINT_COUNT 20
#define FPS 120
#define HAND_CUTOFF_HZ 5.0
#define HAND_2_CUTOFF_HZ 40.0f

static volatile sig_atomic_t stop_requested = 0;

static void on_sigint(int signum) {
    (void)signum;
    stop_requested = 1;
}

static void report_sdk_error(const char *operation) {
    const char *error = wuji_last_error();
    fprintf(stderr, "%s failed: %s\n", operation, error ? error : "Unknown SDK error");
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

typedef struct {
    pthread_mutex_t mutex;
    float keypoints[63];
    int ready;
    double received_at;
    atomic_int failed;
    atomic_int closing;
} Skeleton;

static void on_skeleton(WujiFrameKind kind, const WujiHandSkeleton *frame, void *data) {
    Skeleton *skeleton = data;
    if (kind == WUJI_FRAME_KIND_END && atomic_load(&skeleton->closing)) return;
    if (kind == WUJI_FRAME_KIND_LAG) {
        const char *error = wuji_last_error();
        fprintf(stderr, "Warning: %s\n", error ? error : "subscription frames were dropped");
        return;
    }
    if (kind != WUJI_FRAME_KIND_OK || frame == NULL || frame->joints_len != 21) {
        fprintf(stderr, "hand_skeleton stream failed (%d): %s\n", kind,
                wuji_last_error() ? wuji_last_error() : "invalid or ended stream");
        atomic_store(&skeleton->failed, 1);
        return;
    }
    float keypoints[63];
    for (int i = 0; i < 21; ++i) {
        for (int j = 0; j < 3; ++j) {
            float value = frame->joints[i].pose.position[j];
            if (!isfinite(value)) {
                fprintf(stderr, "Error: invalid skeleton position\n");
                atomic_store(&skeleton->failed, 1);
                return;
            }
            keypoints[i * 3 + j] = value;
        }
    }
    pthread_mutex_lock(&skeleton->mutex);
    memcpy(skeleton->keypoints, keypoints, sizeof(keypoints));
    skeleton->ready = 1;
    skeleton->received_at = now_seconds();
    pthread_mutex_unlock(&skeleton->mutex);
}

#define USER_ID_CAP    256
#define USER_NAME_CAP  128

static WujiStatus select_sdk_user(char *out_prev_user_id, size_t cap, int *previous_loaded)
{
    WujiUserInfo previous;
    memset(&previous, 0, sizeof(previous));
    WujiStatus st = wuji_current_user(&previous);
    if (st != WUJI_STATUS_OK) {
        fprintf(stderr, "wuji_current_user failed: %s\n", wuji_last_error());
        return st;
    }
    char previous_name[USER_NAME_CAP] = {0};
    snprintf(out_prev_user_id, cap, "%s", previous.user_id ? previous.user_id : "");
    snprintf(previous_name, sizeof(previous_name), "%s",
             previous.display_name ? previous.display_name : "");
    wuji_user_info_free(&previous);
    *previous_loaded = 1;

    WujiUserInfo *users = NULL;
    size_t len = 0;
    st = wuji_list_users(&users, &len);
    if (st != WUJI_STATUS_OK) {
        fprintf(stderr, "wuji_list_users failed: %s\n", wuji_last_error());
        return st;
    }

    printf("SDK users:\n");
    for (size_t i = 0; i < len; i++) {
        const char *tags[2];
        size_t tag_count = 0;
        if (users[i].is_default) tags[tag_count++] = "built-in URDF";
        if (users[i].user_id && strcmp(users[i].user_id, out_prev_user_id) == 0)
            tags[tag_count++] = "current";

        const char *name = users[i].display_name ? users[i].display_name : "";
        if (tag_count == 2)
            printf("  [%zu] %s (%s, %s)\n", i, name, tags[0], tags[1]);
        else if (tag_count == 1)
            printf("  [%zu] %s (%s)\n", i, name, tags[0]);
        else
            printf("  [%zu] %s\n", i, name);
    }

    long index = -1;
    for (;;) {
        printf("Select user [0-%zu], Enter keeps current: ", len - 1);
        fflush(stdout);

        char line[128];
        if (!fgets(line, sizeof(line), stdin)) {
            if (stop_requested) {
                printf("\n");
                wuji_user_info_array_free(users, len);
                return WUJI_STATUS_ERR_CANCELLED;
            }
            printf("\nNo terminal input available - keeping the current SDK user.\n");
            break;
        }

        char *s = line;
        while (*s == ' ' || *s == '\t') s++;
        char *end = s + strlen(s);
        while (end > s && (end[-1] == '\n' || end[-1] == '\r' ||
                           end[-1] == ' '  || end[-1] == '\t'))
            *--end = '\0';
        if (*s == '\0') break;
        char *stop = NULL;
        long value = strtol(s, &stop, 10);
        if (*stop != '\0' || value < 0 || (size_t)value >= len) {
            printf("Invalid selection: '%s'\n", s);
            continue;
        }
        index = value;
        break;
    }

    if (index < 0) {
        wuji_user_info_array_free(users, len);
        printf("Running as SDK user: %s\n", previous_name);
    } else {
        char selected_user_id[USER_ID_CAP] = {0};
        snprintf(selected_user_id, sizeof(selected_user_id), "%s",
                 users[index].user_id ? users[index].user_id : "");
        wuji_user_info_array_free(users, len);

        WujiUserInfo selected;
        memset(&selected, 0, sizeof(selected));
        st = wuji_switch_user(selected_user_id, &selected);
        if (st != WUJI_STATUS_OK) {
            fprintf(stderr, "wuji_switch_user failed: %s\n", wuji_last_error());
            return st;
        }
        printf("Running as SDK user: %s\n",
               selected.display_name && selected.display_name[0]
                   ? selected.display_name : selected_user_id);
        wuji_user_info_free(&selected);
    }

    return WUJI_STATUS_OK;
}

static void select_devices(const WujiDiscoveredDevice *list, size_t n_dev,
                           const char *hand_sn, const char *glove_sn,
                           size_t *out_hand_idx, size_t *out_glove_idx,
                           WujiDeviceType *out_hand_type)
{
    size_t hand_idx  = n_dev;
    size_t glove_idx = n_dev;
    WujiDeviceType hand_type = WUJI_DEVICE_TYPE_UNKNOWN;

    for (size_t i = 0; i < n_dev; i++) {
        if (list[i].device_id == WUJI_DEVICE_TYPE_WUJI_HAND ||
            list[i].device_id == WUJI_DEVICE_TYPE_WUJI_HAND_2) {
            if (hand_sn == NULL || strcmp(list[i].serial_number, hand_sn) == 0) {
                hand_idx = i;
                hand_type = list[i].device_id;
            }
        } else if (list[i].device_id == WUJI_DEVICE_TYPE_WUJI_GLOVE) {
            if (glove_sn == NULL || strcmp(list[i].serial_number, glove_sn) == 0) {
                glove_idx = i;
            }
        }
    }

    *out_hand_idx  = hand_idx;
    *out_glove_idx = glove_idx;
    *out_hand_type = hand_type;
}

#define CHECK(call) do { \
    if (stop_requested) goto cleanup; \
    if ((call) != WUJI_STATUS_OK) { \
        report_sdk_error(#call); result = 1; goto cleanup; \
    } \
} while (0)

int main(int argc, char **argv) {
    const char *hand_sn = NULL, *glove_sn = NULL;
    for (int i = 1; i < argc; ++i) {
        const char **slot;
        const char *value;
        if (strcmp(argv[i], "--hand-sn") == 0 || strcmp(argv[i], "--glove-sn") == 0) {
            slot = strcmp(argv[i], "--hand-sn") == 0 ? &hand_sn : &glove_sn;
            if (++i >= argc || argv[i][0] == '-') {
                fprintf(stderr, "Error: missing serial number\n"); return 2;
            }
            value = argv[i];
        } else if (strncmp(argv[i], "--hand-sn=", 10) == 0) {
            slot = &hand_sn; value = argv[i] + 10;
        } else if (strncmp(argv[i], "--glove-sn=", 11) == 0) {
            slot = &glove_sn; value = argv[i] + 11;
        } else {
            fprintf(stderr, "Usage: %s [--hand-sn HAND_SN] [--glove-sn GLOVE_SN]\n", argv[0]); return 2;
        }
        *slot = value;
    }
    if ((hand_sn != NULL && hand_sn[0] == '\0') || (glove_sn != NULL && glove_sn[0] == '\0')) {
        fprintf(stderr, "Error: serial number must not be empty\n"); return 2;
    }
    /* Interrupt the user prompt instead of restarting fgets after Ctrl-C. */
    struct sigaction action = {0};
    action.sa_handler = on_sigint;
    sigemptyset(&action.sa_mask);
    if (sigaction(SIGINT, &action, NULL) != 0) {
        fprintf(stderr, "install Ctrl-C handler failed: %s\n", strerror(errno)); return 1;
    }
    if (wuji_init(NULL) != WUJI_STATUS_OK) { report_sdk_error("wuji_init"); return 1; }
    int result = 0, hand_checked = 0, is_hand2 = 0, previous_loaded = 0;
    char previous_user[USER_ID_CAP] = {0};
    WujiDiscoveredDevice *list = NULL;
    size_t count = 0;
    WujiDevice *hand = NULL, *glove = NULL;
    WujiJointCommandPublisher *publisher = NULL;
    WujiRealtimeController *controller = NULL;
    WujiRetargetSession *session = NULL;
    WujiSub *sub = NULL;
    Skeleton skeleton = {.mutex = PTHREAD_MUTEX_INITIALIZER};
    WujiStatus status = select_sdk_user(previous_user, sizeof(previous_user), &previous_loaded);
    if (status != WUJI_STATUS_OK) {
        result = status == WUJI_STATUS_ERR_CANCELLED && stop_requested ? 0 : 1;
        goto cleanup;
    }
    CHECK(wuji_scan_devices(NULL, &list, &count));
    size_t hand_index, glove_index;
    WujiDeviceType hand_type;
    select_devices(list, count, hand_sn, glove_sn, &hand_index, &glove_index, &hand_type);
    if (hand_index == count || glove_index == count) {
        fprintf(stderr, "Error: no matching hand and glove found\n"); result = 1; goto cleanup;
    }
    WujiConnectTarget hand_target = {.kind = WUJI_CONNECT_TARGET_KIND_SN, .value = list[hand_index].serial_number};
    WujiConnectTarget glove_target = {.kind = WUJI_CONNECT_TARGET_KIND_SN, .value = list[glove_index].serial_number};
    CHECK(wuji_connect(&hand_target, "hand", NULL, &hand));
    WujiDeviceType device_type;
    is_hand2 = hand_type == WUJI_DEVICE_TYPE_WUJI_HAND_2;
    if (is_hand2) {
        if (wuji_dev_device_type(hand, &device_type) != WUJI_STATUS_OK) { report_sdk_error("wuji_dev_device_type"); result = 1; goto cleanup; }
        if (device_type != WUJI_DEVICE_TYPE_WUJI_HAND_2) {
            fprintf(stderr, "Error: expected Wuji Hand 2, got device type %d\n", (int)device_type); result = 1; goto cleanup;
        }
    }
    hand_checked = 1;
    CHECK(wuji_connect(&glove_target, "glove", NULL, &glove));
    if (wuji_dev_device_type(glove, &device_type) != WUJI_STATUS_OK) { report_sdk_error("wuji_dev_device_type"); result = 1; goto cleanup; }
    if (device_type != WUJI_DEVICE_TYPE_WUJI_GLOVE) {
        fprintf(stderr, "Error: expected Wuji Glove, got device type %d\n", (int)device_type); result = 1; goto cleanup;
    }
    printf("Connected hand: %s, glove: %s\n", list[hand_index].serial_number, list[glove_index].serial_number);
    WujiHandedness side;
    if (is_hand2) {
        CHECK(wuji_hand_2_get_handedness(hand, &side));
    } else {
        uint8_t raw_side;
        CHECK(wuji_hand_get_handedness(hand, &raw_side));
        if (raw_side > 1) { fprintf(stderr, "Error: invalid hand side\n"); result = 1; goto cleanup; }
        side = raw_side == 0 ? WUJI_HANDEDNESS_RIGHT : WUJI_HANDEDNESS_LEFT;
    }
    char glove_side[16];
    CHECK(wuji_glove_get_hand_side(glove, glove_side, sizeof(glove_side), NULL));
    if ((side != WUJI_HANDEDNESS_LEFT && side != WUJI_HANDEDNESS_RIGHT) ||
        strcmp(glove_side, side == WUJI_HANDEDNESS_LEFT ? "left" : "right") != 0) {
        fprintf(stderr, "Error: hand and glove must have matching handedness\n"); result = 1; goto cleanup;
    }
    float start[JOINT_COUNT];
    if (is_hand2) {
        // Disable joints before switching control mode.
        CHECK(wuji_hand_2_disable(hand, NULL));
        if (stop_requested) goto cleanup;
        CHECK(wuji_hand_2_set_control_mode(hand, WUJI_CONTROL_MODE_POSITION));
        CHECK(wuji_hand_2_set_position_command_low_pass_cutoff_frequency(hand, HAND_2_CUTOFF_HZ));
        CHECK(wuji_hand_2_enable(hand, NULL));
        result = wait_enabled(hand);
        if (result || stop_requested) goto cleanup;
        result = read_positions(hand, start);
        if (result || stop_requested) goto cleanup;
        CHECK(wuji_hand_2_joint_command_publish(hand, &publisher));
    } else {
        CHECK(wuji_hand_disable(hand));
        CHECK(wuji_hand_set_all_effort_limit(hand, 1.5f));
        WujiLowPass filter = {.cutoff_hz = HAND_CUTOFF_HZ};
        CHECK(wuji_hand_realtime_controller_open(hand, filter, &controller));
        CHECK(wuji_hand_enable(hand));
    }
    CHECK(wuji_retarget_session_create(is_hand2 ? WUJI_HAND_MODEL_WUJI_HAND2 : WUJI_HAND_MODEL_WUJI_HAND, side, &session));
    CHECK(wuji_glove_subscribe_hand_skeleton(glove, on_skeleton, &skeleton, &sub));
    int have_frame = 0, ramp_step = 0;
    float keypoints[63];
    double deadline = now_seconds() + 5.0;
    while (!stop_requested && !atomic_load(&skeleton.failed)) {
        double started = now_seconds();
        pthread_mutex_lock(&skeleton.mutex);
        if (skeleton.ready) {
            memcpy(keypoints, skeleton.keypoints, sizeof(keypoints));
            skeleton.ready = 0; have_frame = 1;
            deadline = skeleton.received_at + 5.0;
        }
        pthread_mutex_unlock(&skeleton.mutex);
        if (now_seconds() >= deadline) {
            fprintf(stderr, "receive hand_skeleton failed: no new frame for 5 seconds\n");
            result = 1; goto cleanup;
        }
        if (!have_frame) {
            sleep_seconds(1.0 / FPS); continue;
        }
        float positions[JOINT_COUNT];
        CHECK(wuji_retarget_session_step(session, keypoints, positions));
        if (is_hand2) {
            WujiJointCommand commands[JOINT_COUNT] = {{0}};
            if (ramp_step < FPS) ++ramp_step;
            float alpha = (float)ramp_step / FPS;
            for (int i = 0; i < JOINT_COUNT; ++i)
                commands[i].position = ramp_step < FPS ? start[i] + alpha * (positions[i] - start[i]) : positions[i];
            CHECK(wuji_joint_command_publisher_send(publisher, commands));
        } else {
            CHECK(wuji_hand_realtime_controller_set_target_position(controller, positions));
        }
        double wait = started + 1.0 / FPS - now_seconds();
        if (wait > 0) sleep_seconds(wait);
    }

cleanup:
    atomic_store(&skeleton.closing, 1);
    if (sub != NULL) wuji_sub_close(sub);
    if (atomic_load(&skeleton.failed)) result = 1;
    pthread_mutex_destroy(&skeleton.mutex);
    if (session != NULL) wuji_retarget_session_free(session);
    if (publisher != NULL) wuji_joint_command_publisher_close(publisher);
    if (controller != NULL) wuji_hand_realtime_controller_close(controller);
    if (hand_checked) {
        status = is_hand2 ? wuji_hand_2_disable(hand, NULL) : wuji_hand_disable(hand);
        if (status != WUJI_STATUS_OK) { report_sdk_error("wuji_hand_2_disable / wuji_hand_disable"); result = 1; }
    }
    if (glove != NULL) {
        if (wuji_dev_disconnect(glove) != WUJI_STATUS_OK) { report_sdk_error("wuji_dev_disconnect"); result = 1; }
        wuji_dev_release(glove);
    }
    if (hand != NULL) {
        if (wuji_dev_disconnect(hand) != WUJI_STATUS_OK) { report_sdk_error("wuji_dev_disconnect"); result = 1; }
        wuji_dev_release(hand);
    }
    if (list != NULL) wuji_discovered_device_free(list, count);
    if (previous_loaded) {
        WujiUserInfo restored;
        status = previous_user[0] ? wuji_switch_user(previous_user, &restored) : wuji_switch_to_default_user(&restored);
        if (status != WUJI_STATUS_OK) { report_sdk_error("wuji_switch_user / wuji_switch_to_default_user"); result = 1; }
        else wuji_user_info_free(&restored);
    }
    wuji_shutdown();
    return result;
}
