/*
 * Purpose:
 *   Subscribe to glove data streams with callbacks.
 * Prerequisites:
 *   Install the C SDK and connect a reachable Wuji Glove.
 * Usage:
 *   Build with CMake as described in ../README.md, selecting 1_subscribe_callback.
 *   ./build/glove/1_subscribe_callback
 *   ./build/glove/1_subscribe_callback --sn DEVICE_SN
 * Expected Behavior:
 *   Print tactile, tactile-zone, EMF pose, joint angle, skeleton, and
 *   point-cloud summaries, palm raw/fused IMU, fingertip poses,
 *   and global coordinate transforms until Ctrl-C.
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

typedef struct {
    const char *device_name;
    const char *stream;
} cb_ctx_t;

static void on_tactile(WujiFrameKind kind, const WujiTactileFrame *f, void *ud) {
    cb_ctx_t *ctx = (cb_ctx_t *)ud;
    if (!frame_ok(kind, f)) return;
    float max_val = 0.0f;
    for (size_t i = 0; i < f->data_len; i++) {
        if (f->data[i] > max_val) max_val = f->data[i];
    }
    printf("[%s][Tactile] seq=%u taxels=%zu max=%.3f\n",
           ctx->device_name, f->header.seq, f->data_len, max_val);
}

static void on_tactile_zones(WujiFrameKind kind, const WujiTactileZones *f, void *ud) {
    cb_ctx_t *ctx = (cb_ctx_t *)ud;
    if (!frame_ok(kind, f)) return;
    float palm = (f->palm_len > 0) ? f->palm[0] : 0.0f;
    printf("[%s][Zones] seq=%u palm=%.3f\n",
           ctx->device_name, f->header.seq, palm);
}

static void on_emf_poses(WujiFrameKind kind, const WujiEmfPoseArray *f, void *ud) {
    cb_ctx_t *ctx = (cb_ctx_t *)ud;
    if (!frame_ok(kind, f)) return;
    if (f->poses_len == 0) {
        printf("[%s][EmfPoses] seq=%u (no poses)\n",
               ctx->device_name, f->header.seq);
        return;
    }
    const float *p = f->poses[0].pose.position;
    printf("[%s][EmfPoses] seq=%u pos=[%+.3f,%+.3f,%+.3f]\n",
           ctx->device_name, f->header.seq, p[0], p[1], p[2]);
}

static void on_joint_angles(WujiFrameKind kind, const WujiHandJointAngles *f, void *ud) {
    cb_ctx_t *ctx = (cb_ctx_t *)ud;
    if (!frame_ok(kind, f)) return;
    if (f->fingers_len < 5) {
        printf("[%s][JointAngles] seq=%u (insufficient fingers: %zu)\n",
               ctx->device_name, f->header.seq, f->fingers_len);
        return;
    }
    printf("[%s][JointAngles] seq=%u conf=[%.2f,%.2f,%.2f,%.2f,%.2f]\n",
           ctx->device_name, f->header.seq,
           f->fingers[0].confidence, f->fingers[1].confidence,
           f->fingers[2].confidence, f->fingers[3].confidence,
           f->fingers[4].confidence);
}

static void on_skeleton(WujiFrameKind kind, const WujiHandSkeleton *f, void *ud) {
    cb_ctx_t *ctx = (cb_ctx_t *)ud;
    if (!frame_ok(kind, f)) return;
    if (f->joints_len == 0) return;
    const float *p = f->joints[0].pose.position;
    printf("[%s][Skeleton] seq=%u wrist=[%+.3f,%+.3f,%+.3f] joints=%zu\n",
           ctx->device_name, f->header.seq, p[0], p[1], p[2], f->joints_len);
}

static void on_point_cloud(WujiFrameKind kind, const WujiPointCloud *f, void *ud) {
    cb_ctx_t *ctx = (cb_ctx_t *)ud;
    if (!frame_ok(kind, f)) return;
    size_t pts = (f->point_stride > 0) ? f->data_len / f->point_stride : 0;
    printf("[%s][PointCloud] seq=%u points=%zu frame=%s\n",
           ctx->device_name, f->header.seq, pts, f->frame_id);
}


static void on_imu(WujiFrameKind kind, const WujiImuData *f, void *ud) {
    cb_ctx_t *ctx = ud;
    if (!frame_ok(kind, f)) return;
    printf("[%s][%s] acceleration=[%.3f, %.3f, %.3f] angular_velocity=[%.3f, %.3f, %.3f] "
           "orientation=[%.4f, %.4f, %.4f, %.4f]\n",
           ctx->device_name, ctx->stream,
           f->linear_acceleration.x, f->linear_acceleration.y, f->linear_acceleration.z,
           f->angular_velocity.x, f->angular_velocity.y, f->angular_velocity.z,
           f->orientation.x, f->orientation.y, f->orientation.z, f->orientation.w);
}

static void on_tip_poses(WujiFrameKind kind, const WujiFingertipPoses *f, void *ud) {
    cb_ctx_t *ctx = ud;
    if (!frame_ok(kind, f)) return;
    for (size_t i = 0; i < f->poses_len; ++i) {
        const WujiFingertipPose *tip = &f->poses[i];
        printf("[%s][TipPoses] finger=%zu position=[%.3f, %.3f, %.3f]\n",
               ctx->device_name, i, tip->pose.position[0],
               tip->pose.position[1], tip->pose.position[2]);
    }
}

static void on_transforms(WujiFrameKind kind, const WujiFrameTransforms *f, void *ud) {
    cb_ctx_t *ctx = (cb_ctx_t *)ud;
    if (!frame_ok(kind, f)) return;
    if (f->transforms_len == 0) {
        printf("[%s][%s] (no transforms)\n",
               ctx->device_name, ctx->stream);
        return;
    }
    const WujiFrameTransform *t = &f->transforms[0];
    printf("[%s][%s] count=%zu first=%s->%s xyz=[%+.3f,%+.3f,%+.3f]\n",
           ctx->device_name, ctx->stream, f->transforms_len,
           t->parent_frame_id, t->child_frame_id,
           t->translation[0], t->translation[1], t->translation[2]);
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
    WujiSub *subs[11] = {0};
    cb_ctx_t contexts[11] = {0};
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

    contexts[0] = (cb_ctx_t){"example", "tactile"};
    CHECK(wuji_glove_subscribe_tactile(device, on_tactile, &contexts[0], &subs[0]));
    contexts[1] = (cb_ctx_t){"example", "tactile_zones"};
    CHECK(wuji_glove_subscribe_tactile_zones(device, on_tactile_zones, &contexts[1], &subs[1]));
    contexts[2] = (cb_ctx_t){"example", "emf_poses"};
    CHECK(wuji_glove_subscribe_emf_poses(device, on_emf_poses, &contexts[2], &subs[2]));
    contexts[3] = (cb_ctx_t){"example", "hand_joint_angles"};
    CHECK(wuji_glove_subscribe_hand_joint_angles(device, on_joint_angles, &contexts[3], &subs[3]));
    contexts[4] = (cb_ctx_t){"example", "hand_skeleton"};
    CHECK(wuji_glove_subscribe_hand_skeleton(device, on_skeleton, &contexts[4], &subs[4]));
    contexts[5] = (cb_ctx_t){"example", "tactile_point_cloud"};
    CHECK(wuji_glove_subscribe_tactile_point_cloud(device, on_point_cloud, &contexts[5], &subs[5]));
    contexts[6] = (cb_ctx_t){"example", "tf"};
    CHECK(wuji_subscribe_tf(on_transforms, &contexts[6], &subs[6]));
    contexts[7] = (cb_ctx_t){"example", "tf_static"};
    CHECK(wuji_subscribe_tf_static(on_transforms, &contexts[7], &subs[7]));
    contexts[8] = (cb_ctx_t){"example", "imu_palm"};
    CHECK(wuji_glove_subscribe_imu_palm(device, on_imu, &contexts[8], &subs[8]));
    contexts[9] = (cb_ctx_t){"example", "imu_data_palm"};
    CHECK(wuji_glove_subscribe_imu_data_palm(device, on_imu, &contexts[9], &subs[9]));
    contexts[10] = (cb_ctx_t){"example", "tip_poses"};
    CHECK(wuji_glove_subscribe_tip_poses(device, on_tip_poses, &contexts[10], &subs[10]));
    while (!stop_requested && !atomic_load(&stream_failed)) sleep_ms(100);
cleanup:
    atomic_store(&closing, 1);
    for (int i = 10; i >= 0; --i) if (subs[i] != NULL) wuji_sub_close(subs[i]);
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
