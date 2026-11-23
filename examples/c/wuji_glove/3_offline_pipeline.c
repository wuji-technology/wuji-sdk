/*
 * Purpose:
 *   Run hand tracking and IMU fusion offline.
 * Prerequisites:
 *   Install the C SDK. No device is required.
 * Usage:
 *   Build with CMake as described in ../README.md, selecting 3_offline_pipeline.
 *   ./build/glove/3_offline_pipeline
 *   ./build/glove/3_offline_pipeline --urdf-path /absolute/path/to/hand.urdf
 * Expected Behavior:
 *   Print finger joint angles and confidence, fingertip positions, and the
 *   skeleton joint count and frame ID. Print fused IMU orientation and its
 *   first covariance entry.
 *   Exit 0 on completion or Ctrl-C, 1 on execution or cleanup failure,
 *   and 2 on invalid arguments. Errors go to stderr.
 */
#include <stdio.h>
#include <errno.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>

#include "wuji_sdk.h"

#define POSE_COUNT 5
#define JOINT_COUNT 21

static const char *source_name(WujiOfflineUrdfSource source) {
    switch (source) {
        case WUJI_OFFLINE_URDF_SOURCE_OVERRIDE: return "override";
        case WUJI_OFFLINE_URDF_SOURCE_CALIBRATION_FILE: return "calibration_file";
        case WUJI_OFFLINE_URDF_SOURCE_BUILTIN_DEFAULT: return "builtin_default";
        default: return "unknown";
    }
}

static void report_sdk_error(const char *operation) {
    const char *error = wuji_last_error();
    fprintf(stderr, "%s failed: %s\n", operation, error ? error : "Unknown SDK error");
}

/* Five fingertip poses spread along the wrist X axis, identity orientation. */
static void fill_emf_frame(WujiEmfPoseArray *frame, WujiEmfPose poses[POSE_COUNT]) {
    static const float offsets[POSE_COUNT] = {0.00f, 0.03f, 0.07f, 0.11f, 0.15f};
    for (int i = 0; i < POSE_COUNT; i++) {
        poses[i].pose.position[0] = offsets[i];
        poses[i].pose.position[1] = 0.0f;
        poses[i].pose.position[2] = 0.0f;
        poses[i].pose.orientation.x = 0.0f;
        poses[i].pose.orientation.y = 0.0f;
        poses[i].pose.orientation.z = 0.0f;
        poses[i].pose.orientation.w = 1.0f;
        poses[i].confidence = 1.0f;
    }

    frame->header.seq = 1;
    frame->header.timestamp_us = 1000000;
    snprintf(frame->header.frame_id, sizeof(frame->header.frame_id), "%s", "r_hand_emf_tx");
    frame->poses = poses;
    frame->poses_len = POSE_COUNT;
}

/* A raw IMU sample: orientation_covariance[0] == -1 marks it as not yet fused. */
static void fill_raw_imu(WujiImuData *raw) {
    memset(raw, 0, sizeof(*raw));
    raw->header.seq = 1;
    raw->header.timestamp_us = 1000000;
    snprintf(raw->header.frame_id, sizeof(raw->header.frame_id), "%s", "imu_raw/palm");
    raw->orientation.w = 1.0;
    raw->orientation_covariance[0] = -1.0;
    raw->angular_velocity.x = 0.1;
    raw->angular_velocity.y = 0.2;
    raw->angular_velocity.z = 0.3;
    raw->linear_acceleration.z = 9.8;
}

static volatile sig_atomic_t stop_requested = 0;
static void on_sigint(int signum) { (void)signum; stop_requested = 1; }

#define CHECK(call) do { \
    if (stop_requested) goto cleanup; \
    status = (call); \
    if (status != WUJI_STATUS_OK) { report_sdk_error(#call); result = 1; goto cleanup; } \
} while (0)

int main(int argc, char **argv) {
    const char *urdf_path = NULL;
    if (argc == 3 && strcmp(argv[1], "--urdf-path") == 0 && argv[2][0] != '\0') urdf_path = argv[2];
    else if (argc != 1) { fprintf(stderr, "Usage: %s [--urdf-path FILE]\n", argv[0]); return 2; }
    if (signal(SIGINT, on_sigint) == SIG_ERR) {
        fprintf(stderr, "install Ctrl-C handler failed: %s\n", strerror(errno)); return 1;
    }
    int result = 0;
    WujiStatus status = wuji_init(NULL);
    if (status != WUJI_STATUS_OK) {
        report_sdk_error("wuji_init");
        return 1;
    }
    WujiGloveOfflinePipeline *pipeline = NULL;
    char *pipeline_sn = NULL;
    char *path = NULL;
    WujiHandJointAngles angles = {0};
    WujiFingertipPoses tips = {0};
    WujiHandSkeleton skeleton = {0};
    CHECK(wuji_glove_offline_create("WUJI-GLOVE-EXAMPLE", WUJI_HANDEDNESS_RIGHT, urdf_path, &pipeline));
    size_t sn_needed = 0;
    if (stop_requested) goto cleanup;
    status = wuji_glove_offline_serial_number(pipeline, NULL, 0, &sn_needed);
    if (status != WUJI_STATUS_ERR_BUFFER_TOO_SMALL) {
        report_sdk_error("wuji_glove_offline_serial_number");
        result = 1; goto cleanup;
    }
    pipeline_sn = malloc(sn_needed);
    if (pipeline_sn == NULL) {
        fprintf(stderr, "Error: out of memory\n");
        result = 1; goto cleanup;
    }
    if (stop_requested) goto cleanup;
    status = wuji_glove_offline_serial_number(pipeline, pipeline_sn, sn_needed, NULL);
    if (status != WUJI_STATUS_OK) {
        report_sdk_error("wuji_glove_offline_serial_number");
        result = 1; goto cleanup;
    }
    WujiHandedness hand_side = WUJI_HANDEDNESS_LEFT;
    if (stop_requested) goto cleanup;
    status = wuji_glove_offline_handedness(pipeline, &hand_side);
    if (status != WUJI_STATUS_OK) {
        report_sdk_error("wuji_glove_offline_handedness");
        result = 1; goto cleanup;
    }
    printf("pipeline: sn=%s, hand_side=%s\n",
           pipeline_sn,
           hand_side == WUJI_HANDEDNESS_LEFT ? "left" : "right");

    WujiOfflineUrdfSource source = WUJI_OFFLINE_URDF_SOURCE_BUILTIN_DEFAULT;
    if (stop_requested) goto cleanup;
    status = wuji_glove_offline_urdf_source(pipeline, &source);
    if (status != WUJI_STATUS_OK) {
        report_sdk_error("wuji_glove_offline_urdf_source");
        result = 1; goto cleanup;
    }
    size_t needed = 0;
    if (stop_requested) goto cleanup;
    status = wuji_glove_offline_urdf_source_path(pipeline, NULL, 0, &needed);
    if (status != WUJI_STATUS_OK && status != WUJI_STATUS_ERR_BUFFER_TOO_SMALL) {
        report_sdk_error("wuji_glove_offline_urdf_source_path");
        result = 1; goto cleanup;
    }
    path = malloc(needed);
    if (path == NULL) {
        fprintf(stderr, "Error: out of memory\n");
        result = 1; goto cleanup;
    }
    if (stop_requested) goto cleanup;
    status = wuji_glove_offline_urdf_source_path(pipeline, path, needed, NULL);
    if (status != WUJI_STATUS_OK) {
        report_sdk_error("wuji_glove_offline_urdf_source_path");
        result = 1; goto cleanup;
    }
    printf("urdf source: %s\n", source_name(source));
    printf("urdf path  : %s\n", path[0] != '\0' ? path : "(built-in, no file)");

    WujiEmfPose poses[POSE_COUNT];
    WujiEmfPoseArray frame;
    fill_emf_frame(&frame, poses);

    if (stop_requested) goto cleanup;
    status = wuji_glove_offline_compute_hand_joint_angles(pipeline, &frame, &angles);
    if (status != WUJI_STATUS_OK) {
        report_sdk_error("wuji_glove_offline_compute_hand_joint_angles");
        result = 1; goto cleanup;
    }
    printf("joint angles: %zu fingers (5 array slots per finger), seq %u\n",
           angles.fingers_len,
           angles.header.seq);
    for (size_t finger = 0; finger < angles.fingers_len; finger++) {
        printf("  finger %zu:", finger);
        for (int joint = 0; joint < 5; joint++) {
            printf(" %7.3f", angles.fingers[finger].angles[joint]);
        }
        printf("  confidence %.3f\n", angles.fingers[finger].confidence);
    }

    if (stop_requested) goto cleanup;
    status = wuji_glove_offline_compute_tip_poses(pipeline, &frame, &tips);
    if (status != WUJI_STATUS_OK) {
        report_sdk_error("wuji_glove_offline_compute_tip_poses");
        result = 1; goto cleanup;
    }
    printf("tip poses: %zu, frame_id %s, seq %u\n",
           tips.poses_len,
           tips.header.frame_id,
           tips.header.seq);
    for (size_t i = 0; i < tips.poses_len; i++) {
        printf("  tip %zu: [%7.3f %7.3f %7.3f]\n",
               i,
               tips.poses[i].pose.position[0],
               tips.poses[i].pose.position[1],
               tips.poses[i].pose.position[2]);
    }

    if (stop_requested) goto cleanup;
    status = wuji_glove_offline_compute_hand_skeleton(pipeline, &frame, &skeleton);
    if (status != WUJI_STATUS_OK) {
        report_sdk_error("wuji_glove_offline_compute_hand_skeleton");
        result = 1; goto cleanup;
    }
    printf("skeleton: %zu joints (expected %d), frame_id %s\n",
           skeleton.joints_len,
           JOINT_COUNT,
           skeleton.header.frame_id);

    WujiImuData raw;
    fill_raw_imu(&raw);
    WujiImuData fused = {0};
    if (stop_requested) goto cleanup;
    status = wuji_glove_offline_compute_imu_data_palm(pipeline, &raw, &fused);
    if (status != WUJI_STATUS_OK) {
        report_sdk_error("wuji_glove_offline_compute_imu_data_palm");
        result = 1; goto cleanup;
    }
    printf("fused imu: q=[%.4f %.4f %.4f %.4f], orientation_covariance[0]=%.1f\n",
           fused.orientation.x,
           fused.orientation.y,
           fused.orientation.z,
           fused.orientation.w,
           fused.orientation_covariance[0]);

cleanup:
    wuji_hand_skeleton_free(&skeleton);
    wuji_fingertip_poses_free(&tips);
    wuji_hand_joint_angles_free(&angles);
    free(path);
    free(pipeline_sn);
    if (pipeline != NULL) wuji_glove_offline_free(pipeline);
    wuji_shutdown();
    return result;
}
