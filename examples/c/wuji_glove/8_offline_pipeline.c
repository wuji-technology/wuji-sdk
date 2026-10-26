/*
 * Wuji SDK C - offline (recorded-input) solving for Wuji Glove.
 *
 * This right-hand example processes one synthetic EMF frame and one raw IMU
 * sample without connecting to a device. It prints the SN and hand side read
 * back from the pipeline, joint angle array slots, fingertip poses, the hand
 * skeleton, and fused IMU data.
 * For a recording, use its device SN and hand side, and submit its frames and
 * samples to the same instance in recording order.
 *
 * Unless urdf_path is supplied, model selection uses the current SDK user.
 * The example prints the selected model source and path.
 *
 * Run: ./build/8_offline_pipeline [urdf_path]
 */
#include <stdio.h>
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

static int fail(const char *what, WujiStatus status) {
    const char *message = wuji_last_error();
    fprintf(stderr, "%s failed: status %d, %s\n", what, (int)status,
            message != NULL ? message : "No error details available.");
    return 1;
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

int main(int argc, char **argv) {
    const char *urdf_path = argc > 1 ? argv[1] : NULL;
    /* Use the recording device's SN to select its parameters. */
    const char *sn = "WUJI-GLOVE-EXAMPLE";

    WujiInitOptions init = {0};
    init.log_level = 2;
    WujiStatus status = wuji_init(&init);
    if (status != WUJI_STATUS_OK) return fail("wuji_init", status);

    struct WujiGloveOfflinePipeline *pipeline = NULL;
    status = wuji_glove_offline_create(sn, WUJI_HANDEDNESS_RIGHT, urdf_path, &pipeline);
    if (status != WUJI_STATUS_OK) {
        fail("wuji_glove_offline_create", status);
        wuji_shutdown();
        return 1;
    }

    /* Read back what the pipeline was created with: the SN through a two-call size
     * query, the hand side as an enum that can be passed back to create. */
    size_t sn_needed = 0;
    status = wuji_glove_offline_serial_number(pipeline, NULL, 0, &sn_needed);
    if (status != WUJI_STATUS_ERR_BUFFER_TOO_SMALL) {
        fail("wuji_glove_offline_serial_number", status);
        wuji_glove_offline_free(pipeline);
        wuji_shutdown();
        return 1;
    }
    char *pipeline_sn = malloc(sn_needed);
    if (pipeline_sn == NULL) {
        fprintf(stderr, "Could not allocate the SN buffer.\n");
        wuji_glove_offline_free(pipeline);
        wuji_shutdown();
        return 1;
    }
    status = wuji_glove_offline_serial_number(pipeline, pipeline_sn, sn_needed, NULL);
    if (status != WUJI_STATUS_OK) {
        fail("wuji_glove_offline_serial_number", status);
        free(pipeline_sn);
        wuji_glove_offline_free(pipeline);
        wuji_shutdown();
        return 1;
    }
    WujiHandedness hand_side = WUJI_HANDEDNESS_LEFT;
    status = wuji_glove_offline_handedness(pipeline, &hand_side);
    if (status != WUJI_STATUS_OK) {
        fail("wuji_glove_offline_handedness", status);
        free(pipeline_sn);
        wuji_glove_offline_free(pipeline);
        wuji_shutdown();
        return 1;
    }
    printf("pipeline: sn=%s, hand_side=%s\n",
           pipeline_sn,
           hand_side == WUJI_HANDEDNESS_LEFT ? "left" : "right");
    free(pipeline_sn);

    WujiOfflineUrdfSource source = WUJI_OFFLINE_URDF_SOURCE_BUILTIN_DEFAULT;
    status = wuji_glove_offline_urdf_source(pipeline, &source);
    if (status != WUJI_STATUS_OK) {
        fail("wuji_glove_offline_urdf_source", status);
        wuji_glove_offline_free(pipeline);
        wuji_shutdown();
        return 1;
    }
    size_t needed = 0;
    status = wuji_glove_offline_urdf_source_path(pipeline, NULL, 0, &needed);
    /* A built-in model has no file path and returns OK even for a size query. */
    if (status != WUJI_STATUS_OK && status != WUJI_STATUS_ERR_BUFFER_TOO_SMALL) {
        fail("wuji_glove_offline_urdf_source_path", status);
        wuji_glove_offline_free(pipeline);
        wuji_shutdown();
        return 1;
    }
    char *path = malloc(needed);
    if (path == NULL) {
        fprintf(stderr, "Could not allocate the URDF path buffer.\n");
        wuji_glove_offline_free(pipeline);
        wuji_shutdown();
        return 1;
    }
    status = wuji_glove_offline_urdf_source_path(pipeline, path, needed, NULL);
    if (status != WUJI_STATUS_OK) {
        fail("wuji_glove_offline_urdf_source_path", status);
        free(path);
        wuji_glove_offline_free(pipeline);
        wuji_shutdown();
        return 1;
    }
    printf("urdf source: %s\n", source_name(source));
    printf("urdf path  : %s\n", path[0] != '\0' ? path : "(built-in, no file)");
    free(path);

    WujiEmfPose poses[POSE_COUNT];
    WujiEmfPoseArray frame;
    fill_emf_frame(&frame, poses);

    WujiHandJointAngles angles = {0};
    status = wuji_glove_offline_compute_hand_joint_angles(pipeline, &frame, &angles);
    if (status != WUJI_STATUS_OK) {
        fail("wuji_glove_offline_compute_hand_joint_angles", status);
        wuji_glove_offline_free(pipeline);
        wuji_shutdown();
        return 1;
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

    WujiFingertipPoses tips = {0};
    status = wuji_glove_offline_compute_tip_poses(pipeline, &frame, &tips);
    if (status != WUJI_STATUS_OK) {
        fail("wuji_glove_offline_compute_tip_poses", status);
        wuji_hand_joint_angles_free(&angles);
        wuji_glove_offline_free(pipeline);
        wuji_shutdown();
        return 1;
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

    WujiHandSkeleton skeleton = {0};
    status = wuji_glove_offline_compute_hand_skeleton(pipeline, &frame, &skeleton);
    if (status != WUJI_STATUS_OK) {
        fail("wuji_glove_offline_compute_hand_skeleton", status);
        wuji_fingertip_poses_free(&tips);
        wuji_hand_joint_angles_free(&angles);
        wuji_glove_offline_free(pipeline);
        wuji_shutdown();
        return 1;
    }
    printf("skeleton: %zu joints (expected %d), frame_id %s\n",
           skeleton.joints_len,
           JOINT_COUNT,
           skeleton.header.frame_id);

    WujiImuData raw;
    fill_raw_imu(&raw);
    WujiImuData fused = {0};
    status = wuji_glove_offline_compute_imu_data_palm(pipeline, &raw, &fused);
    if (status != WUJI_STATUS_OK) {
        fail("wuji_glove_offline_compute_imu_data_palm", status);
        wuji_hand_skeleton_free(&skeleton);
        wuji_fingertip_poses_free(&tips);
        wuji_hand_joint_angles_free(&angles);
        wuji_glove_offline_free(pipeline);
        wuji_shutdown();
        return 1;
    }
    printf("fused imu: q=[%.4f %.4f %.4f %.4f], orientation_covariance[0]=%.1f\n",
           fused.orientation.x,
           fused.orientation.y,
           fused.orientation.z,
           fused.orientation.w,
           fused.orientation_covariance[0]);

    /* Release output heap fields with their matching free functions, then free the pipeline. */
    wuji_hand_skeleton_free(&skeleton);
    wuji_fingertip_poses_free(&tips);
    wuji_hand_joint_angles_free(&angles);
    wuji_glove_offline_free(pipeline);
    wuji_shutdown();
    return 0;
}
