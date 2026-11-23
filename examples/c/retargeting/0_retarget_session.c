/*
 * Purpose:
 *   Map 21 synthetic hand keypoints to 20 joint positions.
 * Prerequisites:
 *   Install the C SDK. No device is required.
 * Usage:
 *   Build with CMake as described in ../README.md, selecting 0_retarget_session.
 *   ./build/0_retarget_session
 * Expected Behavior:
 *   Print the right-hand joint command in radians, grouped by finger.
 *   Exit 0 on completion or Ctrl-C, 1 on execution failure, and 2 on invalid arguments.
 */
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <errno.h>
#include <string.h>

#include "wuji_sdk.h"

#define NUM_KEYPOINTS 21
#define QPOS_LEN 20

static void make_open_hand_keypoints(float kp[NUM_KEYPOINTS * 3]) {
    static const struct {
        int base;
        float x;
    } fingers[] = {
        {1, -0.04f}, {5, -0.03f}, {9, -0.01f}, {13, 0.01f}, {17, 0.03f},
    };
    for (int i = 0; i < NUM_KEYPOINTS * 3; i++) kp[i] = 0.0f;
    for (size_t f = 0; f < sizeof(fingers) / sizeof(fingers[0]); f++) {
        for (int k = 0; k < 4; k++) {
            int j = fingers[f].base + k;
            kp[j * 3 + 0] = fingers[f].x;
            kp[j * 3 + 1] = 0.03f * (float)(k + 1);
        }
    }
    kp[1 * 3 + 0] = -0.03f;
    kp[1 * 3 + 1] = 0.02f;
    kp[1 * 3 + 2] = 0.01f;
}

static volatile sig_atomic_t stop_requested = 0;
static void on_sigint(int signum) { (void)signum; stop_requested = 1; }

int main(int argc, char **argv) {
    if (argc != 1) { fprintf(stderr, "Usage: %s\n", argv[0]); return 2; }
    if (signal(SIGINT, on_sigint) == SIG_ERR) {
        fprintf(stderr, "install Ctrl-C handler failed: %s\n", strerror(errno)); return 1;
    }
    int result = 0;
    struct WujiRetargetSession *session = NULL;
    WujiStatus st = wuji_retarget_session_create(
        WUJI_HAND_MODEL_WUJI_HAND2, WUJI_HANDEDNESS_RIGHT, &session);
    if (st != WUJI_STATUS_OK) {
        fprintf(stderr, "create failed (%d): %s\n", (int)st, wuji_last_error());
        return EXIT_FAILURE;
    }

    if (stop_requested) goto cleanup;
    float keypoints[NUM_KEYPOINTS * 3];
    make_open_hand_keypoints(keypoints);

    float qpos[QPOS_LEN];
    st = wuji_retarget_session_step(session, keypoints, qpos);
    if (st != WUJI_STATUS_OK) {
        fprintf(stderr, "step failed (%d): %s\n", (int)st, wuji_last_error());
        wuji_retarget_session_free(session);
        return EXIT_FAILURE;
    }

    printf("joint command (radians, finger-major thumb/index/middle/ring/pinky):\n");
    static const char *finger_names[] = {"thumb", "index", "middle", "ring", "pinky"};
    for (int f = 0; f < 5; f++) {
        printf("  %-6s: [%+.3f, %+.3f, %+.3f, %+.3f]\n", finger_names[f],
               (double)qpos[f * 4 + 0], (double)qpos[f * 4 + 1],
               (double)qpos[f * 4 + 2], (double)qpos[f * 4 + 3]);
    }

    if (!stop_requested && wuji_retarget_session_reset(session) != WUJI_STATUS_OK) {
        fprintf(stderr, "reset failed: %s\n", wuji_last_error()); result = 1;
    }
cleanup:
    wuji_retarget_session_free(session);
    return result;
}
