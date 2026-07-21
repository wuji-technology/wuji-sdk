/*
 * Wuji SDK C — Retargeting: RetargetSession (no hardware required).
 *
 * Map human hand keypoints (21 MediaPipe-format landmarks, in meters) to a
 * Wuji Hand joint command (20 values). This is the low-level building block:
 * you supply the keypoints from any source (camera, glove replay, a VR
 * headset, ...), and get back a joint command ready to send to the hand.
 * The C API mirrors the Python `RetargetSession` one-to-one.
 *
 * Flow:
 *   wuji_retarget_session_create(model, side, &session)   -- once
 *   wuji_retarget_session_step(session, kp[63], qpos[20]) -- every frame
 *   wuji_retarget_session_reset(session)                  -- after a tracking gap
 *   wuji_retarget_session_free(session)                   -- when done
 *
 * The session keeps warm-start and smoothing state across frames, so create
 * it once and reuse it in your loop.
 *
 * Build: point CMake at your extracted SDK tarball (see ../README.md):
 *          cmake -S . -B build \
 *            -DWUJI_SDK_INCLUDE_DIR=/path/to/sdk/include \
 *            -DWUJI_SDK_LIB=/path/to/sdk/lib/libwuji_sdk_c.so
 *          cmake --build build
 * Run:   ./build/0_retarget_session
 */
#include <stdio.h>
#include <stdlib.h>

#include "wuji_sdk.h"

#define NUM_KEYPOINTS 21 /* MediaPipe landmarks: wrist + 5 fingers x 4 */
#define QPOS_LEN 20      /* joint command length, firmware order */

/* A synthetic open-right-hand pose, (21 x 3) in meters, MediaPipe landmark
 * order (row-major xyz). Replace this with your real keypoint source
 * (camera / glove / replay). */
static void make_open_hand_keypoints(float kp[NUM_KEYPOINTS * 3]) {
    /* finger base landmark index -> x offset (thumb, index, middle, ring, pinky) */
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
    /* thumb CMC (landmark 1) nearer the palm */
    kp[1 * 3 + 0] = -0.03f;
    kp[1 * 3 + 1] = 0.02f;
    kp[1 * 3 + 2] = 0.01f;
}

int main(void) {
    /* Build a session for a right Wuji Hand 2. The hand model selects the
     * builtin tuning config internally, so you don't deal with config paths.
     * (Models: WUJI_HAND_MODEL_WUJI_HAND or WUJI_HAND_MODEL_WUJI_HAND2.) */
    struct WujiRetargetSession *session = NULL;
    WujiStatus st = wuji_retarget_session_create(
        WUJI_HAND_MODEL_WUJI_HAND2, WUJI_HANDEDNESS_RIGHT, &session);
    if (st != WUJI_STATUS_OK) {
        fprintf(stderr, "create failed (%d): %s\n", (int)st, wuji_last_error());
        return EXIT_FAILURE;
    }

    float keypoints[NUM_KEYPOINTS * 3];
    make_open_hand_keypoints(keypoints);

    float qpos[QPOS_LEN]; /* radians, firmware joint order */
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

    /* step(...) every frame; reset() after a tracking gap. */
    wuji_retarget_session_reset(session);
    wuji_retarget_session_free(session);
    return EXIT_SUCCESS;
}
