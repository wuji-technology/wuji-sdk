/*
 * Wuji SDK C - Retargeting: live teleoperation from a Wuji Glove.
 *
 * The SDK exposes only the pure retarget interface (wuji_retarget_session_*).
 * Teleop - read keypoints -> retarget -> drive the hand - is plain example
 * code you can adapt, shown end to end below: read the glove's hand keypoints,
 * retarget each frame, and drive a connected Wuji Hand or Wuji Hand 2.
 *
 * To drive from a different keypoint source (camera / MediaPipe / VR),
 * replace the glove read with your own: wuji_retarget_session_step accepts
 * any 21x3 float array of MediaPipe-format landmarks (in meters) and returns
 * 20 joint values in firmware order, so the result is sent to the hand as-is.
 *
 * The session is built for the detected hand model (WUJI_HAND_MODEL_WUJI_HAND2
 * or WUJI_HAND_MODEL_WUJI_HAND) and the handedness defined by SIDE below.
 * For a left-hand rig (glove + hand) change SIDE to WUJI_HANDEDNESS_LEFT.
 *
 * Before connecting, the example lists the SDK users and lets you pick which
 * one to run under (see select_sdk_user); pressing Enter keeps the current
 * user, and the previously selected user is restored on exit. The default user
 * runs the glove on the SDK built-in default hand URDF; a named user runs it
 * with that user's calibrated hand model, or on the built-in URDF as well when
 * that user has no calibration yet. To teleoperate with a calibrated hand
 * model, create a user and calibrate the glove under it first (see
 * ../wuji_glove/3_user_management.c and ../wuji_glove/4_user_calibration.c).
 *
 * Build: point CMake at your extracted SDK tarball (see ../README.md):
 *          cmake -S . -B build \
 *            -DWUJI_SDK_INCLUDE_DIR=/path/to/sdk/include \
 *            -DWUJI_SDK_LIB=/path/to/sdk/lib/libwuji_sdk_c.so
 *          cmake --build build
 * Run:   ./build/1_teleop_real        (Ctrl+C to stop)
 *        ./build/1_teleop_real --hand-sn <hand SN> --glove-sn <glove SN>
 *
 * Without --hand-sn / --glove-sn the example drives whichever hand and glove
 * the scan reports. Pass the serial numbers when more than one is reachable.
 */

#define _POSIX_C_SOURCE 199309L

#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <pthread.h>

#include "wuji_sdk.h"

/* Handedness of the rig (glove + hand).
 * Change to WUJI_HANDEDNESS_LEFT for a left-hand setup. */
#define SIDE WUJI_HANDEDNESS_RIGHT

#define FPS           120     /* target loop rate; slow frames lower rate, never burst */
#define NUM_JOINTS    20      /* joint command array length, firmware order */
#define NUM_KEYPOINTS 21      /* MediaPipe hand landmarks: wrist + 5 fingers x 4 */
#define RAMP_SEC      1.0     /* startup ease from the hand's pose to the glove pose */

/* ============================================================================
 * SIGINT handling
 * ========================================================================== */

static volatile sig_atomic_t g_stop = 0;
static void on_sigint(int sig) { (void)sig; g_stop = 1; }

/* ============================================================================
 * Skeleton frame shared buffer (drain-latest-frame semantics)
 *
 * The subscription callback runs on an SDK worker thread. The main loop
 * consumes only the latest frame and discards older ones.
 *
 * A pthread_mutex protects the shared state: callback lock->copy->unlock;
 * main loop lock->check have_frame->take frame->clear have_frame->unlock.
 * 63 floats at 120 Hz has negligible contention; correctness over cleverness.
 * ========================================================================== */

typedef struct {
    pthread_mutex_t mu;
    float kp[NUM_KEYPOINTS * 3];   /* flattened (21 x 3) keypoints in meters */
    int   have_frame;              /* a new frame is waiting to be consumed   */
    int   terminated;             /* stream ended (END or ERROR frame kind)  */
} skeleton_buf_t;

/* Subscription callback: invoked by the SDK worker thread for every frame.
 *
 * schema guarantees 21 joints; drop malformed frames defensively. */
static void on_hand_skeleton(WujiFrameKind kind,
                             const struct WujiHandSkeleton *frame,
                             void *user_data)
{
    skeleton_buf_t *buf = (skeleton_buf_t *)user_data;

    /* WUJI_FRAME_KIND_END / WUJI_FRAME_KIND_ERROR are terminal: the callback
     * will never fire again after these (device disconnected or stream error). */
    if (kind == WUJI_FRAME_KIND_END || kind == WUJI_FRAME_KIND_ERROR) {
        pthread_mutex_lock(&buf->mu);
        buf->terminated = 1;
        pthread_mutex_unlock(&buf->mu);
        return;
    }

    if (kind != WUJI_FRAME_KIND_OK || !frame) return;

    /* Drop malformed frames: we need exactly 21 keypoints. */
    if (frame->joints_len < NUM_KEYPOINTS) return;

    pthread_mutex_lock(&buf->mu);
    for (size_t i = 0; i < NUM_KEYPOINTS; i++) {
        buf->kp[i * 3 + 0] = frame->joints[i].pose.position[0];
        buf->kp[i * 3 + 1] = frame->joints[i].pose.position[1];
        buf->kp[i * 3 + 2] = frame->joints[i].pose.position[2];
    }
    buf->have_frame = 1;
    pthread_mutex_unlock(&buf->mu);
}

/* ============================================================================
 * Startup buffers: the hand's own joint states and enable status
 *
 * Same drain-latest-frame shape as skeleton_buf_t, used only during startup.
 * ========================================================================== */

/* Both streams carry online joints only, so a short frame is refused rather than
 * ramped from. had_frame separates "no frame" from "an empty frame" - seen_len
 * leaves both at 0. */

typedef struct {
    pthread_mutex_t mu;
    float pos[NUM_JOINTS];
    int   have_frame;
    int   had_frame;
    size_t seen_len;
} joint_state_buf_t;

static void on_joint_state(WujiFrameKind kind,
                           const struct WujiJointStateFrame *frame,
                           void *user_data)
{
    joint_state_buf_t *buf = (joint_state_buf_t *)user_data;
    const int ok  = (kind == WUJI_FRAME_KIND_OK && frame);
    const size_t len = ok ? frame->joints_len : 0;

    pthread_mutex_lock(&buf->mu);
    /* Degraded frames rewrite have_frame too: returning early would leave a pose
     * the hand has already left standing as valid for the ramp to start from. */
    buf->have_frame = (len == NUM_JOINTS);
    if (buf->have_frame) {
        for (size_t i = 0; i < NUM_JOINTS; i++)
            buf->pos[i] = frame->joints[i].position;
    }
    if (ok) {
        buf->had_frame = 1;
        buf->seen_len = len;
    }
    pthread_mutex_unlock(&buf->mu);
}

typedef struct {
    pthread_mutex_t mu;
    int all_enabled;
    int had_frame;
    size_t seen_len;
} enable_buf_t;

static void on_joint_diagnostics(WujiFrameKind kind,
                                 const struct WujiJointDiagnosticsFrame *frame,
                                 void *user_data)
{
    enable_buf_t *buf = (enable_buf_t *)user_data;
    const int ok  = (kind == WUJI_FRAME_KIND_OK && frame);
    const size_t len = ok ? frame->joints_len : 0;

    /* All-Enabled on a short frame would start the ramp while the missing joints
     * are still coming up, and they would take the step the ramp exists to avoid.
     * Empty and error frames clear the flag for the same reason. */
    int all = (len == NUM_JOINTS);
    for (size_t i = 0; all && i < len; i++) {
        struct WujiJointStatus status;
        if (wuji_hand_2_decode_joint_status(frame->joints[i].status_word, &status)
                != WUJI_STATUS_OK
            || !status.enabled) {
            all = 0;
        }
    }

    pthread_mutex_lock(&buf->mu);
    buf->all_enabled = all;
    if (ok) {
        buf->had_frame = 1;
        buf->seen_len = len;
    }
    pthread_mutex_unlock(&buf->mu);
}

/* ============================================================================
 * Monotonic clock helpers
 * ========================================================================== */

static int64_t now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (int64_t)ts.tv_sec * 1000000000LL + (int64_t)ts.tv_nsec;
}

static void sleep_ns(int64_t ns) {
    if (ns <= 0) return;
    struct timespec req = { .tv_sec = ns / 1000000000LL,
                            .tv_nsec = ns % 1000000000LL };
    nanosleep(&req, NULL);
}

/* ============================================================================
 * Startup ramp (Wuji Hand 2 only)
 *
 * The first retargeted frame is otherwise a step input, and a Wuji Hand 2 has
 * no smoothing channel of its own, so the example ramps in software.
 * ========================================================================== */

/* Block until every online joint reports Enabled. Ramping into motors that are
 * not holding position yet would land the whole ramp at once when they engage. */
static int wait_enabled(struct WujiDevice *dev)
{
    enable_buf_t buf;
    memset(&buf, 0, sizeof(buf));
    pthread_mutex_init(&buf.mu, NULL);

    struct WujiSub *sub = NULL;
    if (wuji_hand_2_subscribe_joint_diagnostics(dev, on_joint_diagnostics, &buf, &sub)
            != WUJI_STATUS_OK) {
        fprintf(stderr, "subscribe_joint_diagnostics: %s\n", wuji_last_error());
        pthread_mutex_destroy(&buf.mu);
        return 1;
    }

    int ok = 0;
    int had = 0;
    size_t seen = 0;
    const int64_t deadline = now_ns() + 5000000000LL;
    while (!g_stop && !ok && now_ns() < deadline) {
        sleep_ns(50000000LL);
        pthread_mutex_lock(&buf.mu);
        ok = buf.all_enabled;
        had = buf.had_frame;
        seen = buf.seen_len;
        pthread_mutex_unlock(&buf.mu);
    }

    wuji_sub_close(sub);
    pthread_mutex_destroy(&buf.mu);
    if (!ok && !g_stop) {
        if (had)
            fprintf(stderr,
                    "Enable timeout - the last diagnostics frame reported %zu of %d "
                    "joints with not all of them Enabled\n", seen, NUM_JOINTS);
        else
            fprintf(stderr,
                    "Enable timeout - no joint_diagnostics frame arrived - "
                    "is the hand publishing?\n");
        return 1;
    }
    return 0;
}

/* Read where the hand actually is, so the ramp has somewhere to start from. */
static int read_current_pose(struct WujiDevice *dev, float out[NUM_JOINTS])
{
    joint_state_buf_t buf;
    memset(&buf, 0, sizeof(buf));
    pthread_mutex_init(&buf.mu, NULL);

    struct WujiSub *sub = NULL;
    if (wuji_hand_2_subscribe_joint_states(dev, on_joint_state, &buf, &sub)
            != WUJI_STATUS_OK) {
        fprintf(stderr, "subscribe_joint_states: %s\n", wuji_last_error());
        pthread_mutex_destroy(&buf.mu);
        return 1;
    }

    int got = 0;
    int had = 0;
    size_t seen = 0;
    const int64_t deadline = now_ns() + 3000000000LL;
    while (!g_stop && !got && now_ns() < deadline) {
        pthread_mutex_lock(&buf.mu);
        had = buf.had_frame;
        seen = buf.seen_len;
        if (buf.have_frame) {
            memcpy(out, buf.pos, NUM_JOINTS * sizeof(float));
            got = 1;
        }
        pthread_mutex_unlock(&buf.mu);
        if (!got) sleep_ns(2000000LL);
    }

    wuji_sub_close(sub);
    pthread_mutex_destroy(&buf.mu);
    if (!got && !g_stop) {
        if (had)
            fprintf(stderr,
                    "joint_states reports %zu online joints, expected %d - "
                    "every joint must be online to ramp\n", seen, NUM_JOINTS);
        else
            fprintf(stderr,
                    "Timed out waiting for joint_states - no frame arrived - "
                    "is the hand publishing?\n");
        return 1;
    }
    return 0;
}

/* Ease from `start` to the glove pose over RAMP_SEC, leaving the keypoints it
 * finished on in `out_last_kp`. The endpoint is re-retargeted every frame: the
 * operator keeps moving, so a frozen target would end the ramp with a step. */
static int ramp_to_glove(struct WujiJointCommandPublisher *pub,
                         struct WujiRetargetSession *session,
                         skeleton_buf_t *skel,
                         const float start[NUM_JOINTS],
                         float out_last_kp[NUM_KEYPOINTS * 3])
{
    const int64_t budget_ns = (int64_t)(1000000000LL / FPS);
    float kp[NUM_KEYPOINTS * 3];
    int   have_kp = 0;

    /* Nothing to ramp toward until the glove has published once. */
    const int64_t kp_deadline = now_ns() + 5000000000LL;
    while (!g_stop && !have_kp && now_ns() < kp_deadline) {
        pthread_mutex_lock(&skel->mu);
        if (skel->have_frame) {
            memcpy(kp, skel->kp, sizeof(kp));
            skel->have_frame = 0;
            have_kp = 1;
        }
        pthread_mutex_unlock(&skel->mu);
        if (!have_kp) sleep_ns(budget_ns);
    }
    if (!have_kp) {
        if (!g_stop)
            fprintf(stderr, "Timed out waiting for the glove's first hand_skeleton frame\n");
        return g_stop ? 0 : 1;
    }

    int steps = (int)(FPS * RAMP_SEC);
    if (steps < 1) steps = 1;

    float qpos[NUM_JOINTS];
    WujiJointCommand cmds[NUM_JOINTS];

    for (int i = 1; i <= steps && !g_stop; i++) {
        const int64_t step_start = now_ns();
        pthread_mutex_lock(&skel->mu);
        if (skel->have_frame) {
            memcpy(kp, skel->kp, sizeof(kp));
            skel->have_frame = 0;
        }
        pthread_mutex_unlock(&skel->mu);

        if (wuji_retarget_session_step(session, kp, qpos) != WUJI_STATUS_OK) {
            fprintf(stderr, "retarget_session_step (ramp): %s\n", wuji_last_error());
            return 1;
        }

        const float alpha = (float)i / (float)steps;
        for (int j = 0; j < NUM_JOINTS; j++) {
            cmds[j].position = start[j] + alpha * (qpos[j] - start[j]);
            cmds[j].velocity = 0.0f;
            cmds[j].effort   = 0.0f;
        }
        if (wuji_joint_command_publisher_send(pub, cmds) != WUJI_STATUS_OK) {
            fprintf(stderr, "publisher_send (ramp): %s\n", wuji_last_error());
            return 1;
        }
        /* Paced on this step's own start, same as the main loop: a slow step
         * lowers the rate instead of being repaid by a burst (see FPS above). */
        int64_t elapsed = now_ns() - step_start;
        sleep_ns(budget_ns - elapsed);
    }

    memcpy(out_last_kp, kp, NUM_KEYPOINTS * 3 * sizeof(float));
    return 0;
}

/* ============================================================================
 * WujiHand2 configuration + enable
 * ========================================================================== */

static int configure_wuji_hand_2(struct WujiDevice *dev)
{
    /* MIT impedance: hold each joint softly at its commanded position.
     * The firmware defaults to MIT control mode; we only set effort limit
     * and kp/kd, then enable. */
    if (wuji_hand_2_set_all_effort_limit(dev, 1.5f) != WUJI_STATUS_OK) {
        fprintf(stderr, "set_all_effort_limit: %s\n", wuji_last_error());
        return -1;
    }
    float kp[NUM_JOINTS], kd[NUM_JOINTS];
    for (int i = 0; i < NUM_JOINTS; i++) { kp[i] = 5.0f; kd[i] = 0.1f; }
    if (wuji_hand_2_set_all_mit_params(dev, kp, kd) != WUJI_STATUS_OK) {
        fprintf(stderr, "set_all_mit_params: %s\n", wuji_last_error());
        return -1;
    }
    if (wuji_hand_2_enable(dev, NULL) != WUJI_STATUS_OK) { /* NULL = all joints */
        fprintf(stderr, "hand_2_enable: %s\n", wuji_last_error());
        return -1;
    }
    return 0;
}

/* ============================================================================
 * WujiHand (gen1) configuration + enable
 * ========================================================================== */

static int configure_wuji_hand(struct WujiDevice *dev)
{
    if (wuji_hand_set_all_effort_limit(dev, 1.5f) != WUJI_STATUS_OK) {
        fprintf(stderr, "hand_set_all_effort_limit: %s\n", wuji_last_error());
        return -1;
    }
    if (wuji_hand_enable(dev) != WUJI_STATUS_OK) {
        fprintf(stderr, "hand_enable: %s\n", wuji_last_error());
        return -1;
    }
    return 0;
}

/* Pick one hand and one glove out of a scan result. A serial number, when given,
 * restricts the search to that device; without one the pick is unchanged:
 * WujiHand2 preferred over gen1, the last matching hand, the first glove.
 * An index comes back as n_dev when nothing matched. */
static void select_devices(const WujiDiscovered *list, size_t n_dev,
                           const char *hand_sn, const char *glove_sn,
                           size_t *out_hand_idx, size_t *out_glove_idx,
                           WujiDeviceType *out_hand_type)
{
    size_t hand_idx  = n_dev;
    size_t glove_idx = n_dev;
    WujiDeviceType hand_type = WUJI_DEVICE_TYPE_UNKNOWN;

    for (size_t i = 0; i < n_dev; i++) {
        bool hand_match  = hand_sn == NULL ||
                           strcmp(list[i].serial_number, hand_sn) == 0;
        bool glove_match = glove_sn == NULL ||
                           strcmp(list[i].serial_number, glove_sn) == 0;

        /* WujiHand2 preferred: overwrite a gen1 hand entry when we see a
         * WujiHand2, but never overwrite a WujiHand2 entry with gen1. */
        if (hand_match && list[i].device_id == WUJI_DEVICE_TYPE_WUJI_HAND_2) {
            hand_idx  = i;
            hand_type = list[i].device_id;
        } else if (hand_match && list[i].device_id == WUJI_DEVICE_TYPE_WUJI_HAND &&
                   hand_type != WUJI_DEVICE_TYPE_WUJI_HAND_2) {
            hand_idx  = i;
            hand_type = list[i].device_id;
        }

        if (glove_match && glove_idx == n_dev &&
            list[i].device_id == WUJI_DEVICE_TYPE_WUJI_GLOVE) {
            glove_idx = i;
        }
    }

    *out_hand_idx  = hand_idx;
    *out_glove_idx = glove_idx;
    *out_hand_type = hand_type;
}

/* ============================================================================
 * Main scan + teleop routine
 * ========================================================================== */

static int run_teleop(const char *hand_sn, const char *glove_sn)
{
    int ret = 0;

    /* ------------------------------------------------------------------
     * 1. Scan and locate one hand (WujiHand2 preferred, else WujiHand)
     *    and one glove.
     *    WujiHand2 is preferred: scanning overwrites a previously recorded
     *    gen1 hand when a WujiHand2 is seen, but not the reverse.
     * ------------------------------------------------------------------ */
    WujiDiscovered *list = NULL;
    size_t n_dev = 0;
    if (wuji_scan(&list, &n_dev) != WUJI_STATUS_OK) {
        fprintf(stderr, "wuji_scan failed: %s\n", wuji_last_error());
        return 1;
    }

    printf("Scan results:\n");
    for (size_t i = 0; i < n_dev; i++) {
        printf("  SN=%s  Type=%s  Address=%s\n",
               list[i].serial_number, list[i].model, list[i].address);
    }

    size_t hand_idx  = n_dev;
    size_t glove_idx = n_dev;
    WujiDeviceType hand_type = WUJI_DEVICE_TYPE_UNKNOWN;
    select_devices(list, n_dev, hand_sn, glove_sn, &hand_idx, &glove_idx, &hand_type);

    if (hand_idx == n_dev || glove_idx == n_dev) {
        if (hand_idx == n_dev) {
            if (hand_sn)
                printf("No Wuji Hand / Wuji Hand 2 found with SN %s\n", hand_sn);
            else
                printf("No Wuji Hand / Wuji Hand 2 found\n");
        } else {
            if (glove_sn)
                printf("No Wuji Glove found with SN %s\n", glove_sn);
            else
                printf("No Wuji Glove found\n");
        }
        wuji_discovered_free(list, n_dev);
        return 1;
    }

    /* ------------------------------------------------------------------
     * 2. Connect hand and glove.
     * ------------------------------------------------------------------ */
    WujiConnectTarget hand_tgt = {
        .kind  = WUJI_CONNECT_TARGET_KIND_SN,
        .value = list[hand_idx].serial_number,
    };
    WujiConnectTarget glove_tgt = {
        .kind  = WUJI_CONNECT_TARGET_KIND_SN,
        .value = list[glove_idx].serial_number,
    };

    struct WujiDevice *hand_dev  = NULL;
    struct WujiDevice *glove_dev = NULL;

    WujiStatus st;
    st = wuji_connect(&hand_tgt, "hand", NULL, &hand_dev);
    if (st != WUJI_STATUS_OK) {
        fprintf(stderr, "connect hand failed: %s\n", wuji_last_error());
        wuji_discovered_free(list, n_dev);
        return 1;
    }
    printf("Connected hand: %s\n", list[hand_idx].serial_number);

    if (g_stop) { wuji_discovered_free(list, n_dev); ret = 0; goto cleanup_hand_only; }

    st = wuji_connect(&glove_tgt, "glove", NULL, &glove_dev);
    if (st != WUJI_STATUS_OK) {
        fprintf(stderr, "connect glove failed: %s\n", wuji_last_error());
        wuji_discovered_free(list, n_dev);
        ret = 1;
        goto cleanup_hand_only;
    }
    printf("Connected glove: %s\n", list[glove_idx].serial_number);

    wuji_discovered_free(list, n_dev);
    list = NULL;

    if (g_stop) { ret = 0; goto cleanup_devs; }

    /* ------------------------------------------------------------------
     * 3. Configure + enable the hand.
     * ------------------------------------------------------------------ */
    bool is_hand2 = (hand_type == WUJI_DEVICE_TYPE_WUJI_HAND_2);
    WujiHandModel model = is_hand2 ? WUJI_HAND_MODEL_WUJI_HAND2
                                   : WUJI_HAND_MODEL_WUJI_HAND;

    int rc;
    if (is_hand2)
        rc = configure_wuji_hand_2(hand_dev);
    else
        rc = configure_wuji_hand(hand_dev);

    if (rc != 0) { ret = 1; goto cleanup_devs; }

    if (g_stop) { ret = 0; goto cleanup_disable; }

    /* ------------------------------------------------------------------
     * 4. Open the hand's command channel first - before building the
     *    retargeting session - so device communication remains active
     *    during setup.
     * ------------------------------------------------------------------ */

    /* Hand 2: streaming joint-command publisher. */
    struct WujiJointCommandPublisher *pub2 = NULL;
    /* Gen1 Hand: low-pass realtime controller. */
    struct WujiRealtimeController *ctrl1 = NULL;

    if (is_hand2) {
        if (wuji_hand_2_joint_command_publish(hand_dev, &pub2) != WUJI_STATUS_OK) {
            fprintf(stderr, "joint_command_publish: %s\n", wuji_last_error());
            ret = 1;
            goto cleanup_disable;
        }
    } else {
        WujiLowPass filter = { .cutoff_hz = 5.0 }; /* 5 Hz cutoff, like Python */
        if (wuji_hand_realtime_controller_open(hand_dev, filter, &ctrl1) != WUJI_STATUS_OK) {
            fprintf(stderr, "realtime_controller_open: %s\n", wuji_last_error());
            ret = 1;
            goto cleanup_disable;
        }
    }

    if (g_stop) { ret = 0; goto cleanup_channel; }

    /* ------------------------------------------------------------------
     * 5. Build the retargeting session (heavy call, takes a few seconds).
     * ------------------------------------------------------------------ */
    struct WujiRetargetSession *session = NULL;
    st = wuji_retarget_session_create(model, SIDE, &session);
    if (st != WUJI_STATUS_OK) {
        fprintf(stderr, "retarget_session_create failed (%d): %s\n",
                (int)st, wuji_last_error());
        ret = 1;
        goto cleanup_channel;
    }

    if (g_stop) { ret = 0; wuji_retarget_session_free(session); goto cleanup_channel; }

    /* ------------------------------------------------------------------
     * 6. Subscribe to the glove's hand_skeleton stream.
     *    Drain-latest-frame: the callback writes each new frame into the
     *    shared mutex-protected buffer; the main loop always picks up the
     *    most recent one, discarding stale frames.
     * ------------------------------------------------------------------ */
    skeleton_buf_t skel_buf;
    memset(&skel_buf, 0, sizeof(skel_buf));
    pthread_mutex_init(&skel_buf.mu, NULL);

    struct WujiSub *skel_sub = NULL;
    st = wuji_glove_subscribe_hand_skeleton(glove_dev, on_hand_skeleton, &skel_buf, &skel_sub);
    if (st != WUJI_STATUS_OK) {
        fprintf(stderr, "subscribe_hand_skeleton: %s\n", wuji_last_error());
        pthread_mutex_destroy(&skel_buf.mu);
        wuji_retarget_session_free(session);
        ret = 1;
        goto cleanup_channel;
    }

    float last_kp[NUM_KEYPOINTS * 3];
    int   have_last_kp = 0;

    /* ------------------------------------------------------------------
     * 6b. Wuji Hand 2 only: ease from the hand's current pose into the
     *     glove pose before the loop starts. See ramp_to_glove.
     * ------------------------------------------------------------------ */
    if (is_hand2 && !g_stop) {
        float start_pose[NUM_JOINTS];
        printf("Easing into the glove pose over %.1fs...\n", (double)RAMP_SEC);
        if (wait_enabled(hand_dev) != 0 ||
            read_current_pose(hand_dev, start_pose) != 0 ||
            ramp_to_glove(pub2, session, &skel_buf, start_pose, last_kp) != 0) {
            wuji_sub_close(skel_sub);
            pthread_mutex_destroy(&skel_buf.mu);
            wuji_retarget_session_free(session);
            ret = 1;
            goto cleanup_channel;
        }
        have_last_kp = 1;
    }

    /* ------------------------------------------------------------------
     * 7. 120 FPS main loop: read -> retarget -> send.
     * ------------------------------------------------------------------ */
    printf("Teleoperating (Ctrl+C to stop)...\n");

    const int64_t budget_ns = (int64_t)(1000000000LL / FPS);
    float qpos[NUM_JOINTS];

    while (!g_stop) {
        int64_t frame_start = now_ns();

        /* Take the latest frame from the shared buffer (if any). */
        int got_frame  = 0;
        int terminated = 0;
        pthread_mutex_lock(&skel_buf.mu);
        terminated = skel_buf.terminated;
        if (skel_buf.have_frame) {
            memcpy(last_kp, skel_buf.kp, sizeof(last_kp));
            skel_buf.have_frame = 0;
            got_frame = 1;
        }
        pthread_mutex_unlock(&skel_buf.mu);

        if (terminated) {
            fprintf(stderr,
                    "hand_skeleton stream ended (glove disconnected?) — stopping\n");
            ret = 1;
            break;
        }

        if (got_frame) have_last_kp = 1;

        if (!have_last_kp) {
            /* No frame yet: wait out the budget and try again. */
            sleep_ns(budget_ns - (now_ns() - frame_start));
            continue;
        }

        /* Retarget: map (21 x 3) keypoints -> 20 joint values. */
        st = wuji_retarget_session_step(session, last_kp, qpos);
        if (st != WUJI_STATUS_OK) {
            fprintf(stderr, "retarget_session_step: %s\n", wuji_last_error());
            ret = 1;
            break;
        }

        /* Send: one command per joint, position only (velocity=0, effort=0). */
        if (is_hand2) {
            WujiJointCommand cmds[NUM_JOINTS];
            for (int i = 0; i < NUM_JOINTS; i++) {
                cmds[i].position = qpos[i];
                cmds[i].velocity = 0.0f;
                cmds[i].effort   = 0.0f;
            }
            st = wuji_joint_command_publisher_send(pub2, cmds);
            if (st != WUJI_STATUS_OK) {
                fprintf(stderr, "publisher_send: %s\n", wuji_last_error());
                ret = 1;
                break;
            }
        } else {
            /* Gen1: realtime controller set_target_position. */
            st = wuji_hand_realtime_controller_set_target_position(ctrl1, qpos);
            if (st != WUJI_STATUS_OK) {
                fprintf(stderr, "set_target_position: %s\n", wuji_last_error());
                ret = 1;
                break;
            }
        }

        /* Frame budget sleep. */
        int64_t elapsed = now_ns() - frame_start;
        sleep_ns(budget_ns - elapsed);
    }

    printf("\nStopping...\n");

    /* ------------------------------------------------------------------
     * 8. Cleanup (ordered: close streams -> disable -> release devices).
     * ------------------------------------------------------------------ */
    wuji_sub_close(skel_sub);
    pthread_mutex_destroy(&skel_buf.mu);
    wuji_retarget_session_free(session);

cleanup_channel:
    if (is_hand2) {
        if (pub2) wuji_joint_command_publisher_close(pub2);
    } else {
        if (ctrl1) wuji_hand_realtime_controller_close(ctrl1);
    }

cleanup_disable:
    if (is_hand2)
        (void)wuji_hand_2_disable(hand_dev, NULL); /* best-effort, ignore error */
    else
        (void)wuji_hand_disable(hand_dev);         /* best-effort, ignore error */

cleanup_devs:
    if (glove_dev) { wuji_dev_disconnect(glove_dev); wuji_dev_release(glove_dev); }

cleanup_hand_only:
    if (hand_dev)  { wuji_dev_disconnect(hand_dev);  wuji_dev_release(hand_dev);  }

    return ret;
}

/* ============================================================================
 * SDK user selection
 * ========================================================================== */

#define USER_ID_CAP    256
#define USER_NAME_CAP  128

/* Interactively pick the SDK user to run under and switch to it, writing the
 * previously selected user's id into out_prev_user_id so main() can restore it
 * on exit.
 *
 * Switching happens before any device is connected, so the glove reads the
 * selected user's hand model. Pressing Enter - or a closed stdin, as in a piped
 * run - keeps the current user; Ctrl+C at the prompt returns
 * WUJI_STATUS_ERR_CANCELLED without switching. */
static WujiStatus select_sdk_user(char *out_prev_user_id, size_t cap)
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

    long index = -1; /* -1 keeps the current user */
    for (;;) {
        printf("Select user [0-%zu], Enter keeps current: ", len - 1);
        fflush(stdout);

        char line[128];
        if (!fgets(line, sizeof(line), stdin)) {
            if (g_stop) { /* Ctrl+C interrupted the read */
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
        if (*s == '\0') break; /* Enter keeps the current user */

        char *stop = NULL;
        long value = strtol(s, &stop, 10);
        /* strtol also parses negatives, which would index from the end. */
        if (*stop != '\0' || value < 0 || (size_t)value >= len) {
            printf("Invalid selection: '%s'\n", s);
            continue;
        }
        index = value;
        break;
    }

    char selected_name[USER_NAME_CAP] = {0};
    if (index < 0) {
        snprintf(selected_name, sizeof(selected_name), "%s", previous_name);
        wuji_user_info_array_free(users, len);
    } else {
        /* users[] is freed before switching, so copy the id out first. */
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
        snprintf(selected_name, sizeof(selected_name), "%s",
                 selected.display_name ? selected.display_name : selected_user_id);
        wuji_user_info_free(&selected);
    }

    printf("Running as SDK user: %s\n", selected_name);
    return WUJI_STATUS_OK;
}

/* ============================================================================
 * Entry point: user management + teleop
 * ========================================================================== */

/* ============================================================================
 * Command line
 * ========================================================================== */

typedef struct {
    const char *hand_sn;
    const char *glove_sn;
} TeleopArgs;

static void print_usage(FILE *out, const char *prog)
{
    fprintf(out,
            "Usage: %s [--hand-sn SN] [--glove-sn SN]\n"
            "  --hand-sn SN   Serial number of the hand to drive.\n"
            "  --glove-sn SN  Serial number of the glove to read.\n"
            "Without a serial number the example drives whichever hand and glove\n"
            "the scan reports, which is an arbitrary one when several are\n"
            "reachable. Pass the serial numbers whenever that is the case.\n",
            prog);
}

/* Returns 0 to continue, 1 when --help was handled, -1 on a bad argument. */
static int parse_args(int argc, char **argv, TeleopArgs *args)
{
    *args = (TeleopArgs){0};
    for (int i = 1; i < argc; i++) {
        const char *arg = argv[i];
        const char **slot = NULL;
        if (strcmp(arg, "-h") == 0 || strcmp(arg, "--help") == 0) {
            print_usage(stdout, argv[0]);
            return 1;
        } else if (strcmp(arg, "--hand-sn") == 0) {
            slot = &args->hand_sn;
        } else if (strcmp(arg, "--glove-sn") == 0) {
            slot = &args->glove_sn;
        } else {
            fprintf(stderr, "Unknown option: %s\n", arg);
            print_usage(stderr, argv[0]);
            return -1;
        }
        if (++i >= argc) {
            fprintf(stderr, "Missing value for %s\n", arg);
            print_usage(stderr, argv[0]);
            return -1;
        }
        *slot = argv[i];
    }
    return 0;
}

int main(int argc, char **argv)
{
    TeleopArgs args;
    int parse_st = parse_args(argc, argv, &args);
    if (parse_st != 0)
        return parse_st > 0 ? 0 : 2;

    /* sa_flags = 0 (no SA_RESTART) so SIGINT interrupts the blocking fgets() in
     * the user prompt with EINTR instead of restarting it. signal() gives no
     * such guarantee: glibc takes its semantics from the feature-test macros,
     * and musl and bionic always set SA_RESTART. */
    struct sigaction sigint_action;
    memset(&sigint_action, 0, sizeof(sigint_action));
    sigint_action.sa_handler = on_sigint;
    sigemptyset(&sigint_action.sa_mask);
    sigint_action.sa_flags = 0;
    if (sigaction(SIGINT, &sigint_action, NULL) != 0) {
        fprintf(stderr, "sigaction(SIGINT) failed\n");
        return 1;
    }

    WujiInitOptions opts = { .log_level = 3 };
    if (wuji_init(&opts) != WUJI_STATUS_OK) {
        fprintf(stderr, "wuji_init failed: %s\n", wuji_last_error());
        return 1;
    }

    /* Pick the SDK user to run under. This has to happen before connecting: the
     * glove reads the selected user's hand model at connect time. Calibrate the
     * glove under a named user to teleoperate on that user's hand model
     * (calibrating under the default user has no effect).
     *
     * Integrating this example and don't need user selection? Drop this call and
     * the restore block below - the SDK then keeps running as whatever user is
     * currently selected. */
    char previous_user_id[USER_ID_CAP] = {0};
    WujiStatus select_st = select_sdk_user(previous_user_id, sizeof(previous_user_id));
    if (select_st != WUJI_STATUS_OK) {
        wuji_shutdown();
        return select_st == WUJI_STATUS_ERR_CANCELLED ? 130 : 1;
    }

    int exit_code = run_teleop(args.hand_sn, args.glove_sn);

    /* Restore the previously selected SDK user on exit. The default user's id is
     * the empty string, so that case goes through wuji_switch_to_default_user
     * rather than wuji_switch_user(""). */
    WujiUserInfo restored;
    memset(&restored, 0, sizeof(restored));
    WujiStatus restore_st = previous_user_id[0] != '\0'
                                ? wuji_switch_user(previous_user_id, &restored)
                                : wuji_switch_to_default_user(&restored);
    if (restore_st != WUJI_STATUS_OK) {
        fprintf(stderr, "Failed to restore previous SDK user: %s\n", wuji_last_error());
        exit_code = 1;
    } else {
        printf("Restored SDK user: %s\n",
               restored.display_name ? restored.display_name : previous_user_id);
        wuji_user_info_free(&restored);
    }

    wuji_shutdown();
    return exit_code;
}
