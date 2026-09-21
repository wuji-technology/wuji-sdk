#define _POSIX_C_SOURCE 200809L

/* Wuji SDK C - fixed 101-command cosine sweep on Hand 2 joint 0. */

#include <errno.h>
#include <math.h>
#include <signal.h>
#include <stdatomic.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

#include "wuji_sdk.h"

#define JOINT_COUNT 20u
#define AMPLITUDE_RAD 0.02f
#define SWEEP_INTERVALS 100u
#define COMMAND_INTERVAL_NS 20000000L
#define RAMP_DURATION_NS 1000000000L
#define RAMP_INTERVAL_NS 1000000L
#define RAMP_STEPS (RAMP_DURATION_NS / RAMP_INTERVAL_NS)
#define CAPTURE_ATTEMPTS 400u
#define CAPTURE_WAIT_NS 5000000L
#define PI_VALUE 3.14159265358979323846

static volatile sig_atomic_t g_stop = 0;

/* Request cooperative cleanup after Ctrl+C. */
static void on_sigint(int signal_number) {
    (void)signal_number;
    g_stop = 1;
}

/* Sleep for one command interval, in nanoseconds. */
static int sleep_ns(long interval_ns) {
    struct timespec duration = {0, interval_ns};
    while (nanosleep(&duration, &duration) != 0) {
        if (errno != EINTR || g_stop) return -1;
    }
    return 0;
}

/* Scan and connect the only discovered device. */
static struct WujiDevice *connect_hand(void) {
    struct WujiDiscovered *devices = NULL;
    struct WujiConnectTarget target = {0};
    struct WujiDevice *hand = NULL;
    size_t count = 0u;

    if (wuji_scan(&devices, &count) != WUJI_STATUS_OK) return NULL;
    if (count == 1u) {
        target.kind = WUJI_CONNECT_TARGET_KIND_SN;
        target.value = devices[0].serial_number;
        if (wuji_connect(&target, "hand_2_mit_sweep", NULL, &hand)
            != WUJI_STATUS_OK) {
            hand = NULL;
        }
    }
    wuji_discovered_free(devices, count);
    return hand;
}

/* Build one command in the fixed joint-0 cosine sequence. */
static void generate_command(
    size_t frame_index,
    struct WujiJointCommand commands[JOINT_COUNT]) {
    const double phase =
        2.0 * PI_VALUE * (double)frame_index / (double)SWEEP_INTERVALS;
    size_t index;

    for (index = 0u; index < JOINT_COUNT; ++index) {
        commands[index].position = 0.0f;
        commands[index].velocity = 0.0f;
        commands[index].effort = 0.0f;
    }
    commands[0].position =
        AMPLITUDE_RAD * 0.5f * (float)(1.0 - cos(phase));
}

/* Send the fixed sequence with a 20 ms command interval. */
static int send_sweep(struct WujiJointCommandPublisher *publisher) {
    struct WujiJointCommand commands[JOINT_COUNT];
    size_t frame_index;

    for (frame_index = 0u; frame_index <= SWEEP_INTERVALS; ++frame_index) {
        if (g_stop) return -2;
        generate_command(frame_index, commands);
        if (wuji_joint_command_publisher_send(publisher, commands)
            != WUJI_STATUS_OK) {
            return -1;
        }
        if (frame_index < SWEEP_INTERVALS && sleep_ns(COMMAND_INTERVAL_NS) != 0) {
            return g_stop ? -2 : -1;
        }
    }
    return 0;
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
static int capture_start_positions(struct WujiDevice *hand,
                                   float start[JOINT_COUNT]) {
    capture_ctx_t ctx;
    struct WujiSub *subscription = NULL;
    struct timespec wait = {0, CAPTURE_WAIT_NS};
    size_t attempt;
    int captured;

    memset(&ctx, 0, sizeof(ctx));
    if (wuji_hand_2_subscribe_joint_states(hand, on_capture_frame, &ctx,
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

/* Blend one command between start and target by a 0..1 ratio. */
static void blend_command(const float start[JOINT_COUNT],
                          const float target[JOINT_COUNT],
                          double ratio,
                          struct WujiJointCommand commands[JOINT_COUNT]) {
    size_t index;

    for (index = 0u; index < JOINT_COUNT; ++index) {
        commands[index].position =
            start[index] + (target[index] - start[index]) * (float)ratio;
        commands[index].velocity = 0.0f;
        commands[index].effort = 0.0f;
    }
}

/* Ease from the captured pose to the opening frame with a cosine profile. */
static int send_ramp(struct WujiJointCommandPublisher *publisher,
                     const float start[JOINT_COUNT]) {
    struct WujiJointCommand target_commands[JOINT_COUNT];
    struct WujiJointCommand commands[JOINT_COUNT];
    float target[JOINT_COUNT];
    size_t index;
    size_t step;

    generate_command(0u, target_commands);
    for (index = 0u; index < JOINT_COUNT; ++index) {
        target[index] = target_commands[index].position;
    }
    for (step = 1u; step <= RAMP_STEPS; ++step) {
        const double ratio =
            0.5 * (1.0 - cos(PI_VALUE * (double)step / (double)RAMP_STEPS));
        if (g_stop) return -2;
        blend_command(start, target, ratio, commands);
        if (wuji_joint_command_publisher_send(publisher, commands)
            != WUJI_STATUS_OK) {
            return -1;
        }
        if (step < RAMP_STEPS && sleep_ns(RAMP_INTERVAL_NS) != 0) {
            return g_stop ? -2 : -1;
        }
    }
    return 0;
}

/* Connect, check 20 online joints, enable, send, and clean up. */
static int run_device(void) {
    struct WujiInitOptions init_options = {.log_level = 0};
    struct WujiDevice *hand = NULL;
    struct WujiJointCommandPublisher *publisher = NULL;
    uint8_t online = 0u;
    int initialized = 0;
    int cleanup_failed = 0;
    int exit_code = 1;
    int send_result;
    float start[JOINT_COUNT];

    if (wuji_init(&init_options) != WUJI_STATUS_OK) {
        fprintf(stderr, "error: wuji_init failed: %s\n", wuji_last_error());
        goto cleanup;
    }
    initialized = 1;
    hand = connect_hand();
    if (hand == NULL) {
        fprintf(stderr, "error: expected exactly one device\n");
        goto cleanup;
    }
    if (wuji_hand_2_online_joints_count(hand, &online) != WUJI_STATUS_OK
        || online != JOINT_COUNT) {
        fprintf(stderr, "error: Hand 2 requires 20/20 online joints\n");
        goto cleanup;
    }
    if (wuji_hand_2_enable(hand, NULL) != WUJI_STATUS_OK) {
        fprintf(stderr, "error: enable failed: %s\n", wuji_last_error());
        goto cleanup;
    }
    if (wuji_hand_2_joint_command_publish(hand, &publisher)
        != WUJI_STATUS_OK) {
        fprintf(stderr, "error: create publisher failed: %s\n",
                wuji_last_error());
        goto cleanup;
    }
    send_result = capture_start_positions(hand, start);
    if (send_result == -2) {
        exit_code = 130;
        goto cleanup;
    }
    if (send_result == -3) {
        fprintf(stderr, "error: subscribe joint_states failed: %s\n",
                wuji_last_error());
        goto cleanup;
    }
    if (send_result != 0) {
        fprintf(stderr, "error: no complete joint_states frame to ramp from\n");
        goto cleanup;
    }
    send_result = send_ramp(publisher, start);
    if (send_result == -1) {
        fprintf(stderr, "error: send ramp failed: %s\n", wuji_last_error());
        goto cleanup;
    }
    if (send_result == 0) send_result = send_sweep(publisher);
    if (send_result == -2) {
        exit_code = 130;
    } else if (send_result == 0) {
        exit_code = 0;
    } else {
        fprintf(stderr, "error: send sweep failed: %s\n", wuji_last_error());
    }

cleanup:
    if (hand != NULL && wuji_hand_2_disable(hand, NULL) != WUJI_STATUS_OK) {
        cleanup_failed = 1;
    }
    wuji_joint_command_publisher_close(publisher);
    if (hand != NULL) {
        if (wuji_dev_disconnect(hand) != WUJI_STATUS_OK) cleanup_failed = 1;
        wuji_dev_release(hand);
    }
    if (initialized) wuji_shutdown();
    if (exit_code == 0 && cleanup_failed) exit_code = 1;
    if (cleanup_failed) {
        fprintf(stderr, "cleanup error: disable or disconnect failed\n");
    }
    return exit_code;
}

/* Run the fixed physical-device transaction. */
int main(void) {
    g_stop = 0;
    signal(SIGINT, on_sigint);
    return run_device();
}
