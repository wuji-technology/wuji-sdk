#define _POSIX_C_SOURCE 200809L

/* Wuji SDK C - fixed 101-command cosine sweep on Hand 2 joint 0. */

#include <errno.h>
#include <math.h>
#include <signal.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>

#include "wuji_sdk.h"

#define JOINT_COUNT 20u
#define AMPLITUDE_RAD 0.02f
#define SWEEP_INTERVALS 100u
#define COMMAND_INTERVAL_NS 20000000L
#define PI_VALUE 3.14159265358979323846

static volatile sig_atomic_t g_stop = 0;

/* Request cooperative cleanup after Ctrl+C. */
static void on_sigint(int signal_number) {
    (void)signal_number;
    g_stop = 1;
}

/* Sleep for one fixed command interval. */
static int sleep_interval(void) {
    struct timespec duration = {0, COMMAND_INTERVAL_NS};
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
        if (frame_index < SWEEP_INTERVALS && sleep_interval() != 0) {
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
    send_result = send_sweep(publisher);
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
