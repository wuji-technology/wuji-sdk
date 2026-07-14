/*
 * Wuji Hand — grasp loop (open/close finger motion).
 *
 * Auto-detects and connects to a Wuji Hand, then opens/closes the four
 * non-thumb fingers with a smooth cosine curl at 200 Hz until Ctrl+C,
 * printing a target-vs-actual readout every 0.2 s. Mirrors
 * examples/python/wuji_hand/2.grasp_loop.py.
 *
 * !! WARNING: this MOVES the hand joints. Keep the hand clear. !!
 *
 * Build: see ../README.md.   Run: ./build/2_grasp_loop   (Ctrl+C to stop)
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <signal.h>
#include <math.h>
#include <time.h>
#include <unistd.h>

#include "wuji_sdk.h"

/* WujiHand has 20 joints arranged finger-major (thumb first). */
#define JOINT_COUNT 20
#define JOINTS_PER_FINGER 4

#define PUB_HZ 200
#define EFFORT_LIMIT_A 1.5f
#define LOWPASS_CUTOFF_HZ 5.0
#define CURL_AMPLITUDE 0.8 /* peak curl ≈ 2 * 0.8 = 1.6 rad */
#define REPORT_INTERVAL_S 0.2

/* Fingers and joints that follow the curl. Thumb (finger 0) and the second
 * joint of each finger (J2) stay at 0, matching the reference open/close
 * gesture — same constants as the Python example. */
static const int ACTIVE_FINGERS[] = {1, 2, 3, 4}; /* index, middle, ring, pinky */
static const int CURL_JOINTS[] = {0, 2, 3};       /* J1, J3, J4 within a finger */
#define INDEX_FINGER 1

static volatile sig_atomic_t g_stop = 0;
static void on_sigint(int sig) { (void)sig; g_stop = 1; }

static double now_s(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

/* Fill the 20 finger-major targets for the given curl amount (radians). */
static void curl_target(double curl, float target[JOINT_COUNT]) {
    for (int i = 0; i < JOINT_COUNT; i++) target[i] = 0.0f;
    for (size_t f = 0; f < sizeof(ACTIVE_FINGERS) / sizeof(ACTIVE_FINGERS[0]); f++) {
        int base = ACTIVE_FINGERS[f] * JOINTS_PER_FINGER;
        for (size_t j = 0; j < sizeof(CURL_JOINTS) / sizeof(CURL_JOINTS[0]); j++) {
            target[base + CURL_JOINTS[j]] = (float)curl;
        }
    }
}

int main(void) {
    int exit_code = 0;
    signal(SIGINT, on_sigint);

    printf("Wuji SDK version: %s\n", wuji_version());

    WujiInitOptions opts = { .log_level = 3 };
    if (wuji_init(&opts) != WUJI_STATUS_OK) {
        fprintf(stderr, "wuji_init failed: %s\n", wuji_last_error());
        return 1;
    }

    /* Scan and connect to the first Wuji Hand (selected by device_id). */
    WujiDiscovered *list = NULL;
    size_t n_dev = 0;
    if (wuji_scan(&list, &n_dev) != WUJI_STATUS_OK) {
        fprintf(stderr, "wuji_scan failed: %s\n", wuji_last_error());
        wuji_shutdown();
        return 1;
    }
    if (n_dev == 0) {
        printf("No devices found\n");
        wuji_discovered_free(list, n_dev);
        wuji_shutdown();
        return 0;
    }
    size_t idx = n_dev;
    for (size_t i = 0; i < n_dev; i++) {
        printf("  SN=%s  Type=%s  Address=%s\n", list[i].serial_number, list[i].model, list[i].address);
        if (idx == n_dev && list[i].device_id == WUJI_DEVICE_TYPE_WUJI_HAND) idx = i;
    }
    if (idx == n_dev) {
        printf("No Wuji Hand found among the scanned devices\n");
        wuji_discovered_free(list, n_dev);
        wuji_shutdown();
        return 0;
    }
    char sn[64];
    snprintf(sn, sizeof(sn), "%s", list[idx].serial_number);
    wuji_discovered_free(list, n_dev);

    /* Connect by USB serial number. */
    struct WujiDevice *dev = NULL;
    WujiStatus st = wuji_hand_connect_sn(sn, NULL, &dev);
    if (st != WUJI_STATUS_OK) {
        fprintf(stderr, "wuji_hand_connect_sn(\"%s\") failed: %s\n", sn, wuji_last_error());
        wuji_shutdown();
        return 1;
    }

    /* Raw handedness encoding: 0=Right, 1=Left. */
    uint8_t handedness = 0xFF;
    (void)wuji_hand_get_handedness(dev, &handedness);
    printf("Connected: %s (%s)\n", sn,
           handedness == 0 ? "right" : handedness == 1 ? "left" : "unknown");

    struct WujiRealtimeController *ctrl = NULL;
    struct WujiHandJointCommandPublisher *pub = NULL;

    st = wuji_hand_set_all_effort_limit(dev, EFFORT_LIMIT_A);
    if (st != WUJI_STATUS_OK) {
        fprintf(stderr, "set_all_effort_limit: %s\n", wuji_last_error());
        exit_code = 1;
        goto cleanup;
    }
    st = wuji_hand_enable(dev);
    if (st != WUJI_STATUS_OK) {
        fprintf(stderr, "wuji_hand_enable: %s\n", wuji_last_error());
        exit_code = 1;
        goto cleanup;
    }
    /* Brief settle time so the firmware transitions to Enabled state. */
    usleep(200000); /* 200 ms */

    printf("Starting LowPass controller (%.1f Hz) ...\n", LOWPASS_CUTOFF_HZ);
    WujiLowPass filter = { .cutoff_hz = LOWPASS_CUTOFF_HZ };
    st = wuji_hand_realtime_controller_open(dev, filter, &ctrl);
    if (st != WUJI_STATUS_OK) {
        fprintf(stderr, "realtime_controller_open: %s\n", wuji_last_error());
        exit_code = 1;
        goto disable;
    }

    st = wuji_hand_joint_command_publish(dev, &pub);
    if (st != WUJI_STATUS_OK) {
        fprintf(stderr, "joint_command_publish: %s\n", wuji_last_error());
        exit_code = 1;
        goto close_ctrl;
    }

    printf("Opening/closing fingers at %d Hz. Ctrl+C to stop.\n\n", PUB_HZ);
    const double dt = 1.0 / PUB_HZ;
    double x = 0.0;
    long n = 0;
    const double t0 = now_s();
    double next_report = t0;
    float target[JOINT_COUNT];
    float actual[JOINT_COUNT];
    WujiJointCommand cmds[JOINT_COUNT];

    while (!g_stop) {
        double curl = (1.0 - cos(x)) * CURL_AMPLITUDE;
        curl_target(curl, target);
        for (int i = 0; i < JOINT_COUNT; i++) {
            cmds[i] = (WujiJointCommand){.position = target[i], .velocity = 0.0f, .effort = 0.0f};
        }
        if (wuji_hand_joint_command_publisher_send(pub, cmds) != WUJI_STATUS_OK) {
            fprintf(stderr, "publisher_send: %s\n", wuji_last_error());
            exit_code = 1;
            break;
        }
        n++;

        double now = now_s();
        if (now >= next_report) {
            if (wuji_hand_realtime_controller_get_actual_position(ctrl, actual) == WUJI_STATUS_OK) {
                const float *cmd_row = &target[INDEX_FINGER * JOINTS_PER_FINGER];
                const float *act_row = &actual[INDEX_FINGER * JOINTS_PER_FINGER];
                printf("[wuji_hand] curl=%+.3f  "
                       "index_cmd=[%+.3f, %+.3f, %+.3f, %+.3f]  "
                       "index_act=[%+.3f, %+.3f, %+.3f, %+.3f]\n",
                       curl,
                       (double)cmd_row[0], (double)cmd_row[1], (double)cmd_row[2], (double)cmd_row[3],
                       (double)act_row[0], (double)act_row[1], (double)act_row[2], (double)act_row[3]);
                fflush(stdout);
            }
            next_report = now + REPORT_INTERVAL_S;
        }

        x += M_PI / PUB_HZ;
        double wait = (t0 + dt * (double)n) - now_s();
        if (wait > 0) usleep((useconds_t)(wait * 1e6));
    }
    if (g_stop) printf("\n(interrupted)\n");

    wuji_hand_joint_command_publisher_close(pub); /* NULL-safe */

close_ctrl:
    wuji_hand_realtime_controller_close(ctrl); /* NULL-safe */

disable:
    if (wuji_hand_disable(dev) != WUJI_STATUS_OK) {
        fprintf(stderr, "disable failed: %s\n", wuji_last_error());
        exit_code = 1;
    }

cleanup:
    wuji_dev_disconnect(dev);
    wuji_dev_release(dev);
    wuji_shutdown();
    return exit_code;
}
