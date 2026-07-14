/*
 * Wuji Hand — joint-command publisher.
 *
 * Auto-detects and connects to a Wuji Hand, enables motors, then streams
 * zero-position commands (20 WujiJointCommand) at 100 Hz for 3 s through
 * a LowPass realtime controller. Mirrors examples/python/wuji_hand/1.publish.py.
 *
 * !! WARNING: this MOVES the hand joints. Keep the hand clear. !!
 *
 * Build: see ../README.md.   Run: ./build/1_publish
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include <time.h>
#include <unistd.h>

#include "wuji_sdk.h"

/* WujiHand has 20 joints arranged finger-major (thumb first). */
#define JOINT_COUNT 20

#define PUB_HZ 100
#define HOLD_SECONDS 3.0
#define EFFORT_LIMIT_A 1.5f
#define LOWPASS_CUTOFF_HZ 5.0

static double now_s(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec / 1e9;
}

/* Print the thumb (first 4 finger-major joints), like the Python example. */
static void print_joint_state(const char *label, struct WujiDevice *dev) {
    float positions[JOINT_COUNT];
    if (wuji_hand_read_joint_state(dev, positions) != WUJI_STATUS_OK) {
        printf("%s: (unavailable) %s\n", label, wuji_last_error());
        return;
    }
    printf("%s: thumb_pos=[%+.3f, %+.3f, %+.3f, %+.3f]\n", label,
           (double)positions[0], (double)positions[1],
           (double)positions[2], (double)positions[3]);
}

int main(void) {
    int exit_code = 0;

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
    bool tactile = false;
    (void)wuji_hand_is_tactile_attached(dev, &tactile);
    printf("Connected: %s (%s, tactile_attached=%s)\n", sn,
           handedness == 0 ? "right" : handedness == 1 ? "left" : "unknown",
           tactile ? "yes" : "no");

    struct WujiRealtimeController *ctrl = NULL;
    struct WujiHandJointCommandPublisher *pub = NULL;

    print_joint_state("Initial joint state", dev);

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

    /* Hold pos=0: exactly 20 commands, position-only. */
    WujiJointCommand zeros[JOINT_COUNT];
    for (int i = 0; i < JOINT_COUNT; i++) {
        zeros[i] = (WujiJointCommand){.position = 0.0f, .velocity = 0.0f, .effort = 0.0f};
    }

    printf("Holding pos=0 for %.1fs at %d Hz ...\n", HOLD_SECONDS, PUB_HZ);
    const double dt = 1.0 / PUB_HZ;
    const double t0 = now_s();
    const double end = t0 + HOLD_SECONDS;
    long n = 0;
    while (now_s() < end) {
        if (wuji_hand_joint_command_publisher_send(pub, zeros) != WUJI_STATUS_OK) {
            fprintf(stderr, "publisher_send: %s\n", wuji_last_error());
            exit_code = 1;
            break;
        }
        n++;
        double wait = (t0 + dt * (double)n) - now_s();
        if (wait > 0) usleep((useconds_t)(wait * 1e6));
    }
    printf("Sent %ld command frames.\n", n);

    wuji_hand_joint_command_publisher_close(pub); /* NULL-safe */

close_ctrl:
    wuji_hand_realtime_controller_close(ctrl); /* NULL-safe */
    print_joint_state("Final joint state", dev);

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
