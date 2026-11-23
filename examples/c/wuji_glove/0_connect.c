/*
 * Purpose:
 *   Scan for devices, then connect to one Wuji Glove and print its identity.
 * Prerequisites:
 *   Install the C SDK.
 *   A reachable Wuji Glove running firmware 1.0.4 or later.
 *   Automatic connection requires exactly one discovered device;
 *   use --sn when several are present.
 * Usage:
 *   Build with CMake as described in ../README.md, selecting target 0_connect.
 *   ./build/glove/0_connect
 *   ./build/glove/0_connect --sn DEVICE_SN
 * Expected Behavior:
 *   Print the device count and each device's serial number, type, and address.
 *   Print the connected serial number, firmware version, hardware version,
 *   and handedness, then list parameters and topics and disconnect.
 *   The SN connection demonstrates explicit timeout and retry options.
 *   Exit 0 on completion or Ctrl-C with successful cleanup, 1 on an operation
 *   or cleanup failure, and 2 on invalid arguments. Errors go to stderr.
 */

#include "wuji_sdk.h"

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
    /* By address:
     * WujiConnectTarget target = {.kind = WUJI_CONNECT_TARGET_KIND_ADDR, .value = "192.168.1.100:50001"};
     * return wuji_connect(&target, "example", NULL, device);
     */
    /* By handedness, restricted to this device type:
     * const uint8_t device_type = WUJI_DEVICE_TYPE_WUJI_GLOVE;
     * WujiConnectTarget target = {
     *     .kind = WUJI_CONNECT_TARGET_KIND_HANDEDNESS,
     *     .value = NULL,
     *     .handedness = WUJI_HANDEDNESS_LEFT,
     *     .device_type = &device_type,
     * };
     * return wuji_connect(&target, "example", NULL, device);
     */
    if (sn != NULL) {
        WujiConnectTarget target = {.kind = WUJI_CONNECT_TARGET_KIND_SN, .value = sn};
        WujiConnectOptions options = wuji_connect_options_default();
        options.timeout_ms = 1000;
        options.retry_count = 3;
        return wuji_connect(&target, "example", &options, device);
    }
    return wuji_auto_connect("example", device);
}

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
        if (sn[0] == '\0') {
            fprintf(stderr, "--sn must not be empty\n");
            return 2;
        }
    }

    if (signal(SIGINT, on_sigint) == SIG_ERR) {
        fprintf(stderr, "install Ctrl-C handler failed: %s\n", strerror(errno));
        return 1;
    }

    int result = 0;
    int initialized = 0;
    WujiDevice *device = NULL;
    WujiDiscoveredDevice *devices = NULL;
    size_t count = 0;
    WujiParam *params = NULL;
    size_t param_count = 0;
    WujiTopic *topics = NULL;
    size_t topic_count = 0;
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
    if (wuji_scan_devices(NULL, &devices, &count) != WUJI_STATUS_OK) {
        report_sdk_error("wuji_scan_devices");
        result = 1;
        goto cleanup;
    }
    if (stop_requested) goto cleanup;
    printf("Found %zu device(s)\n", count);
    for (size_t i = 0; i < count && !stop_requested; ++i) {
        printf("SN=%s, Type=%s, Address=%s\n",
               devices[i].serial_number, devices[i].model, devices[i].address);
    }
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
    char firmware[128];
    if (wuji_glove_get_version(device, firmware, sizeof(firmware), NULL) != WUJI_STATUS_OK) {
        report_sdk_error("wuji_glove_get_version"); result = 1; goto cleanup;
    }
    printf("Firmware version: %s\n", firmware);
    WujiHwVersion hardware;
    if (wuji_glove_get_hw_version(device, &hardware) != WUJI_STATUS_OK) {
        report_sdk_error("wuji_glove_get_hw_version"); result = 1; goto cleanup;
    }
    printf("Hardware version: %u.%u.%u\n", hardware.major, hardware.minor, hardware.patch);
    wuji_hw_version_free(&hardware);
    char side[16];
    if (wuji_glove_get_hand_side(device, side, sizeof(side), NULL) != WUJI_STATUS_OK) {
        report_sdk_error("wuji_glove_get_hand_side"); result = 1; goto cleanup;
    }
    printf("Handedness: %s\n", side);

    if (stop_requested) goto cleanup;
    if (wuji_dev_list_params(device, &params, &param_count) != WUJI_STATUS_OK) {
        report_sdk_error("wuji_dev_list_params"); result = 1; goto cleanup;
    }
    for (size_t i = 0; i < param_count && !stop_requested; ++i) {
        printf("Parameter: %s, readable=%d, writable=%d\n",
               params[i].path, params[i].readable, params[i].writable);
    }
    if (stop_requested) goto cleanup;
    if (wuji_dev_list_topics(device, &topics, &topic_count) != WUJI_STATUS_OK) {
        report_sdk_error("wuji_dev_list_topics"); result = 1; goto cleanup;
    }
    for (size_t i = 0; i < topic_count && !stop_requested; ++i) {
        printf("Topic: %s\n", topics[i].path);
    }

cleanup:
    wuji_topic_array_free(topics, topic_count);
    wuji_param_array_free(params, param_count);
    wuji_discovered_device_free(devices, count);
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
