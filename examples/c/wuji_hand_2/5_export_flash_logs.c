/*
 * Purpose:
 *   Export on-flash log history to a JSONL file.
 * Prerequisites:
 *   Install the C SDK.
 *   A reachable Wuji Hand 2. Automatic connection requires exactly one
 *   discovered device; use --sn when several are present.
 * Usage:
 *   Build with CMake as described in ../README.md, selecting target 5_export_flash_logs.
 *   ./build/5_export_flash_logs
 *   ./build/5_export_flash_logs --sn DEVICE_SN --out DIRECTORY
 * Expected Behavior:
 *   Print the decoded frame count and output path. Default: ~/.wuji/logs.
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
    if (sn != NULL) {
        WujiConnectTarget target = {.kind = WUJI_CONNECT_TARGET_KIND_SN, .value = sn};
        return wuji_connect(&target, "example", NULL, device);
    }
    return wuji_auto_connect("example", device);
}

#define CHECK(call) do { \
    if (stop_requested) goto cleanup; \
    if ((call) != WUJI_STATUS_OK) { \
        report_sdk_error(#call); result = 1; goto cleanup; \
    } \
} while (0)

int main(int argc, char **argv) {
    const char *sn = NULL;
    const char *out_dir = NULL;
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--sn") == 0 && i + 1 < argc && argv[i + 1][0] != '-') {
            sn = argv[++i];
        } else if (strncmp(argv[i], "--sn=", 5) == 0) {
            sn = argv[i] + 5;
        } else if (strcmp(argv[i], "--out") == 0 && i + 1 < argc && argv[i + 1][0] != '-') {
            out_dir = argv[++i];
        } else if (strncmp(argv[i], "--out=", 6) == 0) {
            out_dir = argv[i] + 6;
        } else {
            fprintf(stderr, "Invalid arguments. Usage: %s [--sn DEVICE_SN] [--out DIRECTORY]\n", argv[0]);
            return 2;
        }
        if ((sn && sn[0] == '\0') || (out_dir && out_dir[0] == '\0')) {
            fprintf(stderr, "--sn and --out must not be empty\n");
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
    if (device_type != WUJI_DEVICE_TYPE_WUJI_HAND_2) {
        fprintf(stderr, "Error: expected Wuji Hand 2, got device type %d\n", (int)device_type);
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
    WujiFlashLogExport exported;
    CHECK(wuji_hand_2_export_flash_logs(device, out_dir, &exported));
    printf("Decoded %u frames into %s\n", exported.frame_count, exported.path);
    wuji_hand_2_flash_log_export_free(&exported);

cleanup:
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
