/*
 * Purpose:
 *   Set and read back a custom hand model path.
 * Prerequisites:
 *   Install the C SDK, connect a Wuji Glove, and select a named SDK user.
 *   Provide a readable hand URDF file; the default SDK user cannot set one.
 * Usage:
 *   Build with CMake as described in ../README.md, selecting 10_set_hand_model_path.
 *   ./build/glove/10_set_hand_model_path --hand-model-path /absolute/path/to/hand.urdf
 *   ./build/glove/10_set_hand_model_path --hand-model-path /absolute/path/to/hand.urdf --sn DEVICE_SN
 * Expected Behavior:
 *   Print the configured URDF path, then disconnect.
 *   Exit 0 on completion or Ctrl-C, 1 on execution or cleanup failure,
 *   and 2 on invalid arguments. Errors go to stderr.
 */
#define _XOPEN_SOURCE 700
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
        report_sdk_error(#call); \
        result = 1; goto cleanup; \
    } \
} while (0)

int main(int argc, char **argv) {
    const char *sn = NULL;
    char *path = NULL;
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--sn") == 0 && i + 1 < argc && argv[i + 1][0] != '-') {
            sn = argv[++i];
        } else if (strncmp(argv[i], "--sn=", 5) == 0) {
            sn = argv[i] + 5;
        } else if (strcmp(argv[i], "--hand-model-path") == 0 && i + 1 < argc) {
            path = argv[++i];
        } else {
            fprintf(stderr, "Invalid arguments. Usage: %s [--sn DEVICE_SN] --hand-model-path FILE\n", argv[0]);
            return 2;
        }
        if (sn != NULL && sn[0] == '\0') {
            fprintf(stderr, "--sn must not be empty\n");
            return 2;
        }
    }

    if (path == NULL || path[0] == '\0') {
        fprintf(stderr, "--hand-model-path is required\n"); return 2;
    }
    path = realpath(path, NULL);
    if (path == NULL) {
        fprintf(stderr, "Cannot resolve URDF path: %s\n", strerror(errno));
        return 2;
    }
    FILE *file = fopen(path, "r");
    if (file == NULL) {
        fprintf(stderr, "URDF is not readable: %s\n", path);
        free(path);
        return 2;
    }
    fclose(file);
    if (signal(SIGINT, on_sigint) == SIG_ERR) {
        fprintf(stderr, "install Ctrl-C handler failed: %s\n", strerror(errno));
        free(path);
        return 1;
    }

    int result = 0;
    char *path_buffer = NULL;
    size_t needed = 0;
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

    CHECK(wuji_glove_set_hand_model_path(device, path));
    WujiStatus status = wuji_glove_get_hand_model_path(device, NULL, 0, &needed);
    if (status != WUJI_STATUS_ERR_BUFFER_TOO_SMALL || needed == 0) {
        report_sdk_error("wuji_glove_get_hand_model_path"); result = 1; goto cleanup;
    }
    path_buffer = malloc(needed);
    if (path_buffer == NULL) { fprintf(stderr, "Error: out of memory\n"); result = 1; goto cleanup; }
    CHECK(wuji_glove_get_hand_model_path(device, path_buffer, needed, &needed));
    printf("hand_model_path: %s\n", path_buffer);
cleanup:
    free(path);
    free(path_buffer);
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
