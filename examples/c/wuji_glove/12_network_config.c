/*
 * Purpose:
 *   Read or update the device IP address and data port.
 * Prerequisites:
 *   Install the C SDK and connect a reachable Wuji Glove.
 * Usage:
 *   Build with CMake as described in ../README.md, selecting 12_network_config.
 *   ./build/glove/12_network_config
 *   ./build/glove/12_network_config --sn DEVICE_SN --ip 192.168.1.100 --port 50001
 *   --port accepts 7400-65535, excluding the discovery port 50000.
 * Expected Behavior:
 *   Read and print the IP address and data port. Write only settings supplied
 *   with --ip or --port. Written settings are not restored on exit.
 *   IP and port changes take effect after a device reboot; this example does not reboot.
 *   If the new IP is outside the computer's subnet, discovery and connection may fail.
 *   Configure a computer interface on the device's new /24 subnet before reconnecting.
 *   Exit 0 on completion or Ctrl-C, 1 on execution or cleanup failure,
 *   and 2 on invalid arguments. Errors go to stderr.
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

int main(int argc, char **argv) {
    const char *sn = NULL;
    const char *ip = NULL;
    const char *port_text = NULL;
    unsigned long port = 0;
    for (int i = 1; i < argc; ++i) {
        const char **option = NULL;
        const char *value = NULL;
        if (strcmp(argv[i], "--sn") == 0) option = &sn;
        else if (strcmp(argv[i], "--ip") == 0) option = &ip;
        else if (strcmp(argv[i], "--port") == 0) option = &port_text;
        else if (strncmp(argv[i], "--sn=", 5) == 0) { option = &sn; value = argv[i] + 5; }
        else if (strncmp(argv[i], "--ip=", 5) == 0) { option = &ip; value = argv[i] + 5; }
        else if (strncmp(argv[i], "--port=", 7) == 0) { option = &port_text; value = argv[i] + 7; }
        if (option != NULL && value == NULL && i + 1 < argc && argv[i + 1][0] != '-') {
            value = argv[++i];
        }
        if (option == NULL || value == NULL || value[0] == '\0') {
            fprintf(stderr, "Invalid arguments. Usage: %s [--sn DEVICE_SN] [--ip IP_ADDRESS] [--port PORT]\n", argv[0]);
            return 2;
        }
        *option = value;
        if (option == &port_text) {
            char *end;
            errno = 0;
            port = strtoul(value, &end, 10);
            if (errno || end == value || *end != '\0') {
                fprintf(stderr, "--port must be an integer\n");
                return 2;
            }
        }
    }
    if (port_text != NULL && (port < 7400 || port > 65535 || port == 50000)) {
        fprintf(stderr, "--port must be between 7400 and 65535, excluding 50000\n");
        return 2;
    }

    if (signal(SIGINT, on_sigint) == SIG_ERR) {
        fprintf(stderr, "install Ctrl-C handler failed: %s\n", strerror(errno));
        return 1;
    }

    int result = 0;
    WujiDevice *device = NULL;
    if (wuji_init(NULL) != WUJI_STATUS_OK) {
        report_sdk_error("wuji_init");
        return 1;
    }
    if (stop_requested) goto cleanup;
    if (connect_device(sn, &device) != WUJI_STATUS_OK) {
        report_sdk_error("connect_device");
        result = 1;
        goto cleanup;
    }
    if (stop_requested) goto cleanup;
    WujiDeviceType device_type;
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
    char serial_number[64];
    if (wuji_dev_serial_number(device, serial_number, sizeof(serial_number)) < 0) {
        report_sdk_error("wuji_dev_serial_number");
        result = 1;
        goto cleanup;
    }
    printf("Connected: %s\n", serial_number);
    if (stop_requested) goto cleanup;
    char current_ip[64];
    uint16_t current_port;
    if (wuji_glove_get_ip(device, current_ip, sizeof(current_ip), NULL) != WUJI_STATUS_OK) {
        report_sdk_error("wuji_glove_get_ip"); result = 1; goto cleanup;
    }
    printf("IP: %s\n", current_ip);
    if (stop_requested) goto cleanup;
    if (wuji_glove_get_port(device, &current_port) != WUJI_STATUS_OK) {
        report_sdk_error("wuji_glove_get_port"); result = 1; goto cleanup;
    }
    printf("port: %u\n", (unsigned)current_port);
    if (stop_requested) goto cleanup;
    if (port_text != NULL) {
        if (wuji_glove_set_port(device, (uint16_t)port) != WUJI_STATUS_OK) {
            report_sdk_error("wuji_glove_set_port"); result = 1; goto cleanup;
        }
        printf("port setting sent: %lu\n", port);
    }
    if (stop_requested) goto cleanup;
    if (ip != NULL) {
        if (wuji_glove_set_ip(device, ip) != WUJI_STATUS_OK) {
            report_sdk_error("wuji_glove_set_ip"); result = 1; goto cleanup;
        }
        printf("IP setting sent: %s\n", ip);
    }

cleanup:
    if (device != NULL) {
        if (wuji_dev_disconnect(device) != WUJI_STATUS_OK) {
            report_sdk_error("wuji_dev_disconnect");
            result = 1;
        }
        wuji_dev_release(device);
    }
    wuji_shutdown();
    return result;
}
