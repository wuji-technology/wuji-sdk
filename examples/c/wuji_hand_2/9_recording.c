/*
 * Purpose:
 *   Record Wuji Hand 2 streams to an MCAP file.
 * Prerequisites:
 *   Install the C SDK and connect a reachable Wuji Hand 2.
 * Usage:
 *   Build with CMake as described in ../README.md, selecting 9_recording.
 *   ./build/9_recording --output recording.mcap
 *   ./build/9_recording --output recording.mcap --sn DEVICE_SN
 *
 *   Options:
 *     --compression {lz4,zstd,none}: compression format (default: lz4).
 * Expected Behavior:
 *   Record joint_states and joint_diagnostics for 30 seconds, pause for
 *   10 seconds, then record for 30 more seconds. Print live status and quality metrics,
 *   quality alerts when raised, and the final recording summary.
 *   Choose a new output path for each recording; existing files are overwritten.
 *   Exit 0 on completion or Ctrl-C, 1 on execution or cleanup failure,
 *   and 2 on invalid arguments. Errors go to stderr.
 */
#define _POSIX_C_SOURCE 200809L

#include "wuji_sdk.h"
#include <time.h>

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

#include <inttypes.h>
#include <stdatomic.h>
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

static void sleep_ms(unsigned ms) {
#ifdef _WIN32
    Sleep(ms);
#else
    struct timespec duration = {.tv_sec = ms / 1000, .tv_nsec = (long)(ms % 1000) * 1000000L};
    nanosleep(&duration, NULL);
#endif
}

static uint64_t now_ms(void) {
#ifdef _WIN32
    return GetTickCount64();
#else
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (uint64_t)now.tv_sec * 1000 + (uint64_t)now.tv_nsec / 1000000;
#endif
}

static atomic_int monitor_failed = 0;

static int monitor_frame_ok(WujiFrameKind kind, const void *payload,
                            uint64_t lag_count, const char *error_message) {
    if (kind == WUJI_FRAME_KIND_END) return 0;
    if (kind == WUJI_FRAME_KIND_LAG) {
        fprintf(stderr, "Warning: lost %" PRIu64 " recording alerts\n", lag_count);
        return 0;
    }
    if (kind == WUJI_FRAME_KIND_OK && payload != NULL) return 1;
    fprintf(stderr, "Recording monitor failed: %s\n",
            error_message ? error_message : "invalid monitor event");
    atomic_store(&monitor_failed, 1);
    return 0;
}

static void on_status(WujiFrameKind kind, const WujiRecordingStatus *status,
                      uint64_t lag_count, const char *error_message, void *user_data) {
    (void)user_data;
    if (!monitor_frame_ok(kind, status, lag_count, error_message)) return;
    const char *state = status->state == WUJI_RECORDING_STATE_RECORDING ? "recording" : "paused";
    printf("Status: %s, frames=%" PRIu64 ", duration=%.1f s\n",
           state, status->frame_count, status->duration_s);
}

static void on_metrics(WujiFrameKind kind, const WujiRecordingMetrics *metrics,
                       uint64_t lag_count, const char *error_message, void *user_data) {
    (void)user_data;
    if (!monitor_frame_ok(kind, metrics, lag_count, error_message)) return;
    printf("Quality: drop_rate=%.3f, jitter=%.1f us, sync_offset=%.3f ms, sync_rate=%.3f\n",
           metrics->frame_drop_rate, metrics->frame_jitter_us,
           metrics->sync_offset_ms, metrics->sync_rate);
}

static void on_alert(WujiFrameKind kind, const WujiRecordingAlert *alert,
                     uint64_t lag_count, const char *error_message, void *user_data) {
    (void)user_data;
    if (!monitor_frame_ok(kind, alert, lag_count, error_message)) return;
    fprintf(stderr, "Alert: %s, value=%.3f, threshold=%.3f: %s\n",
            alert->metric, alert->current_value, alert->threshold, alert->message);
}

static void wait_seconds(unsigned seconds) {
    uint64_t deadline = now_ms() + (uint64_t)seconds * 1000;
    while (!stop_requested && !atomic_load(&monitor_failed)) {
        uint64_t now = now_ms();
        if (now >= deadline) break;
        uint64_t remaining = deadline - now;
        sleep_ms((unsigned)(remaining < 100 ? remaining : 100));
    }
}

#define CHECK(call) do { \
    if (stop_requested || atomic_load(&monitor_failed)) goto cleanup; \
    if ((call) != WUJI_STATUS_OK) { \
        report_sdk_error(#call); \
        result = 1; goto cleanup; \
    } \
} while (0)

static int run(int argc, char **argv) {
    const char *sn = NULL;
    const char *output = NULL;
    const char *compression = "lz4";
    WujiRecordingOptions options = {.compression = WUJI_RECORDING_COMPRESSION_LZ4, .chunk_size = 0};
    for (int i = 1; i < argc; ++i) {
        const char **option = NULL;
        const char *value = NULL;
        if (strcmp(argv[i], "--sn") == 0) option = &sn;
        else if (strcmp(argv[i], "--output") == 0) option = &output;
        else if (strcmp(argv[i], "--compression") == 0) option = &compression;
        else if (strncmp(argv[i], "--sn=", 5) == 0) { option = &sn; value = argv[i] + 5; }
        else if (strncmp(argv[i], "--output=", 9) == 0) { option = &output; value = argv[i] + 9; }
        else if (strncmp(argv[i], "--compression=", 14) == 0) { option = &compression; value = argv[i] + 14; }
        if (option != NULL && value == NULL && i + 1 < argc && argv[i + 1][0] != '-') {
            value = argv[++i];
        }
        if (option == NULL || value == NULL) {
            fprintf(stderr, "Invalid arguments. Usage: %s [--sn DEVICE_SN] --output FILE [--compression lz4|zstd|none]\n", argv[0]);
            return 2;
        }
        *option = value;
        if (option == &compression) {
            if (strcmp(compression, "lz4") == 0) options.compression = WUJI_RECORDING_COMPRESSION_LZ4;
            else if (strcmp(compression, "zstd") == 0) options.compression = WUJI_RECORDING_COMPRESSION_ZSTD;
            else if (strcmp(compression, "none") == 0) options.compression = WUJI_RECORDING_COMPRESSION_NONE;
            else { fprintf(stderr, "Invalid compression: %s\n", compression); return 2; }
        }
    }
    if (sn != NULL && sn[0] == '\0') {
        fprintf(stderr, "--sn must not be empty\n");
        return 2;
    }

    if (output == NULL || output[0] == '\0') {
        fprintf(stderr, "--output is required\n"); return 2;
    }
    if (signal(SIGINT, on_sigint) == SIG_ERR) {
        fprintf(stderr, "install Ctrl-C handler failed: %s\n", strerror(errno));
        return 1;
    }

    int result = 0;
    WujiTopicRecorder *recorder = NULL;
    WujiRecordingHandle *handle = NULL;
    WujiRecordingSubscription *status_sub = NULL;
    WujiRecordingSubscription *metrics_sub = NULL;
    WujiRecordingSubscription *alert_sub = NULL;
    WujiRecordingSummary summary = {0};
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

    CHECK(wuji_topic_recorder_create(&options, &recorder));
    CHECK(wuji_topic_recorder_record(recorder, device, "joint_states"));
    CHECK(wuji_topic_recorder_record(recorder, device, "joint_diagnostics"));
    CHECK(wuji_topic_recorder_start(recorder, output, &handle));
    CHECK(wuji_recording_handle_watch_status(handle, on_status, NULL, &status_sub));
    CHECK(wuji_recording_handle_watch_metrics(handle, on_metrics, NULL, &metrics_sub));
    CHECK(wuji_recording_handle_watch_alerts(handle, on_alert, NULL, &alert_sub));
    printf("Recording to %s\n", output);
    wait_seconds(30);
    CHECK(wuji_recording_handle_pause(handle));
    printf("Paused\n");
    wait_seconds(10);
    CHECK(wuji_recording_handle_resume(handle));
    printf("Resumed\n");
    wait_seconds(30);
cleanup:
    if (handle != NULL) {
        if (wuji_recording_handle_stop(handle, &summary) != WUJI_STATUS_OK) {
            report_sdk_error("wuji_recording_handle_stop"); result = 1;
        } else {
            printf("Recorded %" PRIu64 " frames, %" PRIu64 " bytes, %.1f seconds\n",
                   summary.total_frames, summary.file_size, summary.duration_s);
            for (size_t i = 0; i < summary.channels_len; ++i) {
                printf("%s: %" PRIu64 " frames\n", summary.channels[i].channel_name,
                       summary.channels[i].frame_count);
            }
        }
        wuji_recording_summary_free(&summary);
    }
    wuji_recording_subscription_close(alert_sub);
    wuji_recording_subscription_close(metrics_sub);
    wuji_recording_subscription_close(status_sub);
    if (handle != NULL) wuji_recording_handle_free(handle);
    if (atomic_load(&monitor_failed)) result = 1;
    if (recorder != NULL) wuji_topic_recorder_free(recorder);
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

#ifdef _WIN32
#include <shellapi.h>
#endif
#ifdef _WIN32
static char *utf16_to_utf8(const wchar_t *wide) {
    int len = WideCharToMultiByte(CP_UTF8, WC_ERR_INVALID_CHARS, wide, -1, NULL, 0, NULL, NULL);
    if (len <= 0) return NULL;
    char *utf8 = (char *)malloc((size_t)len);
    if (utf8 == NULL) return NULL;
    if (WideCharToMultiByte(CP_UTF8, WC_ERR_INVALID_CHARS, wide, -1, utf8, len, NULL, NULL) <= 0) {
        free(utf8);
        return NULL;
    }
    return utf8;
}
static void free_utf8_argv(int argc, char **argv) {
    for (int i = 0; i < argc; ++i) free(argv[i]);
    free(argv);
}
#endif
#ifdef _WIN32
int main(void) {
    int wide_argc = 0;
    wchar_t **wide_argv = CommandLineToArgvW(GetCommandLineW(), &wide_argc);
    if (wide_argv == NULL) {
        fprintf(stderr, "Cannot read the command line: %lu\n", GetLastError());
        return 1;
    }
    char **utf8_argv = (char **)calloc((size_t)wide_argc + 1, sizeof(char *));
    if (utf8_argv == NULL) {
        fprintf(stderr, "Out of memory while converting the command line\n");
        LocalFree(wide_argv);
        return 1;
    }
    for (int i = 0; i < wide_argc; ++i) {
        utf8_argv[i] = utf16_to_utf8(wide_argv[i]);
        if (utf8_argv[i] == NULL) {
            fprintf(stderr, "Cannot convert argument %d to UTF-8\n", i);
            free_utf8_argv(i, utf8_argv);
            LocalFree(wide_argv);
            return 1;
        }
    }
    LocalFree(wide_argv);
    int rc = run(wide_argc, utf8_argv);
    free_utf8_argv(wide_argc, utf8_argv);
    return rc;
}
#else
int main(int argc, char **argv) {
    return run(argc, argv);
}
#endif
