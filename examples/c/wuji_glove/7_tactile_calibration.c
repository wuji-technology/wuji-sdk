/*
 * Purpose:
 *   Calibrate the glove tactile contact model.
 * Prerequisites:
 *   Install a C SDK build with model training support on Linux or macOS.
 *   Connect a Wuji Glove and prepare to perform the prompted motions.
 * Usage:
 *   Build with CMake as described in ../README.md, selecting 7_tactile_calibration.
 *   ./build/glove/7_tactile_calibration
 *   ./build/glove/7_tactile_calibration --sn DEVICE_SN
 *
 *   Options:
 *     --seconds-per-pose SECONDS: recording time per motion (default: 10).
 *     --epochs N: training epochs (default: 60).
 *     --no-install: train without installing the model (default: install).
 *     --sensitivity VALUE: optional sensitivity override.
 *     --timeout-s SECONDS: calibration timeout (default: 1800).
 * Expected Behavior:
 *   Confirm each motion, then keep it or retry. Train and install the model
 *   unless --no-install is set, and print the calibration summary.
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

#include <math.h>
#include <ctype.h>
#include <stdatomic.h>
#include <sys/select.h>
#include <unistd.h>
static const char *motion_text(const char *name) {
    if (!name) return "Follow the motion guide.";
    if (strcmp(name, "four_finger_L_thumb_in") == 0) {
        return "Curl four fingers into an L, thumb tucked in; vary the pressure.";
    }
    if (strcmp(name, "claw_curl") == 0) return "Curl all fingers into a claw and flex repeatedly.";
    if (strcmp(name, "abduction") == 0) return "Spread and close the fingers repeatedly.";
    if (strcmp(name, "finger_to_palm") == 0) return "Press each fingertip to the palm in turn.";
    return "Follow the motion guide.";
}

static atomic_int aborted = 0;
static int read_line(char *buf, size_t len) {
    size_t used = 0;
    buf[0] = '\0';
    while (!stop_requested && used + 1 < len) {
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);
        struct timeval timeout = {0, 100000};
        int ready = select(STDIN_FILENO + 1, &fds, NULL, NULL, &timeout);
        if (ready < 0 && errno != EINTR) return 0;
        if (ready <= 0) continue;
        if (stop_requested) break;

        char c;
        ssize_t got = read(STDIN_FILENO, &c, 1);
        if (got < 0) {
            if (errno == EINTR) continue;
            return 0;
        }
        if (got == 0) break;
        buf[used++] = c;
        buf[used] = '\0';
        if (c == '\n') break;
    }
    return !stop_requested && used > 0;
}

static int is_abort_answer(const char *line) {
    char c = (char)tolower((unsigned char)line[0]);
    return c == 'a' || c == 'q' || c == 's';
}

static WujiTactileCallbackAction on_feedback(const WujiTactileCalibrationFeedback *ev,
                                             void *userdata) {
    (void)userdata;
    if (stop_requested || !ev) return WUJI_TACTILE_CALLBACK_ACTION_ABORT;

    switch (ev->state_kind) {
    case WUJI_TACTILE_CALIBRATION_STATE_COLLECT:
        if (ev->collect_target > 0.0f) {
            printf("[collect] %u/%u %s %.1f/%.1fs frames=%u\n",
                   ev->step_index + 1, ev->step_total,
                   ev->step_name ? ev->step_name : "pose",
                   ev->collect_elapsed, ev->collect_target, ev->frames_collected);
        }
        break;
    case WUJI_TACTILE_CALIBRATION_STATE_TRAIN:
        if (ev->epoch > 0) {
            printf("[train] epoch %u/%u best_val=%.5f\n",
                   ev->epoch, ev->epoch_total, ev->best_val);
        }
        break;
    default:
        if (ev->state) printf("[%s]\n", ev->state);
        break;
    }
    fflush(stdout);
    return WUJI_TACTILE_CALLBACK_ACTION_CONTINUE;
}

static WujiTactilePromptResponse on_pose_prompt(const WujiTactilePromptRequest *req,
                                                void *userdata) {
    (void)userdata;
    if (stop_requested || !req) return WUJI_TACTILE_PROMPT_RESPONSE_ABORT;

    char line[32];
    const char *pose = req->step_name ? req->step_name : "pose";
    if (req->kind == WUJI_TACTILE_PROMPT_KIND_POSE_READY) {
        printf("\nMotion %u/%u: %s\n", req->step_index + 1, req->step_total, pose);
        printf("  %s\n", motion_text(pose));
        printf("  Records for %.0f seconds after you start.\n", req->seconds_per_pose);
        printf("Press Enter when ready (or q to abort): ");
        fflush(stdout);
        if (!read_line(line, sizeof line)) return WUJI_TACTILE_PROMPT_RESPONSE_ABORT;
        if (is_abort_answer(line)) { atomic_store(&aborted, 1); return WUJI_TACTILE_PROMPT_RESPONSE_ABORT; }
        return WUJI_TACTILE_PROMPT_RESPONSE_PROCEED;
    }

    if (req->kind == WUJI_TACTILE_PROMPT_KIND_POSE_REVIEW) {
        if (req->warnings_len > 0) {
            printf("\nQuality warnings for %s:\n", pose);
            for (size_t i = 0; i < req->warnings_len; i++) {
                printf("  - %s\n", req->warnings[i] ? req->warnings[i] : "<warning>");
            }
        }
        for (;;) {
            printf("Keep this motion? [Enter/r/q] ");
            fflush(stdout);
            if (!read_line(line, sizeof line)) return WUJI_TACTILE_PROMPT_RESPONSE_ABORT;
            char c = (char)tolower((unsigned char)line[0]);
            if (c == '\n' || c == '\0' || c == 'y' || c == 'k') return WUJI_TACTILE_PROMPT_RESPONSE_PROCEED;
            if (c == 'r') return WUJI_TACTILE_PROMPT_RESPONSE_RETRY;
            if (c == 'q' || c == 'a' || c == 's') { atomic_store(&aborted, 1); return WUJI_TACTILE_PROMPT_RESPONSE_ABORT; }
        }
    }

    return WUJI_TACTILE_PROMPT_RESPONSE_PROCEED;
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
    WujiTactileCalibrationOptions options = {
        .seconds_per_pose = 10.0, .epochs = 60, .install = true, .timeout_s = 1800.0,
    };
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--sn") == 0 && i + 1 < argc && argv[i + 1][0] != '-') {
            sn = argv[++i];
        } else if (strncmp(argv[i], "--sn=", 5) == 0) {
            sn = argv[i] + 5;
        } else if (strcmp(argv[i], "--no-install") == 0) {
            options.install = false;
        } else if ((strcmp(argv[i], "--seconds-per-pose") == 0 || strcmp(argv[i], "--epochs") == 0 ||
                    strcmp(argv[i], "--timeout-s") == 0 || strcmp(argv[i], "--sensitivity") == 0) && i + 1 < argc) {
            const char *option = argv[i];
            const char *text = argv[++i];
            char *end = NULL;
            errno = 0;
            double value = strcmp(option, "--epochs") == 0
                ? (double)strtoll(text, &end, 10) : strtod(text, &end);
            if (errno || end == text || *end || !isfinite(value) || value <= 0) {
                fprintf(stderr, "Invalid numeric argument: %s\n", text); return 2;
            }
            if (strcmp(option, "--epochs") == 0) {
                if (value > UINT32_MAX) { fprintf(stderr, "Invalid epoch count\n"); return 2; }
                options.epochs = (uint32_t)value;
            } else if (strcmp(option, "--seconds-per-pose") == 0) options.seconds_per_pose = value;
            else if (strcmp(option, "--timeout-s") == 0) options.timeout_s = value;
            else { options.sensitivity = value; options.has_sensitivity = true; }
        } else {
            fprintf(stderr, "Invalid arguments. Usage: %s [--sn DEVICE_SN] [--seconds-per-pose SECONDS] [--epochs N] [--no-install] [--sensitivity VALUE] [--timeout-s SECONDS]\n", argv[0]);
            return 2;
        }
        if (sn != NULL && sn[0] == '\0') {
            fprintf(stderr, "--sn must not be empty\n");
            return 2;
        }
    }


    if (signal(SIGINT, on_sigint) == SIG_ERR) {
        fprintf(stderr, "install Ctrl-C handler failed: %s\n", strerror(errno));
        return 1;
    }

    int result = 0;
    WujiTactileCalibrationSummary summary = {0};
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

    WujiTactileCalibrationCallbacks callbacks = {
        .on_feedback = on_feedback, .on_pose_prompt = on_pose_prompt,
    };
    if (stop_requested) goto cleanup;
    WujiStatus status = wuji_glove_calibrate_tactile_blocking(device, &options, &callbacks, &summary);
    if (status == WUJI_STATUS_ERR_CANCELLED && (stop_requested || atomic_load(&aborted))) goto cleanup;
    if (status != WUJI_STATUS_OK) { report_sdk_error("wuji_glove_calibrate_tactile_blocking"); result = 1; goto cleanup; }
    printf("Calibrated %zu poses, %u epochs; installed: %s\n", summary.poses_collected,
           summary.epochs, summary.installed ? "true" : "false");
    printf("Alive taxels: %zu; best validation loss: %.5f; model: %s\n", summary.alive_taxels,
           summary.best_val_loss, summary.model_dir ? summary.model_dir : "not installed");
cleanup:
    wuji_tactile_calibration_summary_free(&summary);
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
