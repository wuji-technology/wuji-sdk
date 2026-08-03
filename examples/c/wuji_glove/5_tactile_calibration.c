/*
 * Wuji SDK C - Wuji Glove tactile calibration.
 *
 * Stable v1 example: run the blocking guided calibration flow and print the
 * summary returned by the SDK.
 *
 * Build: cmake -S . -B build \
 *          -DWUJI_SDK_INCLUDE_DIR=/path/to/sdk/include \
 *          -DWUJI_SDK_LIB=/path/to/sdk/lib/libwuji_sdk_c.so
 *        cmake --build build
 * Run:   ./build/5_tactile_calibration <glove_serial> [seconds_per_pose] [epochs]
 */
#define _POSIX_C_SOURCE 199309L
#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "wuji_sdk.h"

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

static void sleep_ms(long ms) {
    struct timespec ts;
    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000000L;
    nanosleep(&ts, NULL);
}

static void countdown_to_recording(void) {
    for (int count = 3; count >= 1; count--) {
        printf("  starting in %d...\r", count);
        fflush(stdout);
        sleep_ms(1000);
    }
    printf("  recording now!        \n");
}

static int read_line(char *buf, size_t len) {
    return fgets(buf, (int)len, stdin) != NULL;
}

static int is_abort_answer(const char *line) {
    char c = (char)tolower((unsigned char)line[0]);
    return c == 'a' || c == 'q' || c == 's';
}

static void print_error(const char *what) {
    const char *err = wuji_last_error();
    fprintf(stderr, "%s: %s\n", what, err ? err : "<no detail>");
}

static WujiTactileCallbackAction on_feedback(const WujiTactileCalibrationFeedback *ev,
                                             void *userdata) {
    (void)userdata;
    if (!ev) return WUJI_TACTILE_CALLBACK_ACTION_ABORT;

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
    if (!req) return WUJI_TACTILE_PROMPT_RESPONSE_ABORT;

    char line[32];
    const char *pose = req->step_name ? req->step_name : "pose";
    if (req->kind == WUJI_TACTILE_PROMPT_KIND_POSE_READY) {
        printf("\nMotion %u/%u: %s\n", req->step_index + 1, req->step_total, pose);
        printf("  %s\n", motion_text(pose));
        printf("  Records for %.0f seconds after you start.\n", req->seconds_per_pose);
        printf("Press Enter when ready (or q to abort): ");
        fflush(stdout);
        if (!read_line(line, sizeof line)) return WUJI_TACTILE_PROMPT_RESPONSE_ABORT;
        if (is_abort_answer(line)) return WUJI_TACTILE_PROMPT_RESPONSE_ABORT;
        countdown_to_recording();
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
            if (c == 'q' || c == 'a' || c == 's') return WUJI_TACTILE_PROMPT_RESPONSE_ABORT;
        }
    }

    return WUJI_TACTILE_PROMPT_RESPONSE_PROCEED;
}

static void print_intro(const char *sn, float seconds_per_pose, unsigned long epochs) {
    printf("Wuji Glove tactile calibration\n");
    printf("  serial number:    %s\n", sn);
    printf("  seconds per pose: %.1f\n", seconds_per_pose);
    printf("  epochs:           %lu\n", epochs);
    printf("\nYou will confirm before each motion and then keep, redo, or abort after it.\n");
}

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <glove_serial> [seconds_per_pose] [epochs]\n", argv[0]);
        return 2;
    }

    const char *sn = argv[1];
    float seconds_per_pose = (argc >= 3) ? (float)strtod(argv[2], NULL) : 10.0f;
    unsigned long epochs = (argc >= 4) ? strtoul(argv[3], NULL, 10) : 60;
    if (seconds_per_pose <= 0.0f || epochs == 0) {
        fprintf(stderr, "seconds_per_pose must be > 0 and epochs must be >= 1\n");
        return 2;
    }

    WujiInitOptions init = { .log_level = 0 };
    if (wuji_init(&init) != WUJI_STATUS_OK) {
        print_error("wuji_init");
        return 1;
    }

    print_intro(sn, seconds_per_pose, epochs);

    WujiConnectTarget target = { .kind = WUJI_CONNECT_TARGET_KIND_SN, .value = sn };
    struct WujiDevice *dev = NULL;
    if (wuji_connect(&target, "glove", NULL, &dev) != WUJI_STATUS_OK) {
        print_error("wuji_connect");
        wuji_shutdown();
        return 1;
    }

    WujiTactileCalibrationOptions options = {
        .seconds_per_pose = seconds_per_pose,
        .epochs = (uint32_t)epochs,
        .install = true,
        .has_sensitivity = false,
        .sensitivity = 0.0,
        .timeout_s = 1800.0,
    };
    WujiTactileCalibrationCallbacks callbacks = {
        .on_feedback = on_feedback,
        .feedback_userdata = NULL,
        .on_pose_prompt = on_pose_prompt,
        .prompt_userdata = NULL,
    };
    WujiTactileCalibrationSummary summary = {0};

    WujiStatus st = wuji_glove_calibrate_tactile_blocking(dev, &options, &callbacks, &summary);
    if (st != WUJI_STATUS_OK) {
        print_error(st == WUJI_STATUS_ERR_UNSUPPORTED
                        ? "tactile calibration unsupported by this C SDK build"
                        : "wuji_glove_calibrate_tactile_blocking");
        wuji_dev_disconnect(dev);
        wuji_dev_release(dev);
        wuji_shutdown();
        return 1;
    }

    printf("\nCalibration complete\n");
    printf("  poses: %zu\n", summary.poses_collected);
    printf("  epochs: %u\n", summary.epochs);
    printf("  best validation loss: %.5f\n", summary.best_val_loss);
    printf("  alive taxels: %zu\n", summary.alive_taxels);
    printf("  dead pct: %.2f\n", summary.dead_pct);
    printf("  installed: %s\n", summary.installed ? "true" : "false");
    printf("  model dir: %s\n", summary.model_dir ? summary.model_dir : "<not installed>");
    if (summary.has_verified_alive_taxels) {
        printf("  verified alive taxels: %zu\n", summary.verified_alive_taxels);
    }

    wuji_tactile_calibration_summary_free(&summary);
    wuji_dev_disconnect(dev);
    wuji_dev_release(dev);
    wuji_shutdown();
    return 0;
}
