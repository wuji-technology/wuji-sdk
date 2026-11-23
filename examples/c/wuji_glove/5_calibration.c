/*
 * Purpose:
 *   Calibrate a glove hand model.
 * Prerequisites:
 *   Install the C SDK, connect a Wuji Glove, and select a named SDK user.
 * Usage:
 *   Build with CMake as described in ../README.md, selecting 5_calibration.
 *   ./build/glove/5_calibration
 *   ./build/glove/5_calibration --sn DEVICE_SN
 *   ./build/glove/5_calibration --mode api
 *
 *   Options:
 *     --mode {terminal,api}: display mode (default: terminal).
 *     --skip-constraints: skip pose constraints (default: check constraints).
 *     --timeout-s SECONDS: calibration timeout (default: 900).
 * Expected Behavior:
 *   Show pose instructions, progress, and adjustment hints in a terminal screen.
 *   Use --mode api for scrolling text; redirected output also uses text.
 *   Print the saved hand model after calibration.
 *   Calibration uses the current SDK user and stores one model per hand side.
 *   Exit 0 on completion or Ctrl-C, 1 on execution or cleanup failure,
 *   and 2 on invalid arguments. Errors go to stderr.
 */
#define _POSIX_C_SOURCE 200809L

#include "wuji_sdk.h"

#include <ctype.h>
#include <sys/ioctl.h>
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

#include <time.h>
#include <math.h>
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

typedef enum CalibrationDisplayMode {
    CALIBRATION_DISPLAY_TERMINAL,
    CALIBRATION_DISPLAY_API,
} CalibrationDisplayMode;

typedef struct CalibrationDisplay CalibrationDisplay;

static void calibration_display_init(
    CalibrationDisplay *display,
    CalibrationDisplayMode mode);
static CalibrationDisplay *calibration_display_create(CalibrationDisplayMode mode);
static void calibration_display_start(CalibrationDisplay *display);
static void calibration_display_stop(CalibrationDisplay *display);
static void calibration_display_destroy(CalibrationDisplay *display);
static void calibration_display_callback(
    const WujiGloveCalibrationFeedback *feedback,
    void *user_data);

#define CHECK(call) do { \
    if (stop_requested) goto cleanup; \
    if ((call) != WUJI_STATUS_OK) { \
        report_sdk_error(#call); \
        result = 1; goto cleanup; \
    } \
} while (0)

static WujiStatus run_calibration(WujiDevice *device,
                                 const WujiGloveCalibrationOptions *options,
                                 CalibrationDisplay *display,
                                 WujiGloveCalibrationResult *summary) {
    WujiGloveCalibrationSession *session = NULL;
    WujiStatus status = wuji_glove_calibration_start(
        device, options, calibration_display_callback, display, &session);
    if (status != WUJI_STATUS_OK) return status;

    bool done = false, cancelled = false;
    while (!done) {
        if (stop_requested && !cancelled) {
            status = wuji_glove_calibration_cancel(session);
            if (status != WUJI_STATUS_OK) break;
            cancelled = true;
        }
        status = wuji_glove_calibration_try_finish(session, &done, summary);
        if (status != WUJI_STATUS_OK || done) break;
        sleep_ms(50);
    }
    wuji_glove_calibration_session_free(session);
    return status;
}

int main(int argc, char **argv) {
    const char *sn = NULL;
    CalibrationDisplayMode mode = CALIBRATION_DISPLAY_TERMINAL;
    WujiGloveCalibrationOptions options = {.skip_constraints = false, .timeout_s = 900.0};
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--sn") == 0 && i + 1 < argc && argv[i + 1][0] != '-') {
            sn = argv[++i];
        } else if (strncmp(argv[i], "--sn=", 5) == 0) {
            sn = argv[i] + 5;
        } else if (strcmp(argv[i], "--mode") == 0 && i + 1 < argc) {
            const char *value = argv[++i];
            if (strcmp(value, "terminal") == 0) mode = CALIBRATION_DISPLAY_TERMINAL;
            else if (strcmp(value, "api") == 0) mode = CALIBRATION_DISPLAY_API;
            else { fprintf(stderr, "--mode must be terminal or api\n"); return 2; }
        } else if (strcmp(argv[i], "--skip-constraints") == 0) {
            options.skip_constraints = true;
        } else if (strcmp(argv[i], "--timeout-s") == 0 && i + 1 < argc) {
            const char *text = argv[++i];
            char *end = NULL;
            options.timeout_s = strtod(text, &end);
            if (end == text || *end || !isfinite(options.timeout_s) || options.timeout_s <= 0) {
                fprintf(stderr, "--timeout-s must be positive and finite\n"); return 2;
            }
        } else {
            fprintf(stderr, "Invalid arguments. Usage: %s [--sn DEVICE_SN] [--mode terminal|api] [--timeout-s SECONDS] [--skip-constraints]\n", argv[0]);
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
    WujiUserInfo user = {0};
    CalibrationDisplay *display = NULL;
    WujiGloveCalibrationResult summary = {0};
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

    CHECK(wuji_current_user(&user));
    if (user.is_default) { fprintf(stderr, "Select a named SDK user before calibration\n"); result = 1; goto cleanup; }
    printf("SDK user: %s\n", user.display_name);
    display = calibration_display_create(mode);
    if (display == NULL) { fprintf(stderr, "Error: out of memory\n"); result = 1; goto cleanup; }
    if (stop_requested) goto cleanup;
    calibration_display_start(display);
    WujiStatus status = run_calibration(device, &options, display, &summary);
    calibration_display_stop(display);
    if (status == WUJI_STATUS_ERR_CANCELLED && stop_requested) goto cleanup;
    if (status != WUJI_STATUS_OK) { report_sdk_error("run hand model calibration"); result = 1; goto cleanup; }
    printf("Calibrated %u poses; hand model: %s\n", summary.poses_collected, summary.calibrated_urdf);
cleanup:
    calibration_display_destroy(display);
    wuji_glove_calibration_result_free(&summary);
    wuji_user_info_free(&user);
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

struct CalibrationDisplay {
    bool terminal_enabled;
    bool terminal_active;
    double last_render_at;
    double last_api_print_at;
    int last_api_step;
    WujiGloveCalibrationState last_api_state;
    char last_api_pose[96];
    char last_line[768];
    char last_diagnostics[2048];
    char last_signature[3072];
};

static CalibrationDisplay *calibration_display_create(CalibrationDisplayMode mode) {
    CalibrationDisplay *display = calloc(1, sizeof(*display));
    if (!display) return NULL;
    calibration_display_init(display, mode);
    return display;
}

static void calibration_display_destroy(CalibrationDisplay *display) {
    if (!display) return;
    calibration_display_stop(display);
    free(display);
}

#define MAX_DIAGNOSTIC_LINES 8
#define MAX_METRIC_MESSAGES 64

typedef struct PoseGuide {
    const char *name;
    const char *title;
    const char *guide;
} PoseGuide;

typedef struct MetricMessage {
    char group[96];
    char message[320];
    size_t order;
} MetricMessage;

typedef struct DiagnosticLines {
    size_t count;
    char lines[MAX_DIAGNOSTIC_LINES][384];
    char signature[2048];
} DiagnosticLines;

static const PoseGuide POSE_GUIDES[] = {
    {"flat_open", "Flat open hand", "Straighten all fingers, palm flat, wrist still."},
    {"pinch_index", "Pinch thumb + index", "Touch thumb tip to index fingertip."},
    {"pinch_middle", "Pinch thumb + middle", "Touch thumb tip to middle fingertip."},
    {"pinch_ring", "Pinch thumb + ring", "Touch thumb tip to ring fingertip."},
    {"pinch_pinky", "Pinch thumb + pinky", "Touch thumb tip to pinky fingertip."},
    {"four_finger_bend_90", "Four-finger 90-degree bend", "Bend four fingers about 90 degrees."},
    {"thumb_to_index_dip", "Thumb presses index DIP base", "Press thumb near the index DIP joint base."},
    {"index_to_thumb_emf", "Index points to thumb EMF side", "Point index fingertip toward thumb sensor side."},
};

static double monotonic_seconds(void) {
    struct timespec now;
    if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) return 0.0;
    return (double)now.tv_sec + (double)now.tv_nsec / 1000000000.0;
}

static double clamp_progress(double value) {
    if (!isfinite(value) || value < 0.0) return 0.0;
    return value > 1.0 ? 1.0 : value;
}

static const char *state_name(WujiGloveCalibrationState state) {
    switch (state) {
        case WUJI_GLOVE_CALIBRATION_STATE_WAITING_MOVEMENT: return "waiting_movement";
        case WUJI_GLOVE_CALIBRATION_STATE_WAITING_STABLE: return "waiting_stable";
        case WUJI_GLOVE_CALIBRATION_STATE_COLLECTING: return "collecting";
        case WUJI_GLOVE_CALIBRATION_STATE_DONE: return "done";
        default: return "unknown";
    }
}

static void progress_bar(double value, size_t width, char *out, size_t out_size) {
    if (!out || out_size == 0) return;
    if (width + 3 > out_size) width = out_size > 3 ? out_size - 3 : 0;
    double scaled = clamp_progress(value) * (double)width;
    size_t done = (size_t)scaled;
    double fraction = scaled - (double)done;
    if (fraction > 0.5 || (fraction == 0.5 && done % 2 != 0)) done++;
    if (done > width) done = width;

    size_t offset = 0;
    out[offset++] = '[';
    for (size_t i = 0; i < width; i++) out[offset++] = i < done ? '=' : '.';
    out[offset++] = ']';
    out[offset] = '\0';
}

static void elapsed_text(
    const WujiGloveCalibrationFeedback *feedback,
    char *out,
    size_t out_size) {
    if (!out || out_size == 0) return;
    out[0] = '\0';
    if (feedback->state == WUJI_GLOVE_CALIBRATION_STATE_COLLECTING) {
        double elapsed = feedback->has_collect_elapsed ? feedback->collect_elapsed : 0.0;
        double target = feedback->has_collect_target ? feedback->collect_target : 0.0;
        snprintf(out, out_size, "%.1f/%.1fs", elapsed, target);
    } else if (feedback->state == WUJI_GLOVE_CALIBRATION_STATE_WAITING_STABLE) {
        double elapsed = feedback->has_hold_elapsed ? feedback->hold_elapsed : 0.0;
        double target = feedback->has_hold_target ? feedback->hold_target : 0.0;
        snprintf(out, out_size, "%.1f/%.1fs", elapsed, target);
    }
}

static unsigned step_number(const WujiGloveCalibrationFeedback *feedback) {
    return (feedback->has_step_index ? feedback->step_index : 0) + 1;
}

static unsigned step_total(const WujiGloveCalibrationFeedback *feedback) {
    return feedback->has_step_total ? feedback->step_total : 0;
}

static unsigned overall_done(const WujiGloveCalibrationFeedback *feedback) {
    unsigned total = step_total(feedback);
    unsigned done = feedback->has_step_index ? feedback->step_index : 0;
    if (feedback->state == WUJI_GLOVE_CALIBRATION_STATE_DONE) done++;
    return done > total ? total : done;
}

static const char *pose_name(const WujiGloveCalibrationFeedback *feedback) {
    return feedback->has_step_name && feedback->step_name ? feedback->step_name : "";
}

static void fallback_pose_title(const char *name, char *out, size_t out_size) {
    if (!out || out_size == 0) return;
    bool capitalize = true;
    size_t offset = 0;
    for (const unsigned char *cursor = (const unsigned char *)name;
         *cursor && offset + 1 < out_size;
         cursor++) {
        if (*cursor == '_') {
            out[offset++] = ' ';
            capitalize = true;
        } else {
            out[offset++] = (char)(capitalize ? toupper(*cursor) : tolower(*cursor));
            capitalize = false;
        }
    }
    out[offset] = '\0';
}

static void pose_guide(
    const char *name,
    char *title,
    size_t title_size,
    const char **guide) {
    for (size_t i = 0; i < sizeof(POSE_GUIDES) / sizeof(POSE_GUIDES[0]); i++) {
        if (strcmp(POSE_GUIDES[i].name, name) == 0) {
            snprintf(title, title_size, "%s", POSE_GUIDES[i].title);
            *guide = POSE_GUIDES[i].guide;
            return;
        }
    }
    fallback_pose_title(name, title, title_size);
    *guide = "Move into the pose and hold still.";
}

static int compare_metric_messages(const void *left, const void *right) {
    const MetricMessage *a = left;
    const MetricMessage *b = right;
    int group_order = strcmp(a->group, b->group);
    if (group_order != 0) return group_order;
    return a->order < b->order ? -1 : a->order > b->order ? 1 : 0;
}

static void add_diagnostic_line(DiagnosticLines *diagnostics, const char *line) {
    if (diagnostics->count >= MAX_DIAGNOSTIC_LINES) return;
    snprintf(
        diagnostics->lines[diagnostics->count],
        sizeof(diagnostics->lines[diagnostics->count]),
        "%s",
        line);
    diagnostics->count++;
}

static void build_diagnostics(
    const WujiGloveCalibrationFeedback *feedback,
    DiagnosticLines *diagnostics) {
    memset(diagnostics, 0, sizeof(*diagnostics));

    if (feedback->has_variance_ok && !feedback->variance_ok) {
        add_diagnostic_line(
            diagnostics,
            "motion: hand is moving too much; hold wrist and fingers still");
    }

    MetricMessage messages[MAX_METRIC_MESSAGES];
    size_t message_count = 0;
    for (size_t i = 0;
         i < feedback->metrics_len && message_count < MAX_METRIC_MESSAGES;
         i++) {
        const WujiGloveCalibrationMetric *metric = &feedback->metrics[i];
        if (!metric->has_error) continue;

        const char *unit = !metric->has_unit ? "" : metric->unit ? metric->unit : "None";
        double scale = strcmp(unit, "m") == 0 ? 1000.0 : 1.0;
        double error = metric->error * scale;
        if (error <= 0.0001) continue;
        const char *display_unit = strcmp(unit, "m") == 0 ? "mm" : unit;
        const char *label = !metric->has_label ? "metric" : metric->label ? metric->label : "None";
        const char *finger = metric->has_finger && metric->finger && metric->finger[0]
                                 ? metric->finger
                                 : "global";
        const char *finger_b = metric->has_finger_b && metric->finger_b && metric->finger_b[0]
                                   ? metric->finger_b
                                   : NULL;
        const char *hint = !metric->has_hint
                               ? "adjust pose"
                               : metric->hint ? metric->hint : "None";

        MetricMessage *message = &messages[message_count++];
        message->order = i;
        if (finger_b) {
            snprintf(message->group, sizeof(message->group), "%s-%s", finger, finger_b);
        } else {
            snprintf(message->group, sizeof(message->group), "%s", finger);
        }
        snprintf(
            message->message,
            sizeof(message->message),
            "  %s: off by %.1f%s; %s",
            label,
            error,
            display_unit,
            hint);
    }

    qsort(messages, message_count, sizeof(messages[0]), compare_metric_messages);
    const char *last_group = NULL;
    for (size_t i = 0; i < message_count && diagnostics->count < MAX_DIAGNOSTIC_LINES; i++) {
        if (!last_group || strcmp(last_group, messages[i].group) != 0) {
            char group_line[112];
            snprintf(group_line, sizeof(group_line), "%.95s:", messages[i].group);
            add_diagnostic_line(diagnostics, group_line);
            last_group = messages[i].group;
        }
        add_diagnostic_line(diagnostics, messages[i].message);
    }

    if (feedback->has_constraints_ok && !feedback->constraints_ok && message_count == 0) {
        add_diagnostic_line(
            diagnostics,
            "pose: not inside target; adjust the current pose");
    }

    size_t offset = 0;
    for (size_t i = 0; i < diagnostics->count && offset < sizeof(diagnostics->signature); i++) {
        int written = snprintf(
            diagnostics->signature + offset,
            sizeof(diagnostics->signature) - offset,
            "%s%s",
            i == 0 ? "" : "\n",
            diagnostics->lines[i]);
        if (written < 0 || (size_t)written >= sizeof(diagnostics->signature) - offset) break;
        offset += (size_t)written;
    }
}

static unsigned terminal_width(void) {
    struct winsize size;
    unsigned width = 100;
    if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &size) == 0 && size.ws_col > 0) {
        width = size.ws_col;
    }
    if (width > 100) width = 100;
    return width;
}

static void print_separator(unsigned width) {
    for (unsigned i = 0; i < width; i++) putchar('-');
    putchar('\n');
}

static void render_terminal(
    const WujiGloveCalibrationFeedback *feedback,
    const DiagnosticLines *diagnostics) {
    unsigned width = terminal_width();
    unsigned step = step_number(feedback);
    unsigned total = step_total(feedback);
    const char *pose = pose_name(feedback);
    char title[128];
    const char *guide;
    pose_guide(pose, title, sizeof(title), &guide);

    printf("\033[2J\033[H");
    printf("Wuji Glove Hand Model Calibration\n");
    print_separator(width);

    if (feedback->state == WUJI_GLOVE_CALIBRATION_STATE_DONE) {
        printf("Collected %u/%u: %s\n\n", step, total, title);
        printf(step >= total
                   ? "Final pose collected. Finishing calibration...\n"
                   : "Open your palm before the next pose.\n");
        print_separator(width);
        printf("Keep the wrist relaxed. Move only when the next pose appears.\n");
        fflush(stdout);
        return;
    }

    if (feedback->state == WUJI_GLOVE_CALIBRATION_STATE_WAITING_MOVEMENT) {
        printf("Next %u/%u: %s\n\n", step, total, title);
        printf("Open your palm, then move into the next pose.\n");
        printf("Pose guide: %s\n", guide);
        print_separator(width);
        printf("Move only after your palm is fully open.\n");
        fflush(stdout);
        return;
    }

    char bar[64];
    char elapsed[64];
    progress_bar(feedback->progress, 28, bar, sizeof(bar));
    elapsed_text(feedback, elapsed, sizeof(elapsed));
    printf("Pose %u/%u: %s\n", step, total, title);
    printf("Guide: %s\n", guide);
    printf("State: %s\n", state_name(feedback->state));
    printf("Overall: %u/%u\n", overall_done(feedback), total);
    printf("Current: %s%s%s\n", bar, elapsed[0] ? " " : "", elapsed);
    if (feedback->has_frames_collected) {
        printf("Frames: %u\n", feedback->frames_collected);
    }
    print_separator(width);
    if (diagnostics->count > 0) {
        printf("Adjust:\n");
        for (size_t i = 0; i < diagnostics->count; i++) {
            printf("  %s\n", diagnostics->lines[i]);
        }
    } else {
        printf("OK - keep holding still.\n");
    }
    print_separator(width);
    printf("Press Ctrl+C to cancel.\n");
    fflush(stdout);
}

static void render_api(
    CalibrationDisplay *progress,
    const WujiGloveCalibrationFeedback *feedback,
    const DiagnosticLines *diagnostics,
    double now) {
    unsigned step = step_number(feedback);
    unsigned total = step_total(feedback);
    const char *pose = pose_name(feedback);

    if (feedback->state == WUJI_GLOVE_CALIBRATION_STATE_DONE) {
        printf(
            "[%u/%u] collected %s; %s\n",
            step,
            total,
            pose,
            step >= total ? "finishing calibration" : "open palm before the next pose");
        fflush(stdout);
        return;
    }
    if (feedback->state == WUJI_GLOVE_CALIBRATION_STATE_WAITING_MOVEMENT) {
        printf("[next %u/%u] open palm, then move into %s\n", step, total, pose);
        fflush(stdout);
        return;
    }

    char bar[32];
    char elapsed[64];
    progress_bar(feedback->progress, 18, bar, sizeof(bar));
    elapsed_text(feedback, elapsed, sizeof(elapsed));
    char line[768];
    snprintf(
        line,
        sizeof(line),
        "[%u/%u] %s %s %s overall %u/%u%s%s",
        step,
        total,
        pose,
        state_name(feedback->state),
        bar,
        overall_done(feedback),
        total,
        elapsed[0] ? " " : "",
        elapsed);
    if (feedback->has_frames_collected) {
        size_t length = strlen(line);
        snprintf(
            line + length,
            sizeof(line) - length,
            " frames=%u",
            feedback->frames_collected);
    }

    bool same_state = progress->last_api_step == (int)step &&
                      progress->last_api_state == feedback->state &&
                      strcmp(progress->last_api_pose, pose) == 0;
    bool line_due = strcmp(line, progress->last_line) != 0 &&
                    (!same_state || now - progress->last_api_print_at >= 1.0);
    if (line_due) {
        snprintf(progress->last_line, sizeof(progress->last_line), "%s", line);
        progress->last_api_step = (int)step;
        progress->last_api_state = feedback->state;
        snprintf(progress->last_api_pose, sizeof(progress->last_api_pose), "%s", pose);
        progress->last_api_print_at = now;
        printf("%s\n", line);
    }

    bool diagnostics_changed =
        strcmp(diagnostics->signature, progress->last_diagnostics) != 0;
    if (diagnostics->count > 0 && diagnostics_changed) {
        printf("Adjust:\n");
        for (size_t i = 0; i < diagnostics->count; i++) {
            printf("  %s\n", diagnostics->lines[i]);
        }
    }
    snprintf(
        progress->last_diagnostics,
        sizeof(progress->last_diagnostics),
        "%s",
        diagnostics->signature);
    if (line_due || diagnostics_changed) fflush(stdout);
}

static void calibration_display_init(
    CalibrationDisplay *progress,
    CalibrationDisplayMode mode) {
    if (!progress) return;
    memset(progress, 0, sizeof(*progress));
    progress->terminal_enabled =
        mode == CALIBRATION_DISPLAY_TERMINAL && isatty(STDOUT_FILENO);
    progress->last_api_step = -1;
    progress->last_api_state = WUJI_GLOVE_CALIBRATION_STATE_UNKNOWN;
}

static void calibration_display_start(CalibrationDisplay *progress) {
    if (!progress || !progress->terminal_enabled || progress->terminal_active) return;
    printf("\033[?1049h\033[?25l\033[2J\033[H");
    fflush(stdout);
    progress->terminal_active = true;
}

static void calibration_display_stop(CalibrationDisplay *progress) {
    if (!progress || !progress->terminal_active) return;
    printf("\033[0m\033[?25h\033[?1049l");
    fflush(stdout);
    progress->terminal_active = false;
}

static void calibration_display_callback(
    const WujiGloveCalibrationFeedback *feedback,
    void *user_data) {
    CalibrationDisplay *progress = user_data;
    if (!feedback || !progress) return;

    DiagnosticLines diagnostics;
    build_diagnostics(feedback, &diagnostics);
    double now = monotonic_seconds();

    if (!progress->terminal_enabled) {
        render_api(progress, feedback, &diagnostics, now);
        return;
    }

    char bar[40];
    char elapsed[64];
    progress_bar(feedback->progress, 18, bar, sizeof(bar));
    elapsed_text(feedback, elapsed, sizeof(elapsed));
    char signature[3072];
    snprintf(
        signature,
        sizeof(signature),
        "%s\n%s\n%u/%u\n%s\n%s\n%s",
        pose_name(feedback),
        state_name(feedback->state),
        overall_done(feedback),
        step_total(feedback),
        bar,
        elapsed,
        diagnostics.signature);
    if (strcmp(signature, progress->last_signature) == 0) return;
    bool urgent = feedback->state == WUJI_GLOVE_CALIBRATION_STATE_DONE ||
                  feedback->state == WUJI_GLOVE_CALIBRATION_STATE_WAITING_MOVEMENT;
    if (!urgent && now - progress->last_render_at < 0.1) return;

    snprintf(progress->last_signature, sizeof(progress->last_signature), "%s", signature);
    progress->last_render_at = now;
    render_terminal(feedback, &diagnostics);
}
