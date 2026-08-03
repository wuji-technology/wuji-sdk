#define _POSIX_C_SOURCE 199309L

/*
 * Wuji SDK C - Wuji Glove IK calibration for the current SDK user.
 *
 * This example uses the SDK's remembered current local user, connects to a
 * Wuji Glove, and runs IK calibration with structured feedback.
 *
 * For interactive calibration from a shell, use `wuji calib ik`. This example
 * is the reference for applications that integrate the SDK calibration API.
 * Run 3_user_management first to create or switch the current SDK user.
 *
 * The default asynchronous mode requests cooperative cancellation on Ctrl+C.
 * The --blocking convenience API has no external cancellation entry point, so
 * Ctrl+C cannot cooperatively cancel that mode.
 *
 * Run: ./build/4_user_calibration [--sn serial_number]
 *      ./build/4_user_calibration --mode api [--sn serial_number]
 *      ./build/4_user_calibration --blocking [--sn serial_number]
 */
#include <ctype.h>
#include <errno.h>
#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#include "wuji_sdk.h"

typedef enum CalibrationDisplayMode {
    CALIBRATION_DISPLAY_TERMINAL,
    CALIBRATION_DISPLAY_API,
} CalibrationDisplayMode;

typedef struct CalibrationDisplay CalibrationDisplay;

typedef struct ExampleArgs {
    const char *sn;
    const char *device_name;
    CalibrationDisplayMode mode;
    bool blocking;
    bool dry_run;
    bool skip_constraints;
    double timeout_s;
    int log_level;
} ExampleArgs;

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

/* =============================================================================
 * SDK API usage: current user, device connection, calibration, and cancel
 * ============================================================================= */

static volatile sig_atomic_t g_stop = 0;

static void on_sigint(int sig) {
    (void)sig;
    g_stop = 1;
}

static int find_first_glove(const WujiDiscovered *list, size_t n) {
    for (size_t i = 0; i < n; i++) {
        if (list[i].device_id == WUJI_DEVICE_TYPE_WUJI_GLOVE) return (int)i;
    }
    return -1;
}

static const char *handedness_name(WujiHandedness side) {
    switch (side) {
        case WUJI_HANDEDNESS_LEFT: return "left";
        case WUJI_HANDEDNESS_RIGHT: return "right";
        default: return "unknown";
    }
}

static const char *device_type_name(WujiDeviceType type) {
    switch (type) {
        case WUJI_DEVICE_TYPE_WUJI_GLOVE: return "DeviceType.WujiGlove";
        case WUJI_DEVICE_TYPE_WUJI_HAND_2: return "DeviceType.WujiHand2";
        case WUJI_DEVICE_TYPE_WUJI_HAND: return "DeviceType.WujiHand";
        default: return "DeviceType.Unknown";
    }
}

static const char *optional_text(const char *value) {
    return value ? value : "None";
}

static int print_current_user(void) {
    WujiUserInfo user = {0};
    WujiStatus st = wuji_current_user(&user);
    if (st != WUJI_STATUS_OK) {
        fprintf(stderr, "Error: %s\n", wuji_last_error());
        return -1;
    }

    printf("current SDK user:\n");
    printf("  user_id:      '%s'\n", optional_text(user.user_id));
    printf("  display_name: %s\n", optional_text(user.display_name));
    printf("  description:  %s\n", optional_text(user.description));
    printf("  is_default:   %s\n", user.is_default ? "True" : "False");
    printf("\nCalibration results are stored per SDK user and hand side.\n");
    if (user.is_default) {
        printf(
            "\nYou are on the default SDK user, which cannot run calibration.\n"
            "Create and switch to a user profile first, e.g.:\n"
            "    ./build/3_user_management --user-name user1\n"
            "then re-run this example.\n");
    }
    wuji_user_info_free(&user);
    return 0;
}

static void print_usage(FILE *stream, const char *program) {
    fprintf(
        stream,
        "Usage: %s [--sn serial_number] [--device-name name]\n"
        "          [--mode terminal|api] [--blocking] [--dry-run]\n"
        "          [--skip-constraints] [--timeout-s seconds]\n"
        "          [--log-level trace|debug|info|warn|warning|error|off]\n",
        program);
}

static int parse_log_level(const char *value, int *out) {
    if (strcmp(value, "trace") == 0) *out = 5;
    else if (strcmp(value, "debug") == 0) *out = 4;
    else if (strcmp(value, "info") == 0) *out = 3;
    else if (strcmp(value, "warn") == 0 || strcmp(value, "warning") == 0) *out = 2;
    else if (strcmp(value, "error") == 0) *out = 1;
    else if (strcmp(value, "off") == 0) *out = 0;
    else return -1;
    return 0;
}

static int option_value(int argc, char **argv, int *index, const char **out) {
    if (*index + 1 >= argc) return -1;
    *out = argv[++(*index)];
    return 0;
}

static int parse_args(int argc, char **argv, ExampleArgs *args) {
    *args = (ExampleArgs){
        .device_name = "my_glove",
        .mode = CALIBRATION_DISPLAY_TERMINAL,
        .timeout_s = 900.0,
        .log_level = 2,
    };

    for (int i = 1; i < argc; i++) {
        const char *arg = argv[i];
        const char *value = NULL;
        if (strcmp(arg, "-h") == 0 || strcmp(arg, "--help") == 0) {
            print_usage(stdout, argv[0]);
            return 1;
        } else if (strcmp(arg, "--sn") == 0) {
            if (option_value(argc, argv, &i, &args->sn) != 0) return -1;
        } else if (strcmp(arg, "--device-name") == 0) {
            if (option_value(argc, argv, &i, &args->device_name) != 0) return -1;
        } else if (strcmp(arg, "--mode") == 0) {
            if (option_value(argc, argv, &i, &value) != 0) return -1;
            if (strcmp(value, "terminal") == 0) args->mode = CALIBRATION_DISPLAY_TERMINAL;
            else if (strcmp(value, "api") == 0) args->mode = CALIBRATION_DISPLAY_API;
            else return -1;
        } else if (strcmp(arg, "--blocking") == 0) {
            args->blocking = true;
        } else if (strcmp(arg, "--dry-run") == 0) {
            args->dry_run = true;
        } else if (strcmp(arg, "--skip-constraints") == 0) {
            args->skip_constraints = true;
        } else if (strcmp(arg, "--timeout-s") == 0) {
            if (option_value(argc, argv, &i, &value) != 0) return -1;
            errno = 0;
            char *end = NULL;
            args->timeout_s = strtod(value, &end);
            if (errno != 0 || !end || end == value || *end != '\0') return -1;
        } else if (strcmp(arg, "--log-level") == 0) {
            if (option_value(argc, argv, &i, &value) != 0 ||
                parse_log_level(value, &args->log_level) != 0) {
                return -1;
            }
        } else {
            return -1;
        }
    }
    return 0;
}

static int connect_glove(const ExampleArgs *args, struct WujiDevice **out_dev) {
    if (args->sn) {
        printf("Connecting to Wuji Glove SN=%s...\n", args->sn);
        WujiConnectTarget target = {
            .kind = WUJI_CONNECT_TARGET_KIND_SN,
            .value = args->sn,
        };
        WujiStatus st = wuji_connect(&target, args->device_name, NULL, out_dev);
        if (st != WUJI_STATUS_OK) {
            fprintf(stderr, "Error: %s\n", wuji_last_error());
            return -1;
        }
        return 0;
    }

    printf("Scanning for Wuji Glove...\n");
    WujiDiscovered *list = NULL;
    size_t count = 0;
    WujiStatus st = wuji_scan(&list, &count);
    if (st != WUJI_STATUS_OK) {
        fprintf(stderr, "Error: %s\n", wuji_last_error());
        return -1;
    }
    for (size_t i = 0; i < count; i++) {
        printf(
            "  SN=%s, Type=%s, Address=%s\n",
            list[i].serial_number,
            device_type_name(list[i].device_id),
            list[i].address);
    }

    int index = find_first_glove(list, count);
    if (index < 0) {
        fprintf(stderr, "No Wuji Glove found among the scanned devices\n");
        wuji_discovered_free(list, count);
        return -1;
    }

    WujiConnectTarget target = {
        .kind = WUJI_CONNECT_TARGET_KIND_SN,
        .value = list[index].serial_number,
    };
    st = wuji_connect(&target, args->device_name, NULL, out_dev);
    wuji_discovered_free(list, count);
    if (st != WUJI_STATUS_OK) {
        fprintf(stderr, "Error: %s\n", wuji_last_error());
        return -1;
    }
    return 0;
}

static void print_json_string(const char *value) {
    if (!value) {
        printf("null");
        return;
    }
    putchar('"');
    for (const unsigned char *cursor = (const unsigned char *)value; *cursor; cursor++) {
        switch (*cursor) {
            case '"': printf("\\\""); break;
            case '\\': printf("\\\\"); break;
            case '\b': printf("\\b"); break;
            case '\f': printf("\\f"); break;
            case '\n': printf("\\n"); break;
            case '\r': printf("\\r"); break;
            case '\t': printf("\\t"); break;
            default:
                if (*cursor < 0x20) printf("\\u%04x", *cursor);
                else putchar(*cursor);
        }
    }
    putchar('"');
}

static void print_result(const WujiGloveCalibrationResult *result) {
    printf("\nCalibration result:\n{\n");
    printf("  \"calibrated_urdf\": ");
    print_json_string(result->calibrated_urdf);
    printf(",\n  \"frames_per_pose\": ");
    if (result->frames_per_pose_len == 0) {
        printf("{}");
    } else {
        printf("{\n");
        for (size_t i = 0; i < result->frames_per_pose_len; i++) {
            printf("    ");
            print_json_string(result->frames_per_pose[i].pose_name);
            printf(
                ": %u%s\n",
                result->frames_per_pose[i].frames,
                i + 1 == result->frames_per_pose_len ? "" : ",");
        }
        printf("  }");
    }
    printf(",\n  \"handedness\": ");
    print_json_string(handedness_name(result->handedness));
    printf(",\n  \"poses_collected\": %u,\n", result->poses_collected);
    printf("  \"sdk_user\": {\n");
    printf("    \"created_at\": ");
    print_json_string(result->sdk_user.created_at);
    if (result->sdk_user.description) {
        printf(",\n    \"description\": ");
        print_json_string(result->sdk_user.description);
    }
    printf(",\n    \"display_name\": ");
    print_json_string(result->sdk_user.display_name);
    if (result->sdk_user.external_id) {
        printf(",\n    \"external_id\": ");
        print_json_string(result->sdk_user.external_id);
    }
    printf(",\n    \"is_default\": %s", result->sdk_user.is_default ? "true" : "false");
    printf(",\n    \"updated_at\": ");
    print_json_string(result->sdk_user.updated_at);
    printf(",\n    \"user_id\": ");
    print_json_string(result->sdk_user.user_id);
    printf("\n  }\n}\n");
}

static WujiStatus run_async_calibration(
    struct WujiDevice *dev,
    const WujiGloveCalibrationOptions *options,
    CalibrationDisplay *display,
    WujiGloveCalibrationResult *result) {
    WujiGloveCalibrationSession *session = NULL;
    WujiStatus st = wuji_glove_calibration_start(
        dev, options, calibration_display_callback, display, &session);
    if (st != WUJI_STATUS_OK) return st;

    bool done = false;
    bool cancel_requested = false;
    const struct timespec poll_interval = { .tv_sec = 0, .tv_nsec = 50 * 1000 * 1000 };
    while (!done) {
        if (g_stop && !cancel_requested) {
            st = wuji_glove_calibration_cancel(session);
            if (st != WUJI_STATUS_OK) break;
            cancel_requested = true;
        }

        st = wuji_glove_calibration_try_finish(session, &done, result);
        if (st != WUJI_STATUS_OK || done) break;
        nanosleep(&poll_interval, NULL);
    }
    wuji_glove_calibration_session_free(session);
    return st;
}

int main(int argc, char **argv) {
    ExampleArgs args;
    int parse_status = parse_args(argc, argv, &args);
    if (parse_status != 0) {
        if (parse_status < 0) print_usage(stderr, argv[0]);
        return parse_status < 0 ? 2 : 0;
    }

    signal(SIGINT, on_sigint);
    WujiInitOptions init = { .log_level = args.log_level };
    if (wuji_init(&init) != WUJI_STATUS_OK) {
        fprintf(stderr, "Error: %s\n", wuji_last_error());
        return 1;
    }

    if (print_current_user() != 0) {
        wuji_shutdown();
        return 1;
    }
    if (args.dry_run) {
        printf("\nDry run complete: no device connection or calibration was performed.\n");
        wuji_shutdown();
        return 0;
    }

    struct WujiDevice *dev = NULL;
    if (connect_glove(&args, &dev) != 0) {
        wuji_shutdown();
        return 1;
    }

    WujiGloveCalibrationOptions options;
    WujiStatus st = wuji_glove_calibration_options_default(&options);
    if (st != WUJI_STATUS_OK) {
        fprintf(stderr, "Error: %s\n", wuji_last_error());
        wuji_dev_disconnect(dev);
        wuji_dev_release(dev);
        wuji_shutdown();
        return 1;
    }
    options.skip_constraints = args.skip_constraints;
    options.timeout_s = args.timeout_s;

    CalibrationDisplay *display = calibration_display_create(args.mode);
    if (!display) {
        fprintf(stderr, "Error: failed to initialize calibration display\n");
        wuji_dev_disconnect(dev);
        wuji_dev_release(dev);
        wuji_shutdown();
        return 1;
    }
    calibration_display_start(display);
    WujiGloveCalibrationResult result = {0};
    st = args.blocking
             ? wuji_glove_calibrate(
                   dev, &options, calibration_display_callback, display, &result)
             : run_async_calibration(dev, &options, display, &result);

    calibration_display_stop(display);
    calibration_display_destroy(display);
    if (st != WUJI_STATUS_OK) {
        if (st == WUJI_STATUS_ERR_CANCELLED) {
            printf("\nCalibration cancelled.\n");
        } else {
            fprintf(stderr, "Error: %s\n", wuji_last_error());
        }
        wuji_dev_disconnect(dev);
        wuji_dev_release(dev);
        wuji_shutdown();
        return st == WUJI_STATUS_ERR_CANCELLED ? 130 : 1;
    }

    print_result(&result);
    wuji_glove_calibration_result_free(&result);
    wuji_dev_disconnect(dev);
    wuji_dev_release(dev);
    wuji_shutdown();
    return 0;
}

/* =============================================================================
 * Optional terminal presentation: mirrors Python TerminalProgress / ApiProgress
 * ============================================================================= */

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
    printf("Wuji Glove IK Calibration\n");
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
