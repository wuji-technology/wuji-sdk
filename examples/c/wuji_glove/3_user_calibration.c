#define _POSIX_C_SOURCE 199309L

/*
 * Wuji SDK C - SDK user management and WujiGlove IK calibration.
 *
 * This example creates or reuses a named SDK user, switches to that user,
 * connects to a Wuji Glove, and runs IK calibration with structured feedback.
 *
 * Run: ./build/3_user_calibration [--mode terminal|api] <user_name> [serial_number]
 */
#include <ctype.h>
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
 * SDK API usage: user selection, device connection, async calibration, and cancel
 * ============================================================================= */

static volatile sig_atomic_t g_stop = 0;

static void on_sigint(int sig) {
    (void)sig;
    g_stop = 1;
}
static int find_target(const WujiDiscovered *list, size_t n, const char *target_sn) {
    if (target_sn && target_sn[0]) {
        for (size_t i = 0; i < n; i++) {
            if (strcmp(list[i].serial_number, target_sn) == 0) return (int)i;
        }
        return -1;
    }
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

static int switch_or_create_user(const char *display_name) {
    WujiUserInfo *users = NULL;
    size_t user_count = 0;
    WujiStatus st = wuji_list_users(&users, &user_count);
    if (st != WUJI_STATUS_OK) {
        fprintf(stderr, "wuji_list_users failed: %s\n", wuji_last_error());
        return -1;
    }

    const char *user_id = NULL;
    for (size_t i = 0; i < user_count; i++) {
        if (users[i].display_name && strcmp(users[i].display_name, display_name) == 0) {
            user_id = users[i].user_id;
            break;
        }
    }

    if (user_id) {
        WujiUserInfo current = {0};
        st = wuji_switch_user(user_id, &current);
        if (st != WUJI_STATUS_OK) {
            fprintf(stderr, "wuji_switch_user failed: %s\n", wuji_last_error());
            wuji_user_info_array_free(users, user_count);
            return -1;
        }
        printf("Using SDK user: %s (%s)\n", current.display_name, current.user_id);
        wuji_user_info_free(&current);
        wuji_user_info_array_free(users, user_count);
        return 0;
    }

    wuji_user_info_array_free(users, user_count);

    WujiUserInfo created = {0};
    st = wuji_create_user(display_name, "C SDK calibration example", NULL, &created);
    if (st != WUJI_STATUS_OK) {
        fprintf(stderr, "wuji_create_user failed: %s\n", wuji_last_error());
        return -1;
    }

    WujiUserInfo current = {0};
    st = wuji_switch_user(created.user_id, &current);
    if (st != WUJI_STATUS_OK) {
        fprintf(stderr, "wuji_switch_user failed: %s\n", wuji_last_error());
        wuji_user_info_free(&created);
        return -1;
    }
    printf("Created SDK user: %s (%s)\n", current.display_name, current.user_id);
    wuji_user_info_free(&created);
    wuji_user_info_free(&current);
    return 0;
}

static void print_usage(const char *program) {
    fprintf(
        stderr,
        "Usage: %s [--mode terminal|api] <user_name> [serial_number]\n",
        program);
}

int main(int argc, char **argv) {
    signal(SIGINT, on_sigint);

    CalibrationDisplayMode display_mode = CALIBRATION_DISPLAY_TERMINAL;
    int arg_index = 1;
    if (argc >= 2 && strcmp(argv[1], "--mode") == 0) {
        if (argc < 4) {
            print_usage(argv[0]);
            return 2;
        }
        if (strcmp(argv[2], "terminal") == 0) {
            display_mode = CALIBRATION_DISPLAY_TERMINAL;
        } else if (strcmp(argv[2], "api") == 0) {
            display_mode = CALIBRATION_DISPLAY_API;
        } else {
            print_usage(argv[0]);
            return 2;
        }
        arg_index += 2;
    }
    int remaining = argc - arg_index;
    if (remaining < 1 || remaining > 2) {
        print_usage(argv[0]);
        return 2;
    }

    const char *user_name = argv[arg_index];
    const char *target_sn = remaining == 2 ? argv[arg_index + 1] : NULL;

    WujiInitOptions init = { .log_level = 2 };
    if (wuji_init(&init) != WUJI_STATUS_OK) {
        fprintf(stderr, "wuji_init failed: %s\n", wuji_last_error());
        return 1;
    }

    if (switch_or_create_user(user_name) != 0) {
        wuji_shutdown();
        return 1;
    }

    WujiDiscovered *list = NULL;
    size_t n = 0;
    if (wuji_scan(&list, &n) != WUJI_STATUS_OK) {
        fprintf(stderr, "wuji_scan failed: %s\n", wuji_last_error());
        wuji_shutdown();
        return 1;
    }

    int idx = find_target(list, n, target_sn);
    if (idx < 0) {
        fprintf(stderr, target_sn ? "Device not found: %s\n" : "No Wuji Glove found\n",
                target_sn ? target_sn : "");
        wuji_discovered_free(list, n);
        wuji_shutdown();
        return 1;
    }

    WujiConnectTarget target = {
        .kind = WUJI_CONNECT_TARGET_KIND_SN,
        .value = list[idx].serial_number,
    };

    struct WujiDevice *dev = NULL;
    WujiStatus st = wuji_connect(&target, "glove_calibration", NULL, &dev);
    wuji_discovered_free(list, n);
    if (st != WUJI_STATUS_OK) {
        fprintf(stderr, "wuji_connect failed: %s\n", wuji_last_error());
        wuji_shutdown();
        return 1;
    }

    WujiGloveCalibrationOptions options;
    st = wuji_glove_calibration_options_default(&options);
    if (st != WUJI_STATUS_OK) {
        fprintf(stderr, "options default failed: %s\n", wuji_last_error());
        wuji_dev_disconnect(dev);
        wuji_dev_release(dev);
        wuji_shutdown();
        return 1;
    }

    CalibrationDisplay *display = calibration_display_create(display_mode);
    if (!display) {
        fprintf(stderr, "failed to initialize calibration display\n");
        wuji_dev_disconnect(dev);
        wuji_dev_release(dev);
        wuji_shutdown();
        return 1;
    }
    calibration_display_start(display);
    WujiGloveCalibrationSession *session = NULL;
    WujiGloveCalibrationResult result = {0};
    st = wuji_glove_calibration_start(
        dev, &options, calibration_display_callback, display, &session);
    if (st != WUJI_STATUS_OK) {
        calibration_display_destroy(display);
        fprintf(stderr, "wuji_glove_calibration_start failed: %s\n", wuji_last_error());
        wuji_dev_disconnect(dev);
        wuji_dev_release(dev);
        wuji_shutdown();
        return 1;
    }

    bool done = false;
    bool cancel_requested = false;
    const struct timespec poll_interval = { .tv_sec = 0, .tv_nsec = 50 * 1000 * 1000 };
    while (!done) {
        if (g_stop && !cancel_requested) {
            st = wuji_glove_calibration_cancel(session);
            if (st != WUJI_STATUS_OK) break;
            cancel_requested = true;
        }

        st = wuji_glove_calibration_try_finish(session, &done, &result);
        if (st != WUJI_STATUS_OK || done) break;
        nanosleep(&poll_interval, NULL);
    }

    calibration_display_stop(display);
    calibration_display_destroy(display);
    if (st != WUJI_STATUS_OK) {
        fprintf(stderr, "calibration failed: %s\n", wuji_last_error());
        wuji_glove_calibration_session_free(session);
        wuji_dev_disconnect(dev);
        wuji_dev_release(dev);
        wuji_shutdown();
        return st == WUJI_STATUS_ERR_CANCELLED ? 130 : 1;
    }
    wuji_glove_calibration_session_free(session);

    printf("Calibration complete: side=%s poses=%u urdf=%s user=%s\n",
           handedness_name(result.handedness),
           result.poses_collected,
           result.calibrated_urdf ? result.calibrated_urdf : "",
           result.sdk_user.display_name ? result.sdk_user.display_name : "");
    for (size_t i = 0; i < result.frames_per_pose_len; i++) {
        printf("  %s: %u frames\n",
               result.frames_per_pose[i].pose_name,
               result.frames_per_pose[i].frames);
    }

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
    size_t done = (size_t)(clamp_progress(value) * (double)width + 0.5);
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
    if (feedback->state == WUJI_GLOVE_CALIBRATION_STATE_COLLECTING &&
        feedback->has_collect_elapsed && feedback->has_collect_target) {
        snprintf(out, out_size, "%.1f/%.1fs", feedback->collect_elapsed, feedback->collect_target);
    } else if (feedback->state == WUJI_GLOVE_CALIBRATION_STATE_WAITING_STABLE &&
               feedback->has_hold_elapsed && feedback->has_hold_target) {
        snprintf(out, out_size, "%.1f/%.1fs", feedback->hold_elapsed, feedback->hold_target);
    }
}

static unsigned step_number(const WujiGloveCalibrationFeedback *feedback) {
    return feedback->has_step_index ? feedback->step_index + 1 : 0;
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
            out[offset++] = (char)(capitalize ? toupper(*cursor) : *cursor);
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

        const char *unit = metric->has_unit && metric->unit ? metric->unit : "";
        double scale = strcmp(unit, "m") == 0 ? 1000.0 : 1.0;
        double error = metric->error * scale;
        if (error <= 0.0001) continue;
        const char *display_unit = strcmp(unit, "m") == 0 ? "mm" : unit;
        const char *label = metric->has_label && metric->label ? metric->label : "metric";
        const char *finger = metric->has_finger && metric->finger ? metric->finger : "global";
        const char *finger_b = metric->has_finger_b && metric->finger_b ? metric->finger_b : NULL;
        const char *hint = metric->has_hint && metric->hint ? metric->hint : "adjust pose";

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
    if (width < 40) width = 40;
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
