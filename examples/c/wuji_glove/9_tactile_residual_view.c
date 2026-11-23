/*
 * Purpose:
 *   View live tactile residual data.
 * Prerequisites:
 *   Install the C SDK on Linux or macOS and connect a calibrated Wuji Glove.
 * Usage:
 *   Build with CMake as described in ../README.md, selecting 9_tactile_residual_view.
 *   ./build/glove/9_tactile_residual_view
 *   ./build/glove/9_tactile_residual_view --sn DEVICE_SN
 *
 *   Options:
 *     --seconds SECONDS: run duration (default: 0, until q or Ctrl-C).
 *     --threshold VALUE: initial display threshold (default: 0.5).
 * Expected Behavior:
 *   Render a 24x31 terminal grid. Use + / - to adjust threshold, q to quit,
 *   or --seconds to set a duration (0 waits for q or Ctrl-C).
 *   Exit 0 on completion or Ctrl-C, 1 on execution or cleanup failure,
 *   and 2 on invalid arguments. Errors go to stderr.
 */
#define _POSIX_C_SOURCE 200809L

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

#include <stdatomic.h>
#include <math.h>
#include <sys/select.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>
static atomic_int stream_failed = 0;
static atomic_int closing = 0;

static int frame_ok(WujiFrameKind kind, const void *frame) {
    if (kind == WUJI_FRAME_KIND_END && atomic_load(&closing)) return 0;
    if (kind == WUJI_FRAME_KIND_LAG) {
        const char *error = wuji_last_error();
        fprintf(stderr, "Warning: %s\n", error ? error : "subscription frames were dropped");
        return 0;
    }
    if (kind == WUJI_FRAME_KIND_OK && frame != NULL) return 1;
    fprintf(stderr, "subscription failed (%d): %s\n", kind,
            wuji_last_error() ? wuji_last_error() : "stream ended or data was lost");
    atomic_store(&stream_failed, 1);
    return 0;
}

#define TACTILE_ROWS 24
#define TACTILE_COLS 31
#define MASKED_VALUE (-1.0f)  // invalid/masked sentinel
#define MASKED_EPS (1e-6f)

// The residual magnitude ramp for taxels below the threshold (low -> high).
static const char RAMP[] = ".:-=+*";
#define RAMP_LEN 6

static _Atomic double g_threshold = 0.5;

static long now_ms(void) {
    struct timespec ts;
    if (clock_gettime(CLOCK_MONOTONIC, &ts) != 0) return 0;
    return ts.tv_sec * 1000L + ts.tv_nsec / 1000000L;
}

static void sleep_ms(long ms) {
    struct timespec ts;
    ts.tv_sec  = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000000L;
    while (nanosleep(&ts, &ts) != 0 && errno == EINTR) {
    }
}

static double clamp_threshold(double value) {
    return value < 0.0 ? 0.0 : value;
}

static char cell_for(float value, double threshold) {
    if (fabsf(value - MASKED_VALUE) < MASKED_EPS) return ' ';
    if (threshold > 0.0f && value >= threshold) return '#';
    if (value <= 0.0f || threshold <= 0.0f) return '.';
    int idx = (int)(value / threshold * RAMP_LEN);
    if (idx >= RAMP_LEN) idx = RAMP_LEN - 1;
    return RAMP[idx];
}

static int read_key_nonblocking(void) {
    fd_set fds;
    struct timeval tv = {0, 0};
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    if (select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) <= 0) return 0;

    unsigned char ch = 0;
    ssize_t n = read(STDIN_FILENO, &ch, 1);
    return n == 1 ? (int)ch : 0;
}

static int enter_cbreak_if_tty(struct termios *old_settings) {
    if (!isatty(STDIN_FILENO)) return 0;
    if (tcgetattr(STDIN_FILENO, old_settings) != 0) return -1;

    struct termios raw = *old_settings;
    raw.c_lflag &= (tcflag_t) ~(ICANON | ECHO);
    raw.c_cc[VMIN]  = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) return -1;
    return 1;
}

static void render_frame(const WujiTactileResidual *frame) {
    const size_t expected = TACTILE_ROWS * TACTILE_COLS;
    const size_t limit = frame->data_len < expected ? frame->data_len : expected;
    const double threshold = atomic_load(&g_threshold);
    size_t above = 0;
    float peak = -1e30f;
    size_t peak_idx = 0;

    for (size_t i = 0; i < limit; i++) {
        float v = frame->data[i];
        if (fabsf(v - MASKED_VALUE) < MASKED_EPS) continue;
        if (threshold > 0.0f && v >= threshold) above++;
        if (v > peak) {
            peak = v;
            peak_idx = i;
        }
    }
    if (peak <= -1e30f) peak = 0.0f;

    printf("\x1b[H\x1b[2J");
    printf("above=%-3zu threshold=%.1f peak=%+.3f row=%zu col=%zu seq=%u len=%zu  (+/- threshold, q quit)\n",
           above,
           (double)threshold,
           (double)peak,
           peak_idx / TACTILE_COLS,
           peak_idx % TACTILE_COLS,
           frame->header.seq,
           frame->data_len);

    for (size_t row = 0; row < TACTILE_ROWS; row++) {
        const size_t base = row * TACTILE_COLS;
        for (size_t col = 0; col < TACTILE_COLS; col++) {
            const size_t idx = base + col;
            const float v = idx < limit ? frame->data[idx] : -1.0f;
            putchar(cell_for(v, threshold));
        }
        putchar('\n');
    }
    fflush(stdout);
}

static void on_residual(WujiFrameKind kind, const WujiTactileResidual *frame, void *userdata) {
    (void)userdata;
    static long last_render_ms = 0;

    if (!frame_ok(kind, frame)) return;

    long t = now_ms();
    if (t - last_render_ms < 100) return;
    last_render_ms = t;
    render_frame(frame);
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
    double seconds = 0, setting = 0.5;
    for (int i = 1; i < argc; ++i) {
        if (strcmp(argv[i], "--sn") == 0 && i + 1 < argc && argv[i + 1][0] != '-') {
            sn = argv[++i];
        } else if (strncmp(argv[i], "--sn=", 5) == 0) {
            sn = argv[i] + 5;
        } else if ((strcmp(argv[i], "--seconds") == 0 || strcmp(argv[i], "--threshold") == 0) && i + 1 < argc) {
            int duration = strcmp(argv[i], "--seconds") == 0;
            const char *text = argv[++i];
            char *end = NULL;
            double value = strtod(text, &end);
            if (end == text || *end || !isfinite(value) || (duration && value < 0)) {
                fprintf(stderr, "Invalid numeric argument: %s\n", text); return 2;
            }
            if (duration) seconds = value; else setting = value;
        } else {
            fprintf(stderr, "Invalid arguments. Usage: %s [--sn DEVICE_SN] [--seconds SECONDS] [--threshold VALUE]\n", argv[0]);
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
    WujiSub *sub = NULL;
    struct termios old_settings;
    int cbreak = 0;
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

    double value = clamp_threshold(setting);
    atomic_store(&g_threshold, value);
    CHECK(wuji_glove_subscribe_tactile_residual(device, on_residual, NULL, &sub));
    cbreak = enter_cbreak_if_tty(&old_settings);
    if (cbreak < 0) { fprintf(stderr, "configure terminal failed: %s\n", strerror(errno)); result = 1; goto cleanup; }
    double started = (double)now_ms() / 1000;
    while (!stop_requested && !atomic_load(&stream_failed)) {
        int key = cbreak ? read_key_nonblocking() : 0;
        if (key == 'q' || key == 'Q') break;
        if (key == '+' || key == '=' || key == '-' || key == '_') {
            value = clamp_threshold(value + ((key == '+' || key == '=') ? 0.1 : -0.1));
            atomic_store(&g_threshold, value);
        }
        if (seconds > 0 && (double)now_ms() / 1000 - started >= seconds) break;
        sleep_ms(10);
    }
cleanup:
    if (cbreak > 0 && tcsetattr(STDIN_FILENO, TCSADRAIN, &old_settings) != 0) {
        fprintf(stderr, "restore terminal failed: %s\n", strerror(errno)); result = 1;
    }
    atomic_store(&closing, 1);
    if (sub != NULL) wuji_sub_close(sub);
    if (atomic_load(&stream_failed)) result = 1;
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
