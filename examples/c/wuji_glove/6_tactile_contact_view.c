/*
 * Wuji SDK C - WujiGlove live tactile contact view.
 *
 * Standalone post-calibration acceptance tool: renders the `tactile_binary`
 * stream as a 24x31 terminal grid.
 * It is not an alternative tactile-calibration workflow.
 *
 * Legend: # = contact, . = hand surface, space = invalid/masked taxel.
 * Press + / - to tune sensitivity, q to quit, or Ctrl-C to stop.
 *
 * Build: point CMake at your extracted SDK tarball (see ../README.md):
 *          cmake -S . -B build \
 *            -DWUJI_SDK_INCLUDE_DIR=/path/to/sdk/include \
 *            -DWUJI_SDK_LIB=/path/to/sdk/lib/libwuji_sdk_c.so
 *          cmake --build build
 * Run:   ./build/6_tactile_contact_view <glove_serial> [seconds] [sensitivity]
 */
#define _POSIX_C_SOURCE 199309L
#include <errno.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <termios.h>
#include <time.h>
#include <unistd.h>

#include "wuji_sdk.h"

#define TACTILE_ROWS 24
#define TACTILE_COLS 31
#define SENSITIVITY_MIN_TENTHS 5
#define SENSITIVITY_MAX_TENTHS 30
#define SENSITIVITY_DEFAULT_TENTHS 10

// Shared across the main thread, the subscription worker thread (render), and
// the SIGINT handler. C11 atomics give the cross-thread ordering that
// `volatile sig_atomic_t` does not; `atomic_int` is lock-free on the C SDK's
// targets, so it is also safe to store from the signal handler.
static atomic_int g_stop = 0;
static atomic_int g_sensitivity_tenths = SENSITIVITY_DEFAULT_TENTHS;

static void on_sigint(int signo) {
    (void)signo;
    atomic_store_explicit(&g_stop, 1, memory_order_relaxed);
}

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

static void print_error(const char *what) {
    const char *err = wuji_last_error();
    fprintf(stderr, "%s: %s\n", what, err ? err : "<no detail>");
}

static int clamp_sensitivity_tenths(int tenths) {
    if (tenths < SENSITIVITY_MIN_TENTHS) return SENSITIVITY_MIN_TENTHS;
    if (tenths > SENSITIVITY_MAX_TENTHS) return SENSITIVITY_MAX_TENTHS;
    return tenths;
}

static void set_sensitivity(struct WujiDevice *dev, int tenths) {
    tenths = clamp_sensitivity_tenths(tenths);
    double value = (double)tenths / 10.0;
    WujiStatus st = wuji_glove_set_tactile_binary_sensitivity(dev, value);
    if (st != WUJI_STATUS_OK) {
        printf("\n(failed to set sensitivity %.1f)\n", value);
        print_error("wuji_glove_set_tactile_binary_sensitivity");
        return;
    }
    atomic_store(&g_sensitivity_tenths, tenths);
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
    if (tcgetattr(STDIN_FILENO, old_settings) != 0) return 0;

    struct termios raw = *old_settings;
    raw.c_lflag &= (tcflag_t) ~(ICANON | ECHO);
    raw.c_cc[VMIN]  = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) return 0;
    return 1;
}

static void render_frame(const WujiTactileBinary *frame) {
    const size_t expected = TACTILE_ROWS * TACTILE_COLS;
    const size_t limit = frame->data_len < expected ? frame->data_len : expected;
    size_t contacts = 0;
    float peak = -1.0f;
    size_t peak_idx = 0;

    for (size_t i = 0; i < limit; i++) {
        float v = frame->data[i];
        if (v > 0.5f) contacts++;
        if (v > peak) {
            peak = v;
            peak_idx = i;
        }
    }

    printf("\x1b[H\x1b[2J");
    printf("contacts=%-3zu sensitivity=%.1f peak=%.3f row=%zu col=%zu len=%zu  (+/- sensitivity, q quit)\n",
           contacts,
           (double)atomic_load(&g_sensitivity_tenths) / 10.0,
           peak,
           peak_idx / TACTILE_COLS,
           peak_idx % TACTILE_COLS,
           frame->data_len);

    for (size_t row = 0; row < TACTILE_ROWS; row++) {
        const size_t base = row * TACTILE_COLS;
        for (size_t col = 0; col < TACTILE_COLS; col++) {
            const size_t idx = base + col;
            const float v = idx < limit ? frame->data[idx] : -1.0f;
            putchar(v > 0.5f ? '#' : (v < -0.5f ? ' ' : '.'));
        }
        putchar('\n');
    }
    fflush(stdout);
}

static void on_contact(WujiFrameKind kind, const WujiTactileBinary *frame, void *userdata) {
    (void)userdata;
    static long last_render_ms = 0;

    if (kind == WUJI_FRAME_KIND_LAG) return;
    if (kind == WUJI_FRAME_KIND_END) {
        printf("\n[end] contact stream ended\n");
        atomic_store(&g_stop, 1);
        return;
    }
    if (kind == WUJI_FRAME_KIND_ERROR) {
        printf("\n[error] contact stream error\n");
        atomic_store(&g_stop, 1);
        return;
    }
    if (kind != WUJI_FRAME_KIND_OK || !frame || !frame->data) return;

    long t = now_ms();
    if (t - last_render_ms < 100) return;
    last_render_ms = t;
    render_frame(frame);
}

int main(int argc, char **argv) {
    if (argc < 2) {
        fprintf(stderr, "Usage: %s <glove_serial> [seconds] [sensitivity]\n", argv[0]);
        return 2;
    }

    const char *sn = argv[1];
    double seconds = (argc >= 3) ? strtod(argv[2], NULL) : 0.0;
    int sensitivity = (argc >= 4)
        ? (int)(strtod(argv[3], NULL) * 10.0 + 0.5)
        : SENSITIVITY_DEFAULT_TENTHS;
    sensitivity = clamp_sensitivity_tenths(sensitivity);
    if (seconds < 0.0) {
        fprintf(stderr, "seconds must be >= 0\n");
        return 2;
    }

    WujiInitOptions init = { .log_level = 0 };
    if (wuji_init(&init) != WUJI_STATUS_OK) {
        print_error("wuji_init");
        return 1;
    }

    printf("Subscribing to tactile_binary. Calibrate the glove first if the stream stays near zero.\n");

    WujiConnectTarget target = { .kind = WUJI_CONNECT_TARGET_KIND_SN, .value = sn };
    struct WujiDevice *dev = NULL;
    if (wuji_connect(&target, "glove", NULL, &dev) != WUJI_STATUS_OK) {
        print_error("wuji_connect");
        wuji_shutdown();
        return 1;
    }

    set_sensitivity(dev, sensitivity);

    struct WujiSub *sub = NULL;
    if (wuji_glove_subscribe_tactile_binary(dev, on_contact, NULL, &sub) != WUJI_STATUS_OK) {
        print_error("wuji_glove_subscribe_tactile_binary");
        wuji_dev_disconnect(dev);
        wuji_dev_release(dev);
        wuji_shutdown();
        return 1;
    }

    struct termios old_settings;
    int cbreak = enter_cbreak_if_tty(&old_settings);
    signal(SIGINT, on_sigint);
    printf("\x1b[H\x1b[2Jwaiting for tactile_binary frames... (+/- sensitivity, q quit)\n");

    long elapsed_ms = 0;
    while (!atomic_load(&g_stop)) {
        int key = cbreak ? read_key_nonblocking() : 0;
        if (key == 'q' || key == 'Q') {
            break;
        } else if (key == '+' || key == '=') {
            set_sensitivity(dev, atomic_load(&g_sensitivity_tenths) + 1);
        } else if (key == '-' || key == '_') {
            set_sensitivity(dev, atomic_load(&g_sensitivity_tenths) - 1);
        }

        sleep_ms(50);
        elapsed_ms += 50;
        if (seconds > 0.0 && elapsed_ms >= (long)(seconds * 1000.0)) break;
    }

    if (cbreak) tcsetattr(STDIN_FILENO, TCSADRAIN, &old_settings);
    printf("\nStopping.\n");

    wuji_sub_close(sub);
    wuji_dev_disconnect(dev);
    wuji_dev_release(dev);
    wuji_shutdown();
    return 0;
}
