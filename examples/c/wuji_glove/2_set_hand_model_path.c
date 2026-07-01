/*
 * Wuji SDK C — WujiGlove online IK custom URDF.
 *
 * This is the C equivalent of:
 *
 *   glove.hand_model_path().set("/path/to/hand.urdf")
 *   print(glove.hand_model_path().get())
 *   glove.hand_joint_angles().subscribe_with_callback(...)
 *
 * `wuji_glove_set_hand_model_path()` stores the URDF path in the SDK parameter
 * store for the connected glove; `wuji_glove_get_hand_model_path()` reads it
 * back. The online IK streams reload the model through the SDK's existing
 * calibration-generation mechanism.
 *
 * Build: point CMake at your extracted SDK tarball (see ../README.md):
 *          cmake -S . -B build \
 *            -DWUJI_SDK_INCLUDE_DIR=/path/to/sdk/include \
 *            -DWUJI_SDK_LIB=/path/to/sdk/lib/libwuji_sdk_c.so
 *          cmake --build build
 * Run:   ./build/2_set_hand_model_path /absolute/path/to/hand.urdf [serial_number]
 *
 * Expected output: after "Subscribed to hand_joint_angles", the program prints
 * IK frames. Confidence values > 0 indicate the online IK solver is producing
 * usable hand joint angles with the selected model.
 */
#include <errno.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "wuji_sdk.h"

static volatile sig_atomic_t g_stop = 0;
static void on_sigint(int sig) {
    (void)sig;
    g_stop = 1;
}

typedef struct {
    uint64_t n_frames;
} ik_ctx_t;

static int file_exists(const char *path) {
    return path && access(path, R_OK) == 0;
}

static void print_usage(const char *argv0) {
    fprintf(stderr, "Usage: %s /absolute/path/to/hand.urdf [serial_number]\n", argv0);
}

static int find_target(const WujiDiscovered *list, size_t n, const char *target_sn) {
    if (!target_sn || !target_sn[0]) return n > 0 ? 0 : -1;

    for (size_t i = 0; i < n; i++) {
        if (strcmp(list[i].serial_number, target_sn) == 0) return (int)i;
    }
    return -1;
}

static int print_hand_model_path(struct WujiDevice *dev) {
    size_t needed = 0;
    WujiStatus st = wuji_glove_get_hand_model_path(dev, NULL, 0, &needed);
    if (st != WUJI_STATUS_ERR_BUFFER_TOO_SMALL || needed == 0) {
        fprintf(stderr, "wuji_glove_get_hand_model_path size query failed: %s\n", wuji_last_error());
        return -1;
    }

    char *buf = (char *)malloc(needed);
    if (!buf) {
        fprintf(stderr, "malloc failed for hand_model_path buffer\n");
        return -1;
    }

    st = wuji_glove_get_hand_model_path(dev, buf, needed, &needed);
    if (st != WUJI_STATUS_OK) {
        fprintf(stderr, "wuji_glove_get_hand_model_path failed: %s\n", wuji_last_error());
        free(buf);
        return -1;
    }

    printf("Read back hand_model_path = %s\n", buf);
    free(buf);
    return 0;
}

static void on_hand_joint_angles(WujiFrameKind kind, const WujiHandJointAngles *frame, void *user_data) {
    ik_ctx_t *ctx = (ik_ctx_t *)user_data;

    if (kind == WUJI_FRAME_KIND_LAG) {
        fprintf(stderr, "[hand_joint_angles] lagged\n");
        return;
    }
    if (kind == WUJI_FRAME_KIND_END) {
        fprintf(stderr, "[hand_joint_angles] stream ended\n");
        return;
    }
    if (kind == WUJI_FRAME_KIND_ERROR) {
        const char *err = wuji_last_error();
        fprintf(stderr, "[hand_joint_angles] error: %s\n", err ? err : "(no error message)");
        return;
    }
    if (kind != WUJI_FRAME_KIND_OK || !frame || !ctx) return;

    ctx->n_frames++;
    if (ctx->n_frames % 30 != 1) return; /* throttle terminal output */

    printf("seq=%u frame_id=%s fingers=%zu frame=%llu\n",
           frame->header.seq,
           frame->header.frame_id,
           frame->fingers_len,
           (unsigned long long)ctx->n_frames);

    for (size_t i = 0; i < frame->fingers_len; i++) {
        const WujiFingerJointAngles *finger = &frame->fingers[i];
        printf("  finger[%zu] confidence=%.3f angles=[%.4f %.4f %.4f %.4f %.4f]\n",
               i,
               finger->confidence,
               finger->angles[0],
               finger->angles[1],
               finger->angles[2],
               finger->angles[3],
               finger->angles[4]);
    }
    fflush(stdout);
}

int main(int argc, char **argv) {
    signal(SIGINT, on_sigint);

    if (argc < 2 || argc > 3) {
        print_usage(argv[0]);
        return 2;
    }

    const char *urdf_path = argv[1];
    const char *target_sn = (argc == 3) ? argv[2] : NULL;
    if (!file_exists(urdf_path)) {
        fprintf(stderr, "URDF is not readable: %s (%s)\n", urdf_path, strerror(errno));
        return 2;
    }

    WujiInitOptions opts = { .log_level = 3 };
    if (wuji_init(&opts) != WUJI_STATUS_OK) {
        fprintf(stderr, "wuji_init failed: %s\n", wuji_last_error());
        return 1;
    }

    WujiDiscovered *list = NULL;
    size_t n = 0;
    if (wuji_scan(&list, &n) != WUJI_STATUS_OK) {
        fprintf(stderr, "wuji_scan failed: %s\n", wuji_last_error());
        wuji_shutdown();
        return 1;
    }
    if (n == 0) {
        printf("No devices found\n");
        wuji_discovered_free(list, n);
        wuji_shutdown();
        return 0;
    }

    int idx = find_target(list, n, target_sn);
    if (idx < 0) {
        fprintf(stderr, "Device not found: %s\n", target_sn);
        wuji_discovered_free(list, n);
        wuji_shutdown();
        return 1;
    }

    printf("Connecting to %s (%s)\n", list[idx].serial_number, list[idx].address);
    WujiConnectTarget connect_target = {
        .kind = WUJI_CONNECT_TARGET_KIND_SN,
        .value = list[idx].serial_number,
    };

    struct WujiDevice *dev = NULL;
    WujiStatus st = wuji_connect(&connect_target, "glove_ik", NULL, &dev);
    wuji_discovered_free(list, n);

    if (st != WUJI_STATUS_OK) {
        fprintf(stderr, "wuji_connect failed: %s\n", wuji_last_error());
        wuji_shutdown();
        return 1;
    }

    if (wuji_glove_set_hand_model_path(dev, urdf_path) != WUJI_STATUS_OK) {
        fprintf(stderr, "wuji_glove_set_hand_model_path failed: %s\n", wuji_last_error());
        wuji_dev_disconnect(dev);
        wuji_dev_release(dev);
        wuji_shutdown();
        return 1;
    }
    printf("Set hand_model_path = %s\n", urdf_path);
    if (print_hand_model_path(dev) != 0) {
        wuji_dev_disconnect(dev);
        wuji_dev_release(dev);
        wuji_shutdown();
        return 1;
    }

    ik_ctx_t ctx = { 0 };
    struct WujiSub *sub = NULL;
    if (wuji_glove_subscribe_hand_joint_angles(dev, on_hand_joint_angles, &ctx, &sub) != WUJI_STATUS_OK) {
        fprintf(stderr, "subscribe hand_joint_angles failed: %s\n", wuji_last_error());
        wuji_dev_disconnect(dev);
        wuji_dev_release(dev);
        wuji_shutdown();
        return 1;
    }

    printf("Subscribed to hand_joint_angles. Move the glove; Ctrl+C to stop.\n");
    while (!g_stop) sleep(1);

    printf("\nStopping after %llu IK frames...\n", (unsigned long long)ctx.n_frames);
    wuji_sub_close(sub);
    wuji_dev_disconnect(dev);
    wuji_dev_release(dev);
    wuji_shutdown();
    return 0;
}
