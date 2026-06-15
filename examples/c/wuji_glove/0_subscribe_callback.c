/*
 * Wuji SDK C bindings — typed multi-stream subscription.
 *
 * Build: cmake -S . -B build && cmake --build build
 * Run:   ./build/0_subscribe_callback
 *
 * NOTE: subscription callbacks do NOT run on this thread (main). They fire on
 * dedicated SDK worker threads — see the THREADING note above the callbacks.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>
#include <unistd.h>
#include <signal.h>

#include "wuji_sdk.h"

#define MAX_DEVICES 8
#define STREAMS_PER_DEVICE 6
/* + 2 for the global tf / tf_static streams, which are subscribed once total
 * (cross-device aggregated) rather than per device. */
#define MAX_SUBS (MAX_DEVICES * STREAMS_PER_DEVICE + 2)

/* Per-subscription context. The typed wrappers thread `user_data` through;
 * we keep device name + frame count in there for labelled output and
 * throttling. */
typedef struct {
    char     device_name[32];
    const char *stream;
    uint64_t n_frames;
} cb_ctx_t;

static volatile sig_atomic_t g_stop = 0;
static void on_sigint(int sig) { (void)sig; g_stop = 1; }

/* Throttle: only print every 30th OK frame so 100Hz streams don't flood. */
#define BUMP_AND_GATE(ctx_var) \
    do { \
        if (!(ctx_var)) return; \
        (ctx_var)->n_frames++; \
        if ((ctx_var)->n_frames % 30 != 1) return; \
    } while (0)

#define HANDLE_TERMINAL(ctx_var, kind_var) \
    do { \
        if ((kind_var) == WUJI_FRAME_KIND_LAG) { \
            printf("[%s][%s][LAG]\n", (ctx_var) ? (ctx_var)->device_name : "?", \
                                       (ctx_var) ? (ctx_var)->stream : "?"); \
            return; \
        } \
        if ((kind_var) == WUJI_FRAME_KIND_END) { \
            printf("[%s][%s][END]\n", (ctx_var) ? (ctx_var)->device_name : "?", \
                                       (ctx_var) ? (ctx_var)->stream : "?"); \
            return; \
        } \
        if ((kind_var) == WUJI_FRAME_KIND_ERROR) { \
            const char *_err_msg = wuji_last_error(); \
            printf("[%s][%s][ERROR] %s\n", (ctx_var) ? (ctx_var)->device_name : "?", \
                                            (ctx_var) ? (ctx_var)->stream : "?", \
                                            _err_msg ? _err_msg : "(no error message)"); \
            return; \
        } \
    } while (0)

/* ---------- Typed per-stream callbacks (one per schema) ---------- */
/*
 * THREADING — read before adding logic to any callback below.
 *
 * These callbacks do NOT run on the thread that called wuji_glove_subscribe_*()
 * / wuji_subscribe_*() (i.e. NOT main). Each subscription delivers its frames on its own dedicated
 * SDK worker thread:
 *   - One worker thread per subscription — never the caller's / main thread.
 *   - Calls within a single subscription are serialized and delivered in order
 *     (at most one in-flight).
 *   - Different subscriptions run on different threads and may fire
 *     concurrently — synchronize any state shared across callbacks yourself.
 */

static void on_tactile(WujiFrameKind kind, const WujiTactileFrame *f, void *ud) {
    cb_ctx_t *ctx = (cb_ctx_t *)ud;
    HANDLE_TERMINAL(ctx, kind);
    if (kind != WUJI_FRAME_KIND_OK || !f) return;
    BUMP_AND_GATE(ctx);
    float max_val = 0.0f;
    for (size_t i = 0; i < f->data_len; i++) {
        if (f->data[i] > max_val) max_val = f->data[i];
    }
    printf("[%s][Tactile] seq=%u taxels=%zu max=%.3f (#%lu)\n",
           ctx->device_name, f->header.seq, f->data_len, max_val, ctx->n_frames);
}

static void on_tactile_zones(WujiFrameKind kind, const WujiTactileZones *f, void *ud) {
    cb_ctx_t *ctx = (cb_ctx_t *)ud;
    HANDLE_TERMINAL(ctx, kind);
    if (kind != WUJI_FRAME_KIND_OK || !f) return;
    BUMP_AND_GATE(ctx);
    float palm = (f->palm_len > 0) ? f->palm[0] : 0.0f;
    printf("[%s][Zones] seq=%u palm=%.3f (#%lu)\n",
           ctx->device_name, f->header.seq, palm, ctx->n_frames);
}

static void on_emf_poses(WujiFrameKind kind, const WujiEmfPoseArray *f, void *ud) {
    cb_ctx_t *ctx = (cb_ctx_t *)ud;
    HANDLE_TERMINAL(ctx, kind);
    if (kind != WUJI_FRAME_KIND_OK || !f) return;
    BUMP_AND_GATE(ctx);
    if (f->poses_len == 0) {
        printf("[%s][EmfPoses] seq=%u (no poses) (#%lu)\n",
               ctx->device_name, f->header.seq, ctx->n_frames);
        return;
    }
    const float *p = f->poses[0].pose.position;
    printf("[%s][EmfPoses] seq=%u pos=[%+.3f,%+.3f,%+.3f] (#%lu)\n",
           ctx->device_name, f->header.seq, p[0], p[1], p[2], ctx->n_frames);
}

static void on_joint_angles(WujiFrameKind kind, const WujiHandJointAngles *f, void *ud) {
    cb_ctx_t *ctx = (cb_ctx_t *)ud;
    HANDLE_TERMINAL(ctx, kind);
    if (kind != WUJI_FRAME_KIND_OK || !f) return;
    BUMP_AND_GATE(ctx);
    if (f->fingers_len < 5) {
        printf("[%s][JointAngles] seq=%u (insufficient fingers: %zu) (#%lu)\n",
               ctx->device_name, f->header.seq, f->fingers_len, ctx->n_frames);
        return;
    }
    printf("[%s][JointAngles] seq=%u conf=[%.2f,%.2f,%.2f,%.2f,%.2f] (#%lu)\n",
           ctx->device_name, f->header.seq,
           f->fingers[0].confidence, f->fingers[1].confidence,
           f->fingers[2].confidence, f->fingers[3].confidence,
           f->fingers[4].confidence, ctx->n_frames);
}

static void on_skeleton(WujiFrameKind kind, const WujiHandSkeleton *f, void *ud) {
    cb_ctx_t *ctx = (cb_ctx_t *)ud;
    HANDLE_TERMINAL(ctx, kind);
    if (kind != WUJI_FRAME_KIND_OK || !f) return;
    BUMP_AND_GATE(ctx);
    if (f->joints_len == 0) return;
    const float *p = f->joints[0].pose.position;
    printf("[%s][Skeleton] seq=%u wrist=[%+.3f,%+.3f,%+.3f] joints=%zu (#%lu)\n",
           ctx->device_name, f->header.seq, p[0], p[1], p[2], f->joints_len, ctx->n_frames);
}

static void on_point_cloud(WujiFrameKind kind, const WujiPointCloud *f, void *ud) {
    cb_ctx_t *ctx = (cb_ctx_t *)ud;
    HANDLE_TERMINAL(ctx, kind);
    if (kind != WUJI_FRAME_KIND_OK || !f) return;
    BUMP_AND_GATE(ctx);
    size_t pts = (f->point_stride > 0) ? f->data_len / f->point_stride : 0;
    printf("[%s][PointCloud] seq=%u points=%zu frame=%s (#%lu)\n",
           ctx->device_name, f->header.seq, pts, f->frame_id, ctx->n_frames);
}

/* Shared by the global tf and tf_static streams (both yield WujiFrameTransforms).
 * `ctx->stream` distinguishes which one. Unlike the device frames above,
 * WujiFrameTransforms has no `header` — it is a flat list of transforms. */
static void on_transforms(WujiFrameKind kind, const WujiFrameTransforms *f, void *ud) {
    cb_ctx_t *ctx = (cb_ctx_t *)ud;
    HANDLE_TERMINAL(ctx, kind);
    if (kind != WUJI_FRAME_KIND_OK || !f) return;
    BUMP_AND_GATE(ctx);
    if (f->transforms_len == 0) {
        printf("[%s][%s] (no transforms) (#%lu)\n",
               ctx->device_name, ctx->stream, ctx->n_frames);
        return;
    }
    const WujiFrameTransform *t = &f->transforms[0];
    printf("[%s][%s] count=%zu first=%s->%s xyz=[%+.3f,%+.3f,%+.3f] (#%lu)\n",
           ctx->device_name, ctx->stream, f->transforms_len,
           t->parent_frame_id, t->child_frame_id,
           t->translation[0], t->translation[1], t->translation[2], ctx->n_frames);
}

/* ---------- Helper macro: subscribe one stream, register ctx ---------- */
/*
 * Note: each typed wrapper expects a callback whose signature matches its own
 * `Wuji<X>SubCallback` typedef. The compiler enforces the match — passing
 * `on_tactile` to `wuji_glove_subscribe_emf_poses` would be a type error.
 */
#define SUB(fn, cb, stream_label) \
    do { \
        cb_ctx_t *ctx = &ctxs[n_subs]; \
        snprintf(ctx->device_name, sizeof(ctx->device_name), "%s", alias); \
        ctx->stream = (stream_label); \
        ctx->n_frames = 0; \
        if (fn(devices[i], cb, ctx, &subs[n_subs]) != WUJI_STATUS_OK) { \
            fprintf(stderr, "  %s failed: %s\n", #fn, wuji_last_error()); \
            subs[n_subs] = NULL; \
        } else { \
            n_subs++; \
        } \
    } while (0)

/* Like SUB, but for global (cross-device) topics: the wrapper takes no device
 * handle, so it is called once total rather than per device. */
#define SUB_GLOBAL(fn, cb, stream_label) \
    do { \
        cb_ctx_t *ctx = &ctxs[n_subs]; \
        snprintf(ctx->device_name, sizeof(ctx->device_name), "%s", "global"); \
        ctx->stream = (stream_label); \
        ctx->n_frames = 0; \
        if (fn(cb, ctx, &subs[n_subs]) != WUJI_STATUS_OK) { \
            fprintf(stderr, "  %s failed: %s\n", #fn, wuji_last_error()); \
            subs[n_subs] = NULL; \
        } else { \
            n_subs++; \
        } \
    } while (0)

int main(void) {
    signal(SIGINT, on_sigint);

    printf("Wuji SDK version: %s\n", wuji_version());

    WujiInitOptions opts = { .log_level = 3 };
    if (wuji_init(&opts) != WUJI_STATUS_OK) {
        fprintf(stderr, "wuji_init failed: %s\n", wuji_last_error());
        return 1;
    }

    WujiDiscovered *list = NULL;
    size_t n_dev_total = 0;
    if (wuji_scan(&list, &n_dev_total) != WUJI_STATUS_OK) {
        fprintf(stderr, "wuji_scan failed: %s\n", wuji_last_error());
        wuji_shutdown();
        return 1;
    }
    if (n_dev_total == 0) {
        printf("No devices found\n");
        wuji_discovered_free(list, n_dev_total);
        wuji_shutdown();
        return 0;
    }
    size_t n_dev_used = (n_dev_total > MAX_DEVICES) ? MAX_DEVICES : n_dev_total;

    printf("Found %zu device(s)%s\n",
           n_dev_total,
           (n_dev_used < n_dev_total) ? " (capped to MAX_DEVICES)" : "");
    for (size_t i = 0; i < n_dev_used; i++) {
        printf("  SN=%s  Address=%s\n", list[i].serial_number, list[i].address);
    }

    WujiDevice *devices[MAX_DEVICES] = {0};
    WujiSub    *subs[MAX_SUBS]       = {0};
    cb_ctx_t    ctxs[MAX_SUBS];
    memset(ctxs, 0, sizeof(ctxs));
    size_t n_subs = 0;

    for (size_t i = 0; i < n_dev_used; i++) {
        char alias[32];
        snprintf(alias, sizeof(alias), "glove_%zu", i);

        WujiConnectTarget tgt = {
            .kind  = WUJI_CONNECT_TARGET_KIND_SN,
            .value = list[i].serial_number,
        };
        if (wuji_connect(&tgt, alias, NULL, &devices[i]) != WUJI_STATUS_OK) {
            fprintf(stderr, "wuji_connect(%s) failed: %s\n", alias, wuji_last_error());
            continue;
        }
        printf("Connected: %s as %s\n", list[i].serial_number, alias);

        /* Subscribe to several typed device streams. */
        SUB(wuji_glove_subscribe_tactile,           on_tactile,       "Tactile");
        SUB(wuji_glove_subscribe_tactile_zones,     on_tactile_zones, "Zones");
        SUB(wuji_glove_subscribe_emf_poses,         on_emf_poses,     "EmfPoses");
        SUB(wuji_glove_subscribe_hand_joint_angles, on_joint_angles,  "JointAngles");
        SUB(wuji_glove_subscribe_hand_skeleton,     on_skeleton,      "Skeleton");
        SUB(wuji_glove_subscribe_tactile_point_cloud, on_point_cloud, "PointCloud");
    }
    wuji_discovered_free(list, n_dev_total);

    /* Global (cross-device) coordinate-transform streams. These aggregate data
     * across all connected devices, so they take no device handle and are
     * subscribed once (not per device). They carry data while a glove streams. */
    SUB_GLOBAL(wuji_subscribe_tf,        on_transforms, "tf");
    SUB_GLOBAL(wuji_subscribe_tf_static, on_transforms, "tf_static");

    if (n_subs == 0) {
        fprintf(stderr, "No subscriptions opened; exiting.\n");
    } else {
        printf("Subscribed to %zu streams. Ctrl+C to stop.\n\n", n_subs);
    }

    while (!g_stop && n_subs > 0) {
        sleep(1);
    }
    printf("\nShutting down...\n");

    /* Close in reverse order: subs → devices → shutdown. */
    for (size_t i = 0; i < n_subs; i++) {
        if (subs[i]) wuji_sub_close(subs[i]);
    }
    for (size_t i = 0; i < n_dev_used; i++) {
        if (devices[i]) {
            wuji_dev_disconnect(devices[i]);
            wuji_dev_release(devices[i]);
        }
    }
    wuji_shutdown();
    return 0;
}
