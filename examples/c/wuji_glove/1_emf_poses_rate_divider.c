/*
 * Wuji SDK C — WujiGlove emf_poses rate divider.
 *
 * Demonstrates the C API for controlling the emf_poses rate divider.
 *
 * EMF poses are solved at the EMF sensor rate (~120 Hz); every downstream
 * hand-tracking stream is recomputed from each emf_poses frame, and the inverse
 * kinematics behind hand_joint_angles is the most expensive step. Most apps do
 * not need 120 Hz hand tracking.
 *
 *   wuji_glove_set_emf_poses_rate_divider(dev, N)
 *
 * publishes emf_poses at input_rate / N (N=4 -> ~30 Hz). The downstream streams
 * are data-driven, so they all follow automatically and the IK/FK solver runs N
 * times less often — directly cutting CPU. It applies at runtime (a few frames)
 * and persists across reconnects. Default N=1 = no change.
 *
 * This subscribes to the emf-derived streams plus imu_palm (an unaffected
 * reference) and prints each stream's measured rate at N=1 and N=4, so you can
 * see the whole emf_poses chain drop together while the IMU stream stays flat.
 *
 * THREADING: the stream callbacks fire on dedicated SDK worker threads, not
 * main; counters are atomic and read only after the subscriptions are closed.
 *
 * Build: point CMake at your extracted SDK tarball (see ../README.md):
 *          cmake -S . -B build \
 *            -DWUJI_SDK_INCLUDE_DIR=/path/to/sdk/include \
 *            -DWUJI_SDK_LIB=/path/to/sdk/lib/libwuji_sdk_c.so
 *          cmake --build build
 * Run:   ./build/1_emf_poses_rate_divider
 */
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <unistd.h>
#include <stdatomic.h>

#include "wuji_sdk.h"

#define N_STREAMS 6
#define SETTLE_US 600000   /* 0.6 s — let a divider change take effect */
#define WINDOW_S  3.0      /* rate-measurement window */
#define WINDOW_US 3000000

typedef struct {
    const char      *label;
    int              affected;     /* follows the divider? (for the table only) */
    _Atomic uint64_t count;        /* bumped by the worker thread, read by main */
} stream_t;

static stream_t g_streams[N_STREAMS] = {
    { "emf_poses",            1, 0 },
    { "hand_joint_angles",    1, 0 },
    { "tip_poses",            1, 0 },
    { "hand_skeleton",        1, 0 },
    { "tactile_point_cloud",  1, 0 },
    { "imu_palm (reference)", 0, 0 },
};

/* One typed callback per stream schema; each just counts OK frames. The typed
 * payload is ignored — we only measure rate. */
#define COUNT_OK(k, ud) do { \
    if ((k) == WUJI_FRAME_KIND_OK && (ud)) \
        atomic_fetch_add_explicit(&((stream_t *)(ud))->count, 1, memory_order_relaxed); \
} while (0)

static void on_emf_poses  (WujiFrameKind k, const WujiEmfPoseArray   *f, void *ud) { (void)f; COUNT_OK(k, ud); }
static void on_joint_angles(WujiFrameKind k, const WujiHandJointAngles *f, void *ud) { (void)f; COUNT_OK(k, ud); }
static void on_tip_poses  (WujiFrameKind k, const WujiFingertipPoses  *f, void *ud) { (void)f; COUNT_OK(k, ud); }
static void on_skeleton   (WujiFrameKind k, const WujiHandSkeleton    *f, void *ud) { (void)f; COUNT_OK(k, ud); }
static void on_point_cloud(WujiFrameKind k, const WujiPointCloud      *f, void *ud) { (void)f; COUNT_OK(k, ud); }
static void on_imu        (WujiFrameKind k, const WujiImuData         *f, void *ud) { (void)f; COUNT_OK(k, ud); }

/* Set the divider, let it settle, then measure each stream's rate (Hz) over a
 * fixed window. Subscriptions are created fresh each call so counting from
 * "now" gives the live rate directly. Returns 0 on success, or -1 on any
 * set/subscribe failure (after printing wuji_last_error() and closing whatever
 * subscriptions were already opened) so the caller can report and exit. */
static int measure_rates(struct WujiDevice *dev, uint32_t divider, double out_hz[N_STREAMS]) {
    if (wuji_glove_set_emf_poses_rate_divider(dev, divider) != WUJI_STATUS_OK) {
        fprintf(stderr, "set_emf_poses_rate_divider(%u): %s\n", divider, wuji_last_error());
        return -1;
    }
    usleep(SETTLE_US);

    for (int i = 0; i < N_STREAMS; i++)
        atomic_store_explicit(&g_streams[i].count, 0, memory_order_relaxed);

    /* Short-circuits on the first failure; the remaining subscribes are skipped
     * and leave their subs[i] NULL, so the cleanup loop only closes what opened. */
    struct WujiSub *subs[N_STREAMS] = {0};
    if (wuji_glove_subscribe_emf_poses          (dev, on_emf_poses,   &g_streams[0], &subs[0]) != WUJI_STATUS_OK ||
        wuji_glove_subscribe_hand_joint_angles  (dev, on_joint_angles,&g_streams[1], &subs[1]) != WUJI_STATUS_OK ||
        wuji_glove_subscribe_tip_poses          (dev, on_tip_poses,   &g_streams[2], &subs[2]) != WUJI_STATUS_OK ||
        wuji_glove_subscribe_hand_skeleton      (dev, on_skeleton,    &g_streams[3], &subs[3]) != WUJI_STATUS_OK ||
        wuji_glove_subscribe_tactile_point_cloud(dev, on_point_cloud, &g_streams[4], &subs[4]) != WUJI_STATUS_OK ||
        wuji_glove_subscribe_imu_palm           (dev, on_imu,         &g_streams[5], &subs[5]) != WUJI_STATUS_OK) {
        fprintf(stderr, "subscribe failed: %s\n", wuji_last_error());
        for (int i = 0; i < N_STREAMS; i++)
            if (subs[i]) wuji_sub_close(subs[i]);
        return -1;
    }

    usleep(WINDOW_US);

    /* Close first (stops the workers), then read the now-stable counts. */
    for (int i = 0; i < N_STREAMS; i++)
        if (subs[i]) wuji_sub_close(subs[i]);
    for (int i = 0; i < N_STREAMS; i++)
        out_hz[i] = (double)atomic_load_explicit(&g_streams[i].count, memory_order_relaxed) / WINDOW_S;
    return 0;
}

int main(void) {
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
    printf("Connecting to %s (%s)\n\n", list[0].serial_number, list[0].address);
    WujiConnectTarget tgt = { .kind = WUJI_CONNECT_TARGET_KIND_SN, .value = list[0].serial_number };
    struct WujiDevice *dev = NULL;
    WujiStatus st = wuji_connect(&tgt, "glove", NULL, &dev);
    wuji_discovered_free(list, n);
    if (st != WUJI_STATUS_OK) {
        fprintf(stderr, "wuji_connect failed: %s\n", wuji_last_error());
        wuji_shutdown();
        return 1;
    }

    double hz1[N_STREAMS], hz4[N_STREAMS];
    int rc = measure_rates(dev, 1, hz1);
    if (rc == 0)
        rc = measure_rates(dev, 4, hz4);
    wuji_glove_set_emf_poses_rate_divider(dev, 1); /* restore default (best-effort) */
    if (rc != 0) {
        wuji_dev_disconnect(dev);
        wuji_dev_release(dev);
        wuji_shutdown();
        return 1;
    }

    printf("%-24s%10s%10s   %s\n", "stream", "N=1 (Hz)", "N=4 (Hz)", "effect");
    for (int i = 0; i < 60; i++) putchar('-');
    putchar('\n');
    for (int i = 0; i < N_STREAMS; i++)
        printf("%-24s%10.0f%10.0f   %s\n", g_streams[i].label, hz1[i], hz4[i],
               g_streams[i].affected ? "follows divider" : "unaffected");
    printf("\nThe emf_poses chain drops ~4x (less IK/FK CPU); the IMU stream is unchanged.\n");

    wuji_dev_disconnect(dev);
    wuji_dev_release(dev);
    wuji_shutdown();
    return 0;
}
