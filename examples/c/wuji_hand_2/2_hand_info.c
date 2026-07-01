/*
 * Wuji SDK C — Wuji Hand 2: read hand info, with the online bitmap.
 *
 * Scans for a hand, connects to the first one found, and reads:
 *   - identity   : handedness, IP (string size-query), online joint count
 *   - per-joint  : get_all_effort_limit(...) into a dense out[20] + an
 *                  `online` bitmap that marks which joints are present
 *   - snapshot   : one joint_diagnostics frame (status_word / current /
 *                  bus voltage / temperature / error code per online joint)
 *
 * THE ONLINE BITMAP (the focus of this example)
 * ---------------------------------------------
 * The whole-hand reads (get_all_effort_limit / get_all_mit_params / ...) fill a
 * caller array of WUJI_HAND_2_JOINT_COUNT (20) elements AND set a `uint32_t
 * online` out-param. Bit i of `online` is 1 iff joint i is present this read;
 * `out[i]` is valid only then. Offline slots are zeroed, so you MUST gate on
 * the bit before trusting out[i]:
 *
 *     uint32_t online;
 *     float limits[WUJI_HAND_2_JOINT_COUNT];
 *     wuji_hand_2_get_all_effort_limit(dev, limits, &online);
 *     for (int i = 0; i < WUJI_HAND_2_JOINT_COUNT; i++)
 *         if (WUJI_JOINT_ONLINE(online, i))     // <-- check the bit first
 *             use(limits[i]);                    //     then read the slot
 *
 * This mirrors POSIX fd_set / select(): a fixed index space (0..19) whose
 * present subset is a bitmask. The joint count = popcount(online).
 *
 * Build: point CMake at your extracted SDK tarball (see ../README.md):
 *          cmake -S . -B build \
 *            -DWUJI_SDK_INCLUDE_DIR=/path/to/sdk/include \
 *            -DWUJI_SDK_LIB=/path/to/sdk/lib/libwuji_sdk_c.so
 *          cmake --build build
 * Run:   ./build/2_hand_info
 */
#define _DEFAULT_SOURCE /* usleep() */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <signal.h>
#include <unistd.h>
#include <stdatomic.h>

#include "wuji_sdk.h"

/* Number of set bits = number of online joints (= wuji_hand_2_online_joints_count). */
static int popcount32(uint32_t v) {
    int c = 0;
    for (; v; v &= v - 1) c++;
    return c;
}

/* One-shot capture of the first joint_diagnostics frame, written by the
 * subscription worker thread and read back by main after a short wait. */
typedef struct {
    _Atomic int got; /* release/acquire flag publishing entries[]/count to main */
    WujiJointDiagnosticsEntry entries[WUJI_HAND_2_JOINT_COUNT];
    uint8_t count;
} diag_snapshot_t;

static void on_diag(WujiFrameKind kind, const WujiJointDiagnosticsFrame *f, void *ud) {
    diag_snapshot_t *s = (diag_snapshot_t *)ud;
    if (atomic_load_explicit(&s->got, memory_order_relaxed) || kind != WUJI_FRAME_KIND_OK || !f)
        return;
    uint8_t n = f->joints_len < WUJI_HAND_2_JOINT_COUNT
                    ? (uint8_t)f->joints_len
                    : WUJI_HAND_2_JOINT_COUNT;
    for (uint8_t i = 0; i < n; i++) s->entries[i] = f->joints[i];
    s->count = n;
    /* release: main's acquire-load of `got` sees entries[]/count written above. */
    atomic_store_explicit(&s->got, 1, memory_order_release);
}

int main(void) {
    WujiInitOptions opts = { .log_level = 3 };
    if (wuji_init(&opts) != WUJI_STATUS_OK) {
        fprintf(stderr, "wuji_init failed: %s\n", wuji_last_error());
        return 1;
    }

    /* Discover and connect to the first hand found. */
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
    printf("Connecting to %s (%s)\n", list[0].serial_number, list[0].address);
    WujiConnectTarget tgt = { .kind = WUJI_CONNECT_TARGET_KIND_SN, .value = list[0].serial_number };
    struct WujiDevice *dev = NULL;
    WujiStatus st = wuji_connect(&tgt, "hand2", NULL, &dev);
    wuji_discovered_free(list, n);
    if (st != WUJI_STATUS_OK) {
        fprintf(stderr, "wuji_connect failed: %s\n", wuji_last_error());
        wuji_shutdown();
        return 1;
    }

    /* --- identity --- */
    WujiHandedness hand;
    if (wuji_hand_2_get_handedness(dev, &hand) == WUJI_STATUS_OK)
        printf("handedness: %s\n", hand == WUJI_HANDEDNESS_LEFT ? "left" : "right");

    /* String getter via the Win32 two-call size-query: 1) ask for the size with
     * buf=NULL, 2) allocate, 3) fill. */
    size_t needed = 0;
    if (wuji_hand_2_get_ip(dev, NULL, 0, &needed) == WUJI_STATUS_ERR_BUFFER_TOO_SMALL) {
        char *ip = (char *)malloc(needed);
        if (ip && wuji_hand_2_get_ip(dev, ip, needed, NULL) == WUJI_STATUS_OK)
            printf("ip: %s\n", ip);
        free(ip);
    }

    uint8_t count = 0;
    if (wuji_hand_2_online_joints_count(dev, &count) == WUJI_STATUS_OK)
        printf("online joints (count): %u\n", count);

    /* --- per-joint effort limit, gated by the online bitmap --- */
    float limits[WUJI_HAND_2_JOINT_COUNT];
    uint32_t online = 0;
    if (wuji_hand_2_get_all_effort_limit(dev, limits, &online) != WUJI_STATUS_OK) {
        fprintf(stderr, "get_all_effort_limit: %s\n", wuji_last_error());
        goto cleanup;
    }

    printf("\nonline bitmap = 0x%08X  (%d joints online)\n", online, popcount32(online));
    printf("  %-3s %-9s %-8s %12s\n", "i", "label", "state", "effort_lim");
    for (int i = 0; i < WUJI_HAND_2_JOINT_COUNT; i++) {
        char label[16];
        if (wuji_hand_2_joint_label((uint8_t)i, label, sizeof(label), NULL) != WUJI_STATUS_OK)
            snprintf(label, sizeof(label), "joint_%d", i);

        /* Gate on the bit: only read limits[i] when joint i is online. */
        if (!WUJI_JOINT_ONLINE(online, i)) {
            printf("  %-3d %-9s %-8s\n", i, label, "offline");
            continue;
        }
        printf("  %-3d %-9s %-8s %10.3fA\n", i, label, "online", (double)limits[i]);
    }

    /* --- optional: one joint_diagnostics snapshot (subscribe ~1s) --- */
    diag_snapshot_t snap = { 0 };
    struct WujiSub *sub = NULL;
    if (wuji_hand_2_subscribe_joint_diagnostics(dev, on_diag, &snap, &sub) == WUJI_STATUS_OK) {
        for (int i = 0; i < 100 && !atomic_load_explicit(&snap.got, memory_order_acquire); i++)
            usleep(10000); /* up to ~1s */
        wuji_sub_close(sub);
    }
    if (atomic_load_explicit(&snap.got, memory_order_acquire)) {
        printf("\njoint_diagnostics snapshot (%u joints):\n", snap.count);
        printf("  %-4s %-12s %8s %7s %6s %8s\n",
               "nid", "status_word", "current", "vbus", "temp", "err_code");
        for (uint8_t i = 0; i < snap.count; i++) {
            const WujiJointDiagnosticsEntry *e = &snap.entries[i];
            printf("  %-4u 0x%08X %7.2fA %6.1fV %5.1fC   0x%04X\n",
                   e->nid, e->status_word, (double)e->current,
                   (double)e->vbus_v_fb, (double)e->mcu_temp_c_fb, e->error_code_current);
        }
    } else {
        printf("\n(no joint_diagnostics frame received)\n");
    }

cleanup:
    wuji_dev_disconnect(dev);
    wuji_dev_release(dev);
    wuji_shutdown();
    return 0;
}
