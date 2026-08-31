/*
 * Wuji SDK C — Wuji Hand 2: export the on-flash log history to a JSONL file.
 *
 * Pulls the whole flash log ring, decodes every frame, and writes one JSON
 * object per line under `out_dir` (or the SDK default `~/.wuji/logs`, next to
 * the SDK's own logs, when NULL). One call returns both the path and the
 * frame count — there is
 * deliberately no "query the size first" step, because every call performs a
 * full export and writes a new file.
 *
 * Build: point CMake at your build tree / extracted SDK tarball:
 *          cmake -S . -B build \
 *            -DWUJI_SDK_INCLUDE_DIR=/path/to/sdk/include \
 *            -DWUJI_SDK_LIB=/path/to/sdk/lib/libwuji_sdk_c.so
 *          cmake --build build
 * Run:   ./build/5_export_flash_logs [out-dir]
 */
#include <stdio.h>
#include <stddef.h>

#include "wuji_sdk.h"

int main(int argc, char **argv) {
    const char *out_dir = (argc >= 2) ? argv[1] : NULL;

    if (wuji_init(NULL) != WUJI_STATUS_OK) {
        fprintf(stderr, "wuji_init failed: %s\n", wuji_last_error());
        return 1;
    }

    WujiDiscovered *list = NULL;
    size_t n = 0;
    if (wuji_scan(&list, &n) != WUJI_STATUS_OK || n == 0) {
        fprintf(stderr, "No devices found\n");
        wuji_discovered_free(list, n);
        wuji_shutdown();
        return 1;
    }

    /* Pick the first Wuji Hand 2 among the scanned devices. */
    int idx = -1;
    for (size_t i = 0; i < n; i++) {
        if (list[i].device_id == WUJI_DEVICE_TYPE_WUJI_HAND_2) { idx = (int)i; break; }
    }
    if (idx < 0) {
        fprintf(stderr, "No Wuji Hand 2 found\n");
        wuji_discovered_free(list, n);
        wuji_shutdown();
        return 1;
    }

    WujiConnectTarget tgt = { .kind = WUJI_CONNECT_TARGET_KIND_SN, .value = list[idx].serial_number };
    struct WujiDevice *dev = NULL;
    WujiStatus st = wuji_connect(&tgt, "wuji_hand_2", NULL, &dev);
    wuji_discovered_free(list, n);
    if (st != WUJI_STATUS_OK) {
        fprintf(stderr, "wuji_connect failed: %s\n", wuji_last_error());
        wuji_shutdown();
        return 1;
    }

    int rc = 0;

    /* One call returns both the path and the frame count. `out` is left
     * untouched on error, so it must only be freed on the success path. */
    WujiFlashLogExport result;
    if (wuji_hand_2_export_flash_logs(dev, out_dir, &result) != WUJI_STATUS_OK) {
        fprintf(stderr, "export_flash_logs failed: %s\n", wuji_last_error());
        rc = 1;
        goto cleanup;
    }

    printf("Decoded %u frames into %s\n", result.frame_count, result.path);
    wuji_hand_2_flash_log_export_free(&result);

cleanup:
    wuji_dev_disconnect(dev);
    wuji_dev_release(dev);
    wuji_shutdown();
    return rc;
}
