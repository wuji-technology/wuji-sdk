/* Record public SUB topics to MCAP. Example:
 *   ./0_record_topics SN001 episode.mcap lz4 emf_poses imu_raw/palm
 * Uses SN connection. Every output path must be unique: existing files are
 * overwritten. Control and cleanup run on this application thread.
 */
#include "wuji_sdk.h"
#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

static int check(WujiStatus status) {
    if (status == WUJI_STATUS_OK) return 1;
    const char *error = wuji_last_error();
    fprintf(stderr, "SDK error %d: %s\n", (int)status, error ? error : "unknown");
    return 0;
}
static void wait_seconds(unsigned seconds) {
    struct timespec remaining = {(time_t)seconds, 0};
    while (nanosleep(&remaining, &remaining) != 0 && errno == EINTR) {}
}
int main(int argc, char **argv) {
    if (argc < 6) {
        fprintf(stderr, "Usage: %s DEVICE_SN OUTPUT.mcap lz4|zstd|none TOPIC TOPIC [TOPIC...]\n", argv[0]);
        return 2;
    }
    int rc = 1;
    WujiDevice *device = NULL;
    WujiTopicRecorder *recorder = NULL;
    WujiRecordingHandle *handle = NULL;
    WujiRecordingSummary summary = {0};
    WujiRecordingOptions options = {.chunk_size = 0};
    if (strcmp(argv[3], "lz4") == 0) options.compression = WUJI_RECORDING_COMPRESSION_LZ4;
    else if (strcmp(argv[3], "zstd") == 0) options.compression = WUJI_RECORDING_COMPRESSION_ZSTD;
    else if (strcmp(argv[3], "none") == 0) options.compression = WUJI_RECORDING_COMPRESSION_NONE;
    else { fprintf(stderr, "Invalid compression: %s\n", argv[3]); return 2; }
    WujiConnectTarget target = {.kind = WUJI_CONNECT_TARGET_KIND_SN, .value = argv[1]};
    if (!check(wuji_init(NULL))) goto cleanup;
    if (!check(wuji_connect(&target, "recording_device", NULL, &device))) goto cleanup;
    if (!check(wuji_topic_recorder_create(&options, &recorder))) goto cleanup;
    for (int i = 4; i < argc; ++i) {
        if (!check(wuji_topic_recorder_record(recorder, device, argv[i]))) goto cleanup;
    }
    if (!check(wuji_topic_recorder_start(recorder, argv[2], &handle))) goto cleanup;
    wait_seconds(30);
    if (!check(wuji_recording_handle_pause(handle))) goto cleanup;
    wait_seconds(10);
    if (!check(wuji_recording_handle_resume(handle))) goto cleanup;
    wait_seconds(30);
    if (!check(wuji_recording_handle_stop(handle, &summary))) goto cleanup;
    printf("Recorded %" PRIu64 " messages, %.3f s, %" PRIu64 " bytes\n",
           summary.total_frames, summary.duration_s, summary.file_size);
    for (size_t i = 0; i < summary.channels_len; ++i) {
        printf("%s: %" PRIu64 "\n", summary.channels[i].channel_name,
               summary.channels[i].frame_count);
    }
    rc = 0;
cleanup:
    wuji_recording_summary_free(&summary);
    wuji_recording_handle_free(handle); /* Also finishes when stop was skipped. */
    wuji_topic_recorder_free(recorder);
    if (device) { wuji_dev_disconnect(device); wuji_dev_release(device); }
    wuji_shutdown();
    return rc;
}
