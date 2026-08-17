/*
 * Wuji SDK C — Wuji Hand 2: fingertip sensor info + data.
 *
 * The fingertip sensor is self-describing. This example:
 *   - reads each finger's info with ONE high-level call,
 *     wuji_hand_2_get_fingertip_info() -> WujiFingertipSensorInfo. The SDK
 *     drives the chunked info read internally; the caller never sees the
 *     chunked-read transport. `info.format` is a JSON string describing the
 *     data-frame layout (a point array) and where that array sits on the hand.
 *   - decodes the format: `point_fields` gives each point's byte layout, and
 *     `positions` / `point_rpy` / `base_xyz` / `base_rpy` place the points on
 *     the hand model. `positions` and `point_rpy` are in the sensor module's own
 *     base frame; `base_xyz` and `base_rpy` give that frame's pose relative to
 *     the finger's `tip_sensor_frame` link, so point i lands at
 *     R(base_rpy) * positions[i] + base_xyz. The hand reports the pose for
 *     whichever hand it is, so this works unchanged on a left or a right hand.
 *   - subscribes to the five per-finger data streams (~100 Hz,
 *     WujiFingertipSensorData) and reports the hardest-pressed point together
 *     with where that point sits on the fingertip.
 *
 * The JSON reader below is deliberately tiny — it only handles the flat numeric
 * keys and arrays this contract uses. Use a real JSON parser in production code.
 *
 * Build: point CMake at your extracted SDK tarball (see ../README.md):
 *          cmake -S . -B build \
 *            -DWUJI_SDK_INCLUDE_DIR=/path/to/sdk/include \
 *            -DWUJI_SDK_LIB=/path/to/sdk/lib/libwuji_sdk_c.so
 *          cmake --build build
 * Run:   ./build/3_fingertip        (Ctrl+C to stop)
 *
 * THREADING: each callback fires on a dedicated SDK worker thread, NOT main.
 * The `frame` pointer is valid only for the duration of the callback (its heap
 * `data` buffer is freed right after the callback returns) — never stash it.
 */
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <time.h>
#include <math.h>

#include "wuji_sdk.h"

#define MAX_POINTS 40  /* thumb has the most; standard fingers have 34 */

/* "In contact" threshold per unit the format declares — never assume one of them.
 * Firmware v2.4.0 and later reports per-point force normalized to full scale;
 * earlier firmware reports newtons, where 0.02 would be far too sensitive. */
static const struct { const char *unit; double threshold; } CONTACT_BY_UNIT[] = {
    { "normalized", 0.02 },
    { "N",          0.2  },
};

static volatile sig_atomic_t g_stop = 0;
static void on_sigint(int sig) { (void)sig; g_stop = 1; }

/* ---------------------------------------------------------------- tiny JSON */

/* End of the array or object whose opening bracket `p` points at, one past its
 * closing bracket. Strings are skipped so brackets inside them do not count. */
static const char *json_value_end(const char *p) {
    if (!p || (*p != '[' && *p != '{')) return NULL;
    int depth = 0, in_str = 0;
    for (; *p; p++) {
        if (in_str) {
            if (*p == '\\' && p[1]) p++;
            else if (*p == '"') in_str = 0;
            continue;
        }
        if (*p == '"') in_str = 1;
        else if (*p == '[' || *p == '{') depth++;
        else if (*p == ']' || *p == '}') {
            if (--depth == 0) return p + 1;
        }
    }
    return NULL;
}

/* First character of `key`'s value, or NULL.
 *
 * `want_depth` is nesting depth counted from `json`: the keys of the outermost
 * object are at depth 1, keys of an object nested one array deep are at 3, and
 * -1 matches any depth. `stop` bounds the search (NULL = to end of string).
 *
 * Depth and bounds both matter here: a plain substring search for "unit" finds
 * the one inside `aggregate_fields` ("N") long before the top-level default
 * ("normalized"), which silently picks the wrong contact threshold. */
static const char *json_find(const char *json, const char *key, int want_depth,
                             const char *stop) {
    if (!json) return NULL;
    size_t klen = strlen(key);
    int depth = 0;
    for (const char *p = json; *p && (!stop || p < stop); p++) {
        if (*p == '"') {
            if ((want_depth < 0 || depth == want_depth) &&
                strncmp(p + 1, key, klen) == 0 && p[1 + klen] == '"') {
                const char *q = p + 1 + klen + 1;
                while (*q == ' ' || *q == '\t' || *q == '\n' || *q == '\r') q++;
                if (*q == ':') {
                    q++;
                    /* land on the value itself: callers bracket-match on it, and a
                     * stray space would make json_value_end() give up and turn a
                     * bounded search back into an unbounded one */
                    while (*q == ' ' || *q == '\t' || *q == '\n' || *q == '\r') q++;
                    return q;
                }
            }
            /* skip the string body so its contents never affect depth */
            for (p++; *p && *p != '"'; p++)
                if (*p == '\\' && p[1]) p++;
            continue;
        }
        if (*p == '[' || *p == '{') depth++;
        else if (*p == ']' || *p == '}') depth--;
    }
    return NULL;
}

/* Value of a key on the format's outermost object. */
static const char *json_value(const char *json, const char *key) {
    return json_find(json, key, 1, NULL);
}

/* Read up to `max` numbers from `p`, skipping any brackets/commas in between.
 * Stops at the matching close of the outermost bracket p points at. Returns the
 * count read, or -1 if p is NULL. Handles both [a,b,c] and [[a,b,c],[d,e,f]]. */
static int json_numbers(const char *p, double *out, int max) {
    if (!p) return -1;
    int depth = 0, n = 0;
    while (*p) {
        if (*p == '[') { depth++; p++; continue; }
        if (*p == ']') { if (--depth == 0) break; p++; continue; }
        /* separators and any whitespace, so a pretty-printed format parses too */
        if (*p == ',' || *p == ' ' || *p == '\t' || *p == '\n' || *p == '\r') { p++; continue; }
        char *end;
        double v = strtod(p, &end);
        if (end == p) break;              /* not a number: stop, never guess */
        if (n < max) out[n] = v;
        n++;
        p = end;
    }
    return n;
}

/* Read a single scalar number for `key`. Returns 0 on success. */
static int json_scalar(const char *json, const char *key, double *out) {
    const char *p = json_value(json, key);
    if (!p) return -1;
    char *end;
    double v = strtod(p, &end);
    if (end == p) return -1;
    *out = v;
    return 0;
}

/* Copy the string value at `p` into `out` (NUL-terminated). Returns 0 on success. */
static int json_string_at(const char *p, char *out, size_t cap) {
    if (!p) return -1;
    while (*p == ' ') p++;
    if (*p != '"') return -1;
    p++;
    const char *end = strchr(p, '"');
    if (!end || (size_t)(end - p) >= cap) return -1;
    memcpy(out, p, (size_t)(end - p));
    out[end - p] = '\0';
    return 0;
}

/* --------------------------------------------------------------- geometry */

/* URDF fixed-axis roll/pitch/yaw -> 3x3 rotation, i.e. Rz(yaw) Ry(pitch) Rx(roll). */
static void rpy_to_matrix(const double rpy[3], double m[3][3]) {
    double cr = cos(rpy[0]), sr = sin(rpy[0]);
    double cp = cos(rpy[1]), sp = sin(rpy[1]);
    double cy = cos(rpy[2]), sy = sin(rpy[2]);
    m[0][0] = cy * cp; m[0][1] = cy * sp * sr - sy * cr; m[0][2] = cy * sp * cr + sy * sr;
    m[1][0] = sy * cp; m[1][1] = sy * sp * sr + cy * cr; m[1][2] = sy * sp * cr - cy * sr;
    m[2][0] = -sp;     m[2][1] = cp * sr;                m[2][2] = cp * cr;
}

static void mat_vec(const double m[3][3], const double v[3], double out[3]) {
    for (int i = 0; i < 3; i++)
        out[i] = m[i][0] * v[0] + m[i][1] * v[1] + m[i][2] * v[2];
}

static void mat_mul(const double a[3][3], const double b[3][3], double out[3][3]) {
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            out[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
}

/* ------------------------------------------------------------ finger state */

/* Everything a finger's callback needs, all derived from that finger's format.
 * Touched only by that finger's single worker thread, so no atomics needed. */
typedef struct {
    const char *name;
    uint64_t    last_print_ms;
    int         point_count;
    int         stride;             /* bytes per point block */
    int         expect_len;         /* exact data length the contract demands */
    int         off[3];             /* byte offset of fx / fy / fz inside a point */
    double      scale[3];           /* raw integer * scale = physical value */
    double      contact;            /* |F| above this counts as contact, in `unit` */
    uint32_t    digest;             /* info revision this decoder was built from */
    int         placed;             /* mount pose present in this firmware */
    double      pos[MAX_POINTS][3];    /* point position in tip_sensor_frame, metres */
    double      axes[MAX_POINTS][3][3]; /* rotates that point's force into the same frame */
} finger_ctx_t;

static uint64_t now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000ULL + (uint64_t)ts.tv_nsec / 1000000ULL;
}

/* Parse the finger's format JSON into ctx. Returns 0 on success. */
static int parse_format(const char *json, finger_ctx_t *ctx) {
    double pc, stride, agg_stride;
    if (json_scalar(json, "point_count", &pc) ||
        json_scalar(json, "point_stride", &stride) ||
        json_scalar(json, "aggregate_stride", &agg_stride)) {
        fprintf(stderr, "[%s] format missing layout keys\n", ctx->name);
        return -1;
    }
    /* Everything below indexes into a device-supplied buffer, so range-check the
     * layout before trusting it: a corrupt or hostile format must not be able to
     * steer a read past the point block or the frame. */
    if (pc <= 0 || pc > MAX_POINTS || stride <= 0 || agg_stride < 0 ||
        stride > 4096 || agg_stride > 4096) {
        fprintf(stderr, "[%s] implausible layout: %g points, stride %g, aggregate %g\n",
                ctx->name, pc, stride, agg_stride);
        return -1;
    }
    ctx->point_count = (int)pc;
    ctx->stride      = (int)stride;
    ctx->expect_len  = ctx->point_count * ctx->stride + (int)agg_stride;

    /* point_fields is an array of objects, one per axis in fx, fy, fz order.
     * Every lookup below is bounded to that array, and each axis to its own
     * object: an unbounded search would walk into aggregate_fields and read
     * another field's offset, scale or unit. */
    const char *pf = json_value(json, "point_fields");
    const char *pf_end = pf ? json_value_end(pf) : NULL;
    if (!pf || *pf != '[' || !pf_end) {
        fprintf(stderr, "[%s] point_fields missing or not an array\n", ctx->name);
        return -1;
    }
    const char *cur = pf + 1;
    for (int i = 0; i < 3; i++) {
        while (cur < pf_end && *cur != '{') cur++;
        const char *obj_end = cur < pf_end ? json_value_end(cur) : NULL;
        if (!obj_end || obj_end > pf_end) {
            fprintf(stderr, "[%s] point_fields has fewer than 3 entries\n", ctx->name);
            return -1;
        }
        const char *o = json_find(cur, "offset", -1, obj_end);
        const char *s = json_find(cur, "scale", -1, obj_end);
        if (!o || !s) {
            fprintf(stderr, "[%s] point_fields[%d] has no offset/scale\n", ctx->name, i);
            return -1;
        }
        double off = strtod(o, NULL);
        /* Each axis is an i16 read at blk + off, so the whole read must land
         * inside the point block. */
        if (off < 0 || off + (double)sizeof(int16_t) > stride) {
            fprintf(stderr, "[%s] point_fields[%d] offset %g does not fit stride %g\n",
                    ctx->name, i, off, stride);
            return -1;
        }
        ctx->off[i]   = (int)off;
        ctx->scale[i] = strtod(s, NULL);
        cur = obj_end;
    }

    /* Contact threshold follows the unit the format declares. A unit on the
     * point fields themselves wins; otherwise the top-level default applies.
     * The point-field lookup is bounded to that array, because `aggregate_fields`
     * declares "N" and an unbounded search would pick it up instead. An unknown
     * unit is a hard stop — guessing would silently misreport contact. */
    char unit[32] = {0};
    const char *u = json_find(pf, "unit", -1, pf_end);
    if (!u) u = json_value(json, "unit");
    if (json_string_at(u, unit, sizeof unit) != 0) {
        fprintf(stderr, "[%s] format declares no unit\n", ctx->name);
        return -1;
    }
    ctx->contact = -1.0;
    for (size_t u = 0; u < sizeof CONTACT_BY_UNIT / sizeof CONTACT_BY_UNIT[0]; u++)
        if (strcmp(unit, CONTACT_BY_UNIT[u].unit) == 0) ctx->contact = CONTACT_BY_UNIT[u].threshold;
    if (ctx->contact < 0) {
        fprintf(stderr, "[%s] unsupported point unit \"%s\"\n", ctx->name, unit);
        return -1;
    }

    /* Three format generations are in the field, so the placement keys need a
     * state machine rather than an all-or-nothing test:
     *   - `positions` predates the mount pose and may be absent on its own;
     *   - `point_rpy` / `base_xyz` / `base_rpy` arrived together. All three
     *     absent means older firmware and only forces are available; all three
     *     present means the points can be placed; anything in between is a
     *     self-contradictory format and is rejected rather than half-used. */
    const char *mount_keys[3] = { "point_rpy", "base_xyz", "base_rpy" };
    int have = 0;
    for (int k = 0; k < 3; k++) have += json_value(json, mount_keys[k]) != NULL;
    if (have == 0) {
        ctx->placed = 0;
        return 0;
    }
    if (have != 3) {
        fprintf(stderr, "[%s] mount pose partially present (%d of 3 keys)\n", ctx->name, have);
        return -1;
    }

    double base_xyz[3], base_rpy[3], raw[MAX_POINTS * 3], rpy[MAX_POINTS * 3];
    if (json_numbers(json_value(json, "base_xyz"), base_xyz, 3) != 3 ||
        json_numbers(json_value(json, "base_rpy"), base_rpy, 3) != 3) {
        fprintf(stderr, "[%s] base_xyz / base_rpy are not 3 numbers each\n", ctx->name);
        return -1;
    }
    int npos = json_numbers(json_value(json, "positions"), raw, MAX_POINTS * 3);
    int nrpy = json_numbers(json_value(json, "point_rpy"), rpy, MAX_POINTS * 3);
    if (npos != ctx->point_count * 3 || nrpy != ctx->point_count * 3) {
        fprintf(stderr, "[%s] positions/point_rpy have %d/%d numbers, want %d each\n",
                ctx->name, npos, nrpy, ctx->point_count * 3);
        return -1;
    }

    double R[3][3];
    rpy_to_matrix(base_rpy, R);
    for (int k = 0; k < ctx->point_count; k++) {
        mat_vec(R, &raw[k * 3], ctx->pos[k]);
        for (int a = 0; a < 3; a++) ctx->pos[k][a] += base_xyz[a];
        /* Force axes of point k rotate by R(base_rpy) * R(point_rpy[k]). */
        double Rp[3][3];
        rpy_to_matrix(&rpy[k * 3], Rp);
        mat_mul(R, Rp, ctx->axes[k]);
    }
    ctx->placed = 1;
    return 0;
}

static void on_fingertip_data(WujiFrameKind kind, const WujiFingertipSensorData *f, void *ud) {
    finger_ctx_t *ctx = (finger_ctx_t *)ud;

    if (kind == WUJI_FRAME_KIND_LAG)   { fprintf(stderr, "[%s] lagged\n", ctx->name); return; }
    if (kind == WUJI_FRAME_KIND_END)   { fprintf(stderr, "[%s] stream ended\n", ctx->name); return; }
    if (kind == WUJI_FRAME_KIND_ERROR) { fprintf(stderr, "[%s] error: %s\n", ctx->name, wuji_last_error()); return; }
    if (kind != WUJI_FRAME_KIND_OK || !f) return;

    /* Throttle printing to ~1 Hz (the stream is ~100 Hz). */
    uint64_t now = now_ms();
    if (now - ctx->last_print_ms < 1000) return;
    ctx->last_print_ms = now;

    /* info_digest binds the frame to a specific info revision. Two revisions can
     * share a payload length while differing in scale, unit or mounting pose, so
     * check it before decoding — a length check alone would let this decoder keep
     * applying a stale format. A real application re-fetches info on mismatch;
     * this example drops the frame and says so. */
    if (f->info_digest != ctx->digest) {
        fprintf(stderr, "[%s] info_digest 0x%08x != 0x%08x from info; re-GET info\n",
                ctx->name, f->info_digest, ctx->digest);
        return;
    }

    /* The contract demands an exact length; a short frame is a fault, not a
     * partial read. */
    if ((int)f->data_len != ctx->expect_len) {
        fprintf(stderr, "[%s] data_len %zu != %d\n", ctx->name, f->data_len, ctx->expect_len);
        return;
    }

    /* Decode per the format and find the hardest-pressed point. */
    int peak = -1;
    double peak_mag = 0.0, peak_f[3] = {0};
    for (int k = 0; k < ctx->point_count; k++) {
        const uint8_t *blk = f->data + (size_t)k * ctx->stride;
        double c[3];
        for (int a = 0; a < 3; a++) {
            int16_t raw;
            memcpy(&raw, blk + ctx->off[a], sizeof raw);   /* i16 little-endian */
            c[a] = raw * ctx->scale[a];
        }
        double mag = sqrt(c[0] * c[0] + c[1] * c[1] + c[2] * c[2]);
        if (mag > peak_mag) {
            peak_mag = mag;
            peak = k;
            memcpy(peak_f, c, sizeof peak_f);
        }
    }

    if (peak < 0 || peak_mag <= ctx->contact) {
        printf("[%-6s] seq=%u  info_digest=0x%08x  no contact\n",
               ctx->name, f->header.seq, f->info_digest);
    } else if (ctx->placed) {
        double dir[3];
        mat_vec(ctx->axes[peak], peak_f, dir);   /* force axes into tip_sensor_frame */
        printf("[%-6s] seq=%u  info_digest=0x%08x  peak#%-2d |F|=%.3f  "
               "at (%+6.2f,%+6.2f,%+6.2f)mm  dir=(%+5.2f,%+5.2f,%+5.2f)\n",
               ctx->name, f->header.seq, f->info_digest, peak, peak_mag,
               ctx->pos[peak][0] * 1000, ctx->pos[peak][1] * 1000, ctx->pos[peak][2] * 1000,
               dir[0], dir[1], dir[2]);
    } else {
        printf("[%-6s] seq=%u  info_digest=0x%08x  peak#%-2d |F|=%.3f  (no mount pose)\n",
               ctx->name, f->header.seq, f->info_digest, peak, peak_mag);
    }
    fflush(stdout);
}

int main(void) {
    signal(SIGINT, on_sigint);
    printf("Wuji SDK version: %s\n", wuji_version());

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
    size_t idx = n;
    for (size_t i = 0; i < n; i++)
        if (idx == n && list[i].device_id == WUJI_DEVICE_TYPE_WUJI_HAND_2) idx = i;
    if (idx == n) {
        printf("No Wuji Hand 2 found\n");
        wuji_discovered_free(list, n);
        wuji_shutdown();
        return 0;
    }
    printf("Connecting to %s (%s)\n", list[idx].serial_number, list[idx].address);

    WujiConnectTarget tgt = { .kind = WUJI_CONNECT_TARGET_KIND_SN, .value = list[idx].serial_number };
    WujiConnectOptions connect_opts = wuji_connect_options_default();
    connect_opts.timeout_ms = 1000;
    connect_opts.retry_count = 3;
    connect_opts.enable_bridge = true;
    struct WujiDevice *dev = NULL;
    WujiStatus st = wuji_connect(&tgt, "hand2", &connect_opts, &dev);
    wuji_discovered_free(list, n);
    if (st != WUJI_STATUS_OK) {
        fprintf(stderr, "wuji_connect failed: %s\n", wuji_last_error());
        wuji_shutdown();
        return 1;
    }

    /* Read each finger's self-describing info (high-level; chunked read hidden)
     * and derive its decoder plus where its points sit on the hand. */
    const char *names[5] = { "thumb", "index", "middle", "ring", "pinky" };
    finger_ctx_t ctxs[5];
    memset(ctxs, 0, sizeof ctxs);
    for (uint8_t i = 0; i < 5; i++) {
        WujiFingertipSensorInfo info;
        if (wuji_hand_2_get_fingertip_info(dev, i, &info) != WUJI_STATUS_OK) {
            fprintf(stderr, "get_fingertip_info(%s): %s\n", names[i], wuji_last_error());
            goto cleanup;
        }
        ctxs[i].name = names[i];
        /* Remember which info revision this decoder came from; every data frame
         * carries the same value and is rejected if it drifts. */
        ctxs[i].digest = info.digest;
        if (parse_format(info.format, &ctxs[i]) != 0) goto cleanup;
        printf("  %-6s frame_id=%s  device_type=0x%04x  rate=%.0fHz  digest=0x%08x  %d points  %s\n",
               names[i], info.header.frame_id, info.device_type, info.rate_hz, info.digest,
               ctxs[i].point_count,
               ctxs[i].placed ? "placed on the hand model"
                              : "no mount pose in this firmware");
        if (ctxs[i].placed)
            printf("         point 0 at (%+6.2f,%+6.2f,%+6.2f)mm in %s_tip_sensor_frame\n",
                   ctxs[i].pos[0][0] * 1000, ctxs[i].pos[0][1] * 1000,
                   ctxs[i].pos[0][2] * 1000, names[i]);
    }

    /* Subscribe to the five per-finger typed data streams. */
    struct WujiSub *subs[5] = { 0 };
    st  = wuji_hand_2_subscribe_fingertip_thumb_data(dev, on_fingertip_data, &ctxs[0], &subs[0]);
    st |= wuji_hand_2_subscribe_fingertip_index_data(dev, on_fingertip_data, &ctxs[1], &subs[1]);
    st |= wuji_hand_2_subscribe_fingertip_middle_data(dev, on_fingertip_data, &ctxs[2], &subs[2]);
    st |= wuji_hand_2_subscribe_fingertip_ring_data(dev, on_fingertip_data, &ctxs[3], &subs[3]);
    st |= wuji_hand_2_subscribe_fingertip_pinky_data(dev, on_fingertip_data, &ctxs[4], &subs[4]);
    if (st != WUJI_STATUS_OK) {
        fprintf(stderr, "subscribe fingertip data: %s\n", wuji_last_error());
        goto cleanup_subs;
    }
    printf("Subscribed to 5 fingertip data streams. Ctrl+C to stop.\n");
    fflush(stdout);

    while (!g_stop) sleep(1);
    printf("\nStopping...\n");

cleanup_subs:
    for (int i = 0; i < 5; i++)
        if (subs[i]) wuji_sub_close(subs[i]); /* stop workers before releasing the device */
cleanup:
    wuji_dev_disconnect(dev);
    wuji_dev_release(dev);
    wuji_shutdown();
    return 0;
}
