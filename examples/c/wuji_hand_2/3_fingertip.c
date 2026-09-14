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
 *     R(base_rpy) * positions[i] + base_xyz, and `aggregate_xyz` marks where the
 *     aggregate force acts and `aggregate_rpy` orients its axes, both in that same
 *     sensor frame, so the resultant becomes a wrench rather than a bare force.
 *     Firmware without `aggregate_xyz` reports no point and no moment; without
 *     `aggregate_rpy` both still appear, with the force axes taken from the mounting
 *     pose. The hand reports the pose for whichever hand it is, so this works
 *     unchanged on a left or a right hand.
 *   - subscribes to the five per-finger data streams (~100 Hz,
 *     WujiFingertipSensorData) and refreshes an in-place display at 100 Hz: one
 *     status line per finger (aggregate force, temperature, contact count, and
 *     the hardest-pressed point with where it sits on the fingertip) above a
 *     table of every point's fx / fy / fz — 40 points for thumb, 34 for the
 *     other fingers.
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
 * `data` buffer is freed right after the callback returns) — never stash it. The
 * table needs all five fingers at once, so each callback copies its payload into
 * C-owned storage under a lock and the main thread renders from those copies.
 */
#include <pthread.h>
#include <stdio.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <unistd.h>
#include <math.h>

#include "wuji_sdk.h"

#define FINGER_COUNT 5
#define MAX_POINTS 40  /* thumb has the most; standard fingers have 34 */
#define MAX_FRAME_BYTES 2048  /* per-finger payload copy; layouts above this are refused */
#define REFRESH_US 10000      /* 100 Hz redraw, matching the stream rate */
#define CELL_W 18             /* nominal cell width; cells only ever grow, never clip */

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

/* Read the finite number at `p`. Returns 0 on success. Plain strtod would accept
 * "nan" and stop at trailing junk ("1abc" reads as 1), so require a JSON delimiter
 * after the number and test the range positively, which drops NaN. */
static int number_at(const char *p, double *out) {
    if (!p) return -1;
    char *end;
    double v = strtod(p, &end);
    if (end == p) return -1;
    while (*end == ' ' || *end == '\t' || *end == '\n' || *end == '\r') end++;
    if (*end && !strchr(",}]", *end)) return -1;
    if (!(v > -HUGE_VAL && v < HUGE_VAL)) return -1;
    *out = v;
    return 0;
}

/* Read a whole number in [0, max] at `p` as an int. Returns 0 on success. Every
 * value that indexes a device-supplied buffer goes through here. */
static int whole_at(const char *p, double max, int *out) {
    double v;
    if (number_at(p, &v) != 0) return -1;
    if (!(v >= 0.0 && v <= max)) return -1;
    if (v != (double)(long)v) return -1;
    *out = (int)v;
    return 0;
}

/* Read a whole number for a key on the format's outermost object. */
static int json_whole(const char *json, const char *key, double max, int *out) {
    return whole_at(json_value(json, key), max, out);
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

/* Read the offset and scale of the field called `name` from an array of field
 * objects. Every lookup is bounded to the matching object, so a field that does
 * not declare one of them cannot pick up the next entry's value. The i16 read at
 * that offset must land inside `stride`. Returns 0 on success, -1 if the field is
 * absent or malformed. Every field this contract declares is required. */
static int field_lookup(const char *arr, const char *arr_end, const char *name,
                        int stride, int *off, double *scale) {
    if (!arr || *arr != '[' || !arr_end) return -1;
    for (const char *cur = arr + 1; cur < arr_end; ) {
        while (cur < arr_end && *cur != '{') cur++;
        const char *obj_end = cur < arr_end ? json_value_end(cur) : NULL;
        if (!obj_end || obj_end > arr_end) return -1;
        char have[32] = {0};
        if (json_string_at(json_find(cur, "name", -1, obj_end), have, sizeof have) == 0 &&
            strcmp(have, name) == 0) {
            if (whole_at(json_find(cur, "offset", -1, obj_end),
                         (double)stride - (double)sizeof(int16_t), off) != 0) return -1;
            /* A non-finite scale would turn every decoded value into a nan. */
            if (number_at(json_find(cur, "scale", -1, obj_end), scale) != 0) return -1;
            return 0;
        }
        cur = obj_end;
    }
    return -1;
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

/* Why a frame is not being shown. Recorded by the callback and rendered on the
 * finger's status line: printing straight from the callback would scribble over
 * the table the main thread is redrawing. */
typedef enum {
    FINGER_WAITING = 0,
    FINGER_OK,
    FINGER_LAGGED,
    FINGER_ENDED,
    FINGER_ERRORED,
    FINGER_INFO_CHANGED,
    FINGER_BAD_LENGTH,
} finger_state_t;

/* Everything a finger's callback needs, all derived from that finger's format,
 * plus the latest payload for the main thread to render. */
typedef struct {
    const char *name;
    int         point_count;
    int         stride;             /* bytes per point block */
    int         expect_len;         /* exact data length the contract demands */
    int         off[3];             /* byte offset of fx / fy / fz inside a point */
    double      scale[3];           /* raw integer * scale = physical value */
    int         agg_off[4];         /* aggregate fx / fy / fz / temperature offsets */
    double      agg_scale[4];
    double      contact;            /* |F| above this counts as contact, in `unit` */
    uint32_t    digest;             /* info revision this decoder was built from */
    int         placed;             /* mount pose present in this firmware */
    double      pos[MAX_POINTS][3];    /* point position in tip_sensor_frame, metres */
    double      axes[MAX_POINTS][3][3]; /* rotates that point's force into the same frame */
    int         has_force_point;    /* format declares where the aggregate force acts */
    double      force_point[3];     /* that point in tip_sensor_frame, metres */
    double      force_R[3][3];      /* rotates the aggregate force into tip_sensor_frame */

    /* Written by this finger's worker thread, read by the renderer. */
    pthread_mutex_t lock;
    int             lock_ready;
    finger_state_t  state;
    uint8_t         data[MAX_FRAME_BYTES];
    int             data_len;
    uint32_t        seq;
    uint32_t        seen_digest;    /* the digest that did not match, for the message */
} finger_ctx_t;

/* Indices into agg_off / agg_scale. */
enum { AGG_FX = 0, AGG_FY, AGG_FZ, AGG_TEMP };

/* Parse the finger's format JSON into ctx. Returns 0 on success. */
static int parse_format(const char *json, finger_ctx_t *ctx) {
    /* Everything below indexes into a device-supplied buffer, so range-check the
     * layout before trusting it: a corrupt or hostile format must not be able to
     * steer a read past the point block or the frame. whole_at() also refuses
     * anything that is not a plain whole number, so no NaN and no fraction can
     * reach the ints the rest of this function indexes with. */
    int pc = 0, stride = 0, agg_stride = 0;
    if (json_whole(json, "point_count", MAX_POINTS, &pc) ||
        json_whole(json, "point_stride", 4096, &stride) ||
        json_whole(json, "aggregate_stride", 4096, &agg_stride) ||
        pc == 0 || stride == 0) {
        fprintf(stderr, "[%s] missing or implausible layout keys\n", ctx->name);
        return -1;
    }
    ctx->point_count = pc;
    ctx->stride      = stride;
    ctx->expect_len  = pc * stride + agg_stride;
    /* Each frame is copied into a fixed per-finger buffer so the renderer can read
     * all five at once. A layout the buffer cannot hold would make every frame
     * unusable, so refuse it here with the size rather than sitting on "waiting". */
    if (ctx->expect_len > MAX_FRAME_BYTES) {
        fprintf(stderr, "[%s] layout needs %dB, buffer holds %d; raise MAX_FRAME_BYTES\n",
                ctx->name, ctx->expect_len, MAX_FRAME_BYTES);
        return -1;
    }

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
        /* Each axis is an i16 read at blk + off, so the whole read must land
         * inside the point block. */
        if (whole_at(o, (double)stride - (double)sizeof(int16_t), &ctx->off[i]) != 0) {
            fprintf(stderr, "[%s] point_fields[%d] offset does not fit stride %d\n",
                    ctx->name, i, stride);
            return -1;
        }
        /* A non-finite scale would turn every decoded value into a nan. */
        if (number_at(s, &ctx->scale[i]) != 0) {
            fprintf(stderr, "[%s] point_fields[%d] has an unusable scale\n", ctx->name, i);
            return -1;
        }
        cur = obj_end;
    }

    /* The aggregate block sits after the point array and carries the resultant
     * force and the sensor temperature, both required by the contract. Same i16
     * assumption as the point fields; offsets are bounded to the block. */
    const char *af = json_value(json, "aggregate_fields");
    const char *af_end = af ? json_value_end(af) : NULL;
    const char *agg_names[3] = { "fx", "fy", "fz" };
    for (int i = 0; i < 3; i++) {
        if (field_lookup(af, af_end, agg_names[i], agg_stride,
                         &ctx->agg_off[i], &ctx->agg_scale[i]) != 0) {
            fprintf(stderr, "[%s] aggregate_fields has no usable %s\n", ctx->name, agg_names[i]);
            return -1;
        }
    }
    if (field_lookup(af, af_end, "temperature", agg_stride,
                     &ctx->agg_off[AGG_TEMP], &ctx->agg_scale[AGG_TEMP]) != 0) {
        fprintf(stderr, "[%s] aggregate_fields has no temperature\n", ctx->name);
        return -1;
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
    memcpy(ctx->force_R, R, sizeof R);
    ctx->placed = 1;

    /* `aggregate_xyz` arrived a generation after the mount pose: where the resultant in
     * `aggregate_fields` acts, in the same frame as `positions`. Absent means older
     * firmware and the resultant has no declared point; present means force plus point
     * is a wrench. Nothing is guessed in between. */
    /* A missing key and a malformed one are different failures and must not collapse into
     * one silent path: absent means older firmware, present-but-not-three-numbers is a
     * broken format and is rejected, same split the mount block above uses. */
    double agg_xyz[3], agg_rpy[3];
    const char *agg_xyz_val = json_value(json, "aggregate_xyz");
    if (agg_xyz_val) {
        if (json_numbers(agg_xyz_val, agg_xyz, 3) != 3) {
            fprintf(stderr, "[%s] aggregate_xyz is not 3 numbers\n", ctx->name);
            return -1;
        }
        mat_vec(R, agg_xyz, ctx->force_point);
        for (int a = 0; a < 3; a++) ctx->force_point[a] += base_xyz[a];
        ctx->has_force_point = 1;
    }
    /* `aggregate_rpy` orients the resultant's own axes, same convention as point_rpy.
     * Without it the axes are undeclared and the sensor frame is the only sane reading,
     * which is what force_R already holds. Validated independently of aggregate_xyz. */
    const char *agg_rpy_val = json_value(json, "aggregate_rpy");
    if (agg_rpy_val) {
        if (json_numbers(agg_rpy_val, agg_rpy, 3) != 3) {
            fprintf(stderr, "[%s] aggregate_rpy is not 3 numbers\n", ctx->name);
            return -1;
        }
        double Ra[3][3], Rf[3][3];
        rpy_to_matrix(agg_rpy, Ra);
        mat_mul(R, Ra, Rf);
        memcpy(ctx->force_R, Rf, sizeof Rf);
    }
    return 0;
}

/* Record why this finger has nothing to show. */
static void set_state(finger_ctx_t *ctx, finger_state_t state) {
    pthread_mutex_lock(&ctx->lock);
    ctx->state = state;
    pthread_mutex_unlock(&ctx->lock);
}

static void on_fingertip_data(WujiFrameKind kind, const WujiFingertipSensorData *f, void *ud) {
    finger_ctx_t *ctx = (finger_ctx_t *)ud;

    if (kind == WUJI_FRAME_KIND_LAG)   { set_state(ctx, FINGER_LAGGED);  return; }
    if (kind == WUJI_FRAME_KIND_END)   { set_state(ctx, FINGER_ENDED);   return; }
    if (kind == WUJI_FRAME_KIND_ERROR) { set_state(ctx, FINGER_ERRORED); return; }
    if (kind != WUJI_FRAME_KIND_OK || !f || !f->data) return;

    /* info_digest binds the frame to a specific info revision. Two revisions can
     * share a payload length while differing in scale, unit or mounting pose, so
     * check it before decoding — a length check alone would let this decoder keep
     * applying a stale format. A real application re-fetches info on mismatch;
     * this example drops the frame and says so on the status line. */
    if (f->info_digest != ctx->digest) {
        pthread_mutex_lock(&ctx->lock);
        ctx->state = FINGER_INFO_CHANGED;
        ctx->seen_digest = f->info_digest;
        pthread_mutex_unlock(&ctx->lock);
        return;
    }

    /* The contract demands an exact length; a short frame is a fault, not a
     * partial read. */
    if ((int)f->data_len != ctx->expect_len) {
        set_state(ctx, FINGER_BAD_LENGTH);
        return;
    }

    /* Copy the payload out before returning: the frame and its heap buffer die
     * with this call, and the renderer needs all five fingers together. */
    pthread_mutex_lock(&ctx->lock);
    memcpy(ctx->data, f->data, f->data_len);
    ctx->data_len = (int)f->data_len;
    ctx->seq      = f->header.seq;
    ctx->state    = FINGER_OK;
    pthread_mutex_unlock(&ctx->lock);
}

/* ----------------------------------------------------------------- display */

/* One decoded frame, ready to render. */
typedef struct {
    finger_state_t state;
    uint32_t       seq;
    uint32_t       seen_digest;
    int            point_count;
    double         pts[MAX_POINTS][3];   /* per-point fx / fy / fz */
    double         agg[3];               /* aggregate resultant force */
    double         temp;
    int            contacts;
    int            peak;                 /* hardest-pressed point, -1 if none */
    double         peak_mag;
    double         peak_dir[3];          /* peak force in tip_sensor_frame */
} snapshot_t;

/* Read a scaled i16 at `base + off`. The offsets were bounded against their
 * block at parse time, so the read cannot leave the payload. */
static double read_i16(const uint8_t *data, int base, int off, double scale) {
    int16_t raw;
    memcpy(&raw, data + base + off, sizeof raw);   /* little-endian on the wire */
    return raw * scale;
}

/* Take this finger's latest frame and decode it. */
static void snapshot(finger_ctx_t *ctx, snapshot_t *s) {
    uint8_t data[MAX_FRAME_BYTES];
    int len = 0;

    pthread_mutex_lock(&ctx->lock);
    s->state       = ctx->state;
    s->seq         = ctx->seq;
    s->seen_digest = ctx->seen_digest;
    if (ctx->state == FINGER_OK) {
        len = ctx->data_len;
        memcpy(data, ctx->data, (size_t)len);
    }
    pthread_mutex_unlock(&ctx->lock);

    s->point_count = ctx->point_count;
    s->peak        = -1;
    s->peak_mag    = 0.0;
    s->contacts    = 0;
    if (s->state != FINGER_OK || len != ctx->expect_len) {
        if (s->state == FINGER_OK) s->state = FINGER_WAITING;
        return;
    }

    for (int k = 0; k < ctx->point_count; k++) {
        int blk = k * ctx->stride;
        for (int a = 0; a < 3; a++)
            s->pts[k][a] = read_i16(data, blk, ctx->off[a], ctx->scale[a]);
        double mag = sqrt(s->pts[k][0] * s->pts[k][0] + s->pts[k][1] * s->pts[k][1] +
                          s->pts[k][2] * s->pts[k][2]);
        if (mag > ctx->contact) s->contacts++;
        if (mag > s->peak_mag) { s->peak_mag = mag; s->peak = k; }
    }

    int agg_base = ctx->point_count * ctx->stride;
    for (int a = 0; a < 3; a++)
        s->agg[a] = read_i16(data, agg_base, ctx->agg_off[a], ctx->agg_scale[a]);
    s->temp = read_i16(data, agg_base, ctx->agg_off[AGG_TEMP], ctx->agg_scale[AGG_TEMP]);

    /* Rotate the peak point's force into tip_sensor_frame, where its position is. */
    if (s->peak >= 0 && ctx->placed)
        mat_vec(ctx->axes[s->peak], s->pts[s->peak], s->peak_dir);
}

static const char *state_text(const snapshot_t *s) {
    switch (s->state) {
        case FINGER_LAGGED:       return "lagged, waiting for the next frame";
        case FINGER_ENDED:        return "stream ended";
        case FINGER_ERRORED:      return "stream error";
        case FINGER_INFO_CHANGED: return "info changed; re-GET info";
        case FINGER_BAD_LENGTH:   return "unexpected payload length";
        default:                  return "waiting for data...";
    }
}

/* Per-finger status line: aggregate force, temperature, contact count and the
 * hardest-pressed point with where it sits on the fingertip. */
static void status_line(finger_ctx_t *ctx, const snapshot_t *s) {
    if (s->state != FINGER_OK) {
        if (s->state == FINGER_INFO_CHANGED)
            printf("\033[2K%-7s info changed (0x%08x != 0x%08x); re-GET info\n",
                   ctx->name, s->seen_digest, ctx->digest);
        else
            printf("\033[2K%-7s %s\n", ctx->name, state_text(s));
        return;
    }
    printf("\033[2K%-7s %2dpt  seq=%-6u", ctx->name, s->point_count, s->seq);
    printf("  temp=%5.1fC", s->temp);
    printf("  force=(%+6.2f,%+6.2f,%+6.2f)N  contacts=%2d",
           s->agg[0], s->agg[1], s->agg[2], s->contacts);
    /* Resultant plus its application point is a wrench: rotate the force into
     * tip_sensor_frame and take p x f about that frame's origin, in mN*m. */
    if (ctx->has_force_point) {
        double f[3];
        mat_vec(ctx->force_R, s->agg, f);
        const double *pt = ctx->force_point;
        printf("  moment=(%+6.1f,%+6.1f,%+6.1f)mNm",
               (pt[1] * f[2] - pt[2] * f[1]) * 1000.0,
               (pt[2] * f[0] - pt[0] * f[2]) * 1000.0,
               (pt[0] * f[1] - pt[1] * f[0]) * 1000.0);
    }
    if (s->peak >= 0 && s->peak_mag > ctx->contact) {
        printf("  peak#%-2d |F|=%.3f", s->peak, s->peak_mag);
        if (ctx->placed)
            printf(" at (%+6.2f,%+6.2f,%+6.2f)mm dir=(%+5.2f,%+5.2f,%+5.2f)",
                   ctx->pos[s->peak][0] * 1000, ctx->pos[s->peak][1] * 1000,
                   ctx->pos[s->peak][2] * 1000,
                   s->peak_dir[0], s->peak_dir[1], s->peak_dir[2]);
    } else {
        printf("  no contact");
    }
    printf("\n");
}

/* Status lines for all five fingers, then every point's force components. */
static void render(finger_ctx_t ctxs[FINGER_COUNT], int max_points) {
    snapshot_t snaps[FINGER_COUNT];
    for (int f = 0; f < FINGER_COUNT; f++) {
        snapshot(&ctxs[f], &snaps[f]);
        status_line(&ctxs[f], &snaps[f]);
    }

    printf("\033[2Kpt   | ");
    for (int f = 0; f < FINGER_COUNT; f++)
        printf("%-*s%s", CELL_W, ctxs[f].name, f == FINGER_COUNT - 1 ? "\n" : " | ");
    printf("\033[2K-----+");
    for (int f = 0; f < FINGER_COUNT; f++) {
        for (int j = 0; j < CELL_W + 2; j++) putchar('-');
        printf("%s", f == FINGER_COUNT - 1 ? "\n" : "+");
    }

    for (int i = 0; i < max_points; i++) {
        printf("\033[2Kp%02d  | ", i);
        for (int f = 0; f < FINGER_COUNT; f++) {
            /* Roomier than CELL_W on purpose: printf pads a short cell out to the
             * column but never trims a long one, so a force too wide for the
             * column nudges the row instead of losing a digit. */
            char cell[48] = "";   /* stays empty past this finger's point count */
            if (i < snaps[f].point_count) {
                if (snaps[f].state != FINGER_OK)
                    snprintf(cell, sizeof cell, "waiting");
                else
                    snprintf(cell, sizeof cell, "%+.1f,%+.1f,%+.1f",
                             snaps[f].pts[i][0], snaps[f].pts[i][1], snaps[f].pts[i][2]);
            }
            printf("%-*s%s", CELL_W, cell, f == FINGER_COUNT - 1 ? "\n" : " | ");
        }
    }
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
    const char *names[FINGER_COUNT] = { "thumb", "index", "middle", "ring", "pinky" };
    finger_ctx_t ctxs[FINGER_COUNT];
    int max_points = 0;
    memset(ctxs, 0, sizeof ctxs);
    for (uint8_t i = 0; i < FINGER_COUNT; i++) {
        WujiFingertipSensorInfo info;
        if (wuji_hand_2_get_fingertip_info(dev, i, &info) != WUJI_STATUS_OK) {
            fprintf(stderr, "get_fingertip_info(%s): %s\n", names[i], wuji_last_error());
            goto cleanup;
        }
        ctxs[i].name = names[i];
        pthread_mutex_init(&ctxs[i].lock, NULL);
        ctxs[i].lock_ready = 1;
        /* Remember which info revision this decoder came from; every data frame
         * carries the same value and is rejected if it drifts. */
        ctxs[i].digest = info.digest;
        if (parse_format(info.format, &ctxs[i]) != 0) goto cleanup;
        if (ctxs[i].point_count > max_points) max_points = ctxs[i].point_count;
        printf("  %-6s frame_id=%s  device_type=0x%04x  rate=%.0fHz  digest=0x%08x  %d points  %s\n",
               names[i], info.header.frame_id, info.device_type, info.rate_hz, info.digest,
               ctxs[i].point_count,
               ctxs[i].placed ? "placed on the hand model"
                              : "no mount pose in this firmware");
        if (ctxs[i].placed)
            printf("         point 0 at (%+6.2f,%+6.2f,%+6.2f)mm in %s_tip_sensor_frame\n",
                   ctxs[i].pos[0][0] * 1000, ctxs[i].pos[0][1] * 1000,
                   ctxs[i].pos[0][2] * 1000, names[i]);
        if (ctxs[i].has_force_point)
            printf("         aggregate force acts at (%+6.2f,%+6.2f,%+6.2f)mm\n",
                   ctxs[i].force_point[0] * 1000, ctxs[i].force_point[1] * 1000,
                   ctxs[i].force_point[2] * 1000);
    }

    /* Subscribe to the five per-finger typed data streams. */
    struct WujiSub *subs[FINGER_COUNT] = { 0 };
    st  = wuji_hand_2_subscribe_fingertip_thumb_data(dev, on_fingertip_data, &ctxs[0], &subs[0]);
    st |= wuji_hand_2_subscribe_fingertip_index_data(dev, on_fingertip_data, &ctxs[1], &subs[1]);
    st |= wuji_hand_2_subscribe_fingertip_middle_data(dev, on_fingertip_data, &ctxs[2], &subs[2]);
    st |= wuji_hand_2_subscribe_fingertip_ring_data(dev, on_fingertip_data, &ctxs[3], &subs[3]);
    st |= wuji_hand_2_subscribe_fingertip_pinky_data(dev, on_fingertip_data, &ctxs[4], &subs[4]);
    if (st != WUJI_STATUS_OK) {
        fprintf(stderr, "subscribe fingertip data: %s\n", wuji_last_error());
        goto cleanup_subs;
    }
    printf("\nSubscribed to 5 fingertip data streams. Ctrl+C to stop.\n\n");
    fflush(stdout);

    /* Redraw in place: five status lines, the table header, then one row per
     * point. Every line clears itself, so a shorter line never leaves a tail. */
    int rows = FINGER_COUNT + 2 + max_points;
    int drawn = 0;
    while (!g_stop) {
        if (drawn) printf("\033[%dA", rows);
        render(ctxs, max_points);
        fflush(stdout);
        drawn = 1;
        usleep(REFRESH_US);
    }
    printf("\nStopping...\n");

cleanup_subs:
    for (int i = 0; i < FINGER_COUNT; i++)
        if (subs[i]) wuji_sub_close(subs[i]); /* stop workers before releasing the device */
cleanup:
    for (int i = 0; i < FINGER_COUNT; i++)
        if (ctxs[i].lock_ready) pthread_mutex_destroy(&ctxs[i].lock);
    wuji_dev_disconnect(dev);
    wuji_dev_release(dev);
    wuji_shutdown();
    return 0;
}
