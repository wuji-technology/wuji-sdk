#!/usr/bin/env python3
"""Fingertip sensor reader — Wuji Hand 2.

The fingertip sensor is self-describing: first GET each finger's info metadata
(carrying a format JSON that describes the data-frame layout), then subscribe to
fingertip/<finger>/data and decode each frame per the format — the layout is
never hardcoded, it all comes from the info contract.
Thumb has 40 points, the other fingers 34, streamed at 100 Hz.

The format carries the unit as well as the layout, so the contact threshold is
picked from the declared unit rather than assumed: firmware v2.4.0 and later
reports per-point force normalized to full scale (normal axis 0..1, tangential
-1..1), earlier firmware reports newtons. Aggregate force stays in newtons and
temperature in Celsius on both.

The format also places the sensor on the hand model. `positions` and `point_rpy`
are in the sensor module's own base frame; `base_xyz` and `base_rpy` give that
frame's pose relative to the finger's `tip_sensor_frame` link, so a contact point
lands at `R(base_rpy) @ positions[i] + base_xyz` and its force axes rotate by
`R(base_rpy) @ R(point_rpy[i])`. The hand reports the pose for whichever hand it
is, so the same code works on a left or a right hand.

The aggregate force comes with the point it acts on and the axes it is measured
along: `aggregate_xyz` is the sensor pad's centroid in that same sensor frame and
`aggregate_rpy` orients its axes, so the resultant becomes a real wrench — force
`R(base_rpy) @ R(aggregate_rpy) @ (fx, fy, fz)` applied at
`R(base_rpy) @ aggregate_xyz + base_xyz`, and a moment about the link origin of
`p x f`. Firmware without `aggregate_xyz` reports no point and no moment; without
`aggregate_rpy` both still appear, with the force axes taken from the mounting pose.

Redrawn in place at 100 Hz: one status line per finger — aggregate force,
temperature, contact count and where on the fingertip the strongest contact sits
— above a table of every point's fx / fy / fz. Each cycle drains the
subscription queue and shows the newest frame.

Usage: python 3.fingertip_typed.py
"""
import json
import math
import struct
import sys
import time

import wuji_sdk
from wuji_sdk import DeviceType, SdkManager

FINGERS = ["thumb", "index", "middle", "ring", "pinky"]
# "in contact" threshold per declared point unit; never assume one of them.
# normalized -> share of full scale; N -> newtons.
CONTACT_BY_UNIT = {"normalized": 0.02, "N": 0.2}
DURATION = 15            # seconds to run
REFRESH_S = 0.01         # 100 Hz display refresh, matching the stream rate
CELL_W = 18              # nominal cell width; cells only ever grow, never clip
# contract field type -> struct format char; unknown types raise, never guess
FIELD_FMT = {"i8": "<b", "u8": "<B", "i16": "<h", "u16": "<H",
             "i32": "<i", "u32": "<I", "f32": "<f"}
# The mounting pose arrived as one group, after `positions` was already shipping.
# All three absent means older firmware; a partial set is a broken format.
MOUNT_KEYS = ("point_rpy", "base_xyz", "base_rpy")


def rpy_to_matrix(rpy):
    """URDF fixed-axis roll/pitch/yaw -> 3x3 rotation, i.e. Rz(yaw) Ry(pitch) Rx(roll)."""
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p),
                              math.sin(p), math.cos(y), math.sin(y))
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp,     cp * sr,                cp * cr),
    )


def mat_vec(m, v):
    return tuple(sum(m[i][k] * v[k] for k in range(3)) for i in range(3))


def mat_mul(a, b):
    return tuple(tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
                 for i in range(3))


def fetch_format(hand, finger):
    """GET the finger's info and return its format JSON (the data-frame layout).

    The SDK drives the chunked info read internally and hands back a decoded
    FingertipSensorInfo; we only read its `format` string.
    """
    info = hand.get_fingertip_info(finger)
    fmt = json.loads(info.format)
    if fmt["v"] != 1 or fmt["encoding"] != "point_array":
        raise ValueError(f"unsupported format: {fmt.get('encoding')}")
    # fx/fy/fz and temperature are all required; name what is missing here rather
    # than dying on a KeyError mid-stream.
    have = {f["name"] for f in fmt["aggregate_fields"]}
    missing = {"fx", "fy", "fz", "temperature"} - have
    if missing:
        raise ValueError(f"aggregate_fields is missing {sorted(missing)}")
    return fmt, info.digest


def make_decoder(fmt):
    """Build a decoder from the format: data bytes -> (per-point dicts, aggregate dict)."""
    pc, stride = fmt["point_count"], fmt["point_stride"]
    expect = pc * stride + fmt["aggregate_stride"]

    def read(defs, data, base):
        return {d["name"]: struct.unpack_from(FIELD_FMT[d["type"]], data, base + d["offset"])[0]
                * d.get("scale", 1.0) for d in defs}

    def decode(data):
        if len(data) != expect:
            raise ValueError(f"data length {len(data)} != expected {expect}")
        points = [read(fmt["point_fields"], data, k * stride) for k in range(pc)]
        return points, read(fmt["aggregate_fields"], data, pc * stride)

    return decode


def make_placement(fmt):
    """Place the point cloud on the hand model, or return None on older firmware.

    Returns (positions, axes, force_point) in the finger's `tip_sensor_frame`:
    positions[i] is where point i sits in metres, axes[i] rotates that point's
    (fx, fy, fz) into the same frame, and force_point is where the aggregate
    force acts — None on firmware that predates `aggregate_xyz`, since guessing
    a point there would quietly fabricate a moment arm.

    All three mounting-pose keys absent means older firmware — positions are
    simply unavailable. A partial set is a self-contradictory format and raises,
    because silently degrading there would hide broken metadata.
    """
    present = [k for k in MOUNT_KEYS if k in fmt]
    if not present:
        return None
    if len(present) != len(MOUNT_KEYS):
        missing = [k for k in MOUNT_KEYS if k not in fmt]
        raise ValueError(f"mount pose partially present, missing {missing}")
    pc = fmt["point_count"]
    for key in ("positions", "point_rpy"):
        rows = fmt.get(key)
        if not isinstance(rows, list) or len(rows) != pc:
            raise ValueError(f"{key} must be {pc} rows, got {len(rows) if rows else rows}")
        if any(not isinstance(r, list) or len(r) != 3 for r in rows):
            raise ValueError(f"{key} rows must be 3 numbers each")
    base_r = rpy_to_matrix(fmt["base_rpy"])
    base_t = fmt["base_xyz"]
    positions = [tuple(c + base_t[k] for k, c in enumerate(mat_vec(base_r, p)))
                 for p in fmt["positions"]]
    axes = [mat_mul(base_r, rpy_to_matrix(rpy)) for rpy in fmt["point_rpy"]]
    # Where the aggregate force acts. Same frame as `positions`, same transform.
    # Presence is decided by the key, not by its value: a key written as null is a broken
    # format, not an older firmware, and must not degrade silently.
    if "aggregate_xyz" not in fmt:
        force_point = None
    else:
        agg = fmt["aggregate_xyz"]
        if not isinstance(agg, list) or len(agg) != 3:
            raise ValueError("aggregate_xyz must be 3 numbers")
        force_point = tuple(c + base_t[k] for k, c in enumerate(mat_vec(base_r, agg)))
    return positions, axes, force_point


def point_unit(fmt):
    """Effective unit of the per-point fields: field-level `unit` overrides the
    top-level default. Mixed units would make a single threshold meaningless, so
    refuse instead of guessing."""
    units = {f.get("unit", fmt.get("unit", "")) for f in fmt["point_fields"]}
    if len(units) != 1:
        raise ValueError(f"point_fields declare mixed units {sorted(units)}")
    return units.pop()


def contact_threshold(unit):
    """Threshold for |F| in the unit the format declares; unknown units abort."""
    if unit not in CONTACT_BY_UNIT:
        raise ValueError(
            f"unsupported point unit {unit!r}; known units: {sorted(CONTACT_BY_UNIT)}")
    return CONTACT_BY_UNIT[unit]


def force(p):
    """Magnitude of a point's force |F| = sqrt(fx^2 + fy^2 + fz^2), in the format's unit."""
    return math.sqrt(p["fx"] ** 2 + p["fy"] ** 2 + p["fz"] ** 2)


def point_cell(points, count, i):
    """One table cell: point i of a finger, padded out to CELL_W so the separators
    stay aligned. A force too wide for the column nudges the row rather than
    losing a digit. Blank past that finger's point count, "waiting" until it has
    a frame to show."""
    if i >= count:
        text = ""
    elif points is None:
        text = "waiting"
    else:
        p = points[i]
        text = f"{p['fx']:+.1f},{p['fy']:+.1f},{p['fz']:+.1f}"
    return f"{text:<{CELL_W}}"


def main():
    wuji_sdk.set_log_level("error")
    manager = SdkManager.instance()
    devices = [d for d in manager.scan() if d.device_type == DeviceType.WujiHand2]
    if not devices:
        print("No Wuji Hand 2 found")
        return
    hand = manager.connect(sn=devices[0].sn, device_name="wuji_hand_2")
    print(f"Connected {hand.serial_number}")

    # Fetch each finger's info (data-frame layout) once, build its decoder, take
    # the contact threshold from the unit the format declares, and place the
    # point cloud on the hand model from the mount pose the format carries.
    decoders, thresholds, units, placements, digests, point_counts = {}, {}, {}, {}, {}, {}
    force_rots = {}
    for i, name in enumerate(FINGERS):
        fmt, digests[name] = fetch_format(hand, i)
        # Rotation that takes the aggregate force into the link frame:
        # R(base_rpy) @ R(aggregate_rpy). Firmware without aggregate_rpy leaves the
        # aggregate axes undeclared, and the only sane reading is the sensor frame.
        if "base_rpy" not in fmt:
            force_rots[name] = None
        elif "aggregate_rpy" not in fmt:
            force_rots[name] = rpy_to_matrix(fmt["base_rpy"])
        else:
            agg_rpy = fmt["aggregate_rpy"]
            if not isinstance(agg_rpy, list) or len(agg_rpy) != 3:
                raise ValueError("aggregate_rpy must be 3 numbers")
            force_rots[name] = mat_mul(rpy_to_matrix(fmt["base_rpy"]), rpy_to_matrix(agg_rpy))
        units[name] = point_unit(fmt)
        thresholds[name] = contact_threshold(units[name])
        decoders[name] = make_decoder(fmt)
        placements[name] = make_placement(fmt)
        point_counts[name] = fmt["point_count"]
        where = ("placed on the hand model" if placements[name]
                 else "no mount pose in this firmware; positions unavailable")
        if placements[name] and placements[name][2]:
            x, y, z = (c * 1000 for c in placements[name][2])
            where += f", aggregate force acts at ({x:+.2f},{y:+.2f},{z:+.2f})mm"
        print(f"  {name}: {fmt['point_count']} points, per-point unit={units[name]}, "
              f"contact>{thresholds[name]}, {where}")
    if any(p is None for p in placements.values()):
        print("  (update the hand firmware to get contact positions on the hand model)")

    # Typed data subscriptions; each frame is a FingertipSensorData
    # (.header / .info_digest / .data).
    subs = {
        "thumb": hand.fingertip_thumb_data().subscribe(),
        "index": hand.fingertip_index_data().subscribe(),
        "middle": hand.fingertip_middle_data().subscribe(),
        "ring": hand.fingertip_ring_data().subscribe(),
        "pinky": hand.fingertip_pinky_data().subscribe(),
    }

    print(f"\nSubscribed to 5 finger streams, running {DURATION}s (Ctrl+C to stop)\n")
    latest = {name: None for name in FINGERS}
    lines_ready = False
    max_points = max(point_counts.values())
    display_rows = len(FINGERS) + 2 + max_points  # status lines + table header + points
    t0 = time.monotonic()
    next_tick = t0
    try:
        while time.monotonic() - t0 < DURATION:
            for name, sub in subs.items():
                while (frame := sub.recv()) is not None:
                    latest[name] = frame
            lines = []
            decoded = dict.fromkeys(FINGERS)
            for name in FINGERS:
                frame = latest[name]
                if frame is None:
                    lines.append(f"{name:<7} waiting for data...")
                    continue
                # info_digest binds the frame to the info this decoder came from.
                # Two revisions can share a payload length while differing in
                # scale, unit or mounting pose, so a length check is not enough;
                # on mismatch a real application re-fetches info.
                if frame.info_digest != digests[name]:
                    lines.append(f"{name:<7} info changed "
                                 f"(0x{frame.info_digest:08x} != 0x{digests[name]:08x}); re-GET info")
                    continue
                try:
                    points, agg = decoders[name](bytes(frame.data))
                except ValueError as exc:
                    # A frame that matches the digest but not the declared length
                    # is a fault, not a partial read. The C example drops it the
                    # same way rather than stopping.
                    lines.append(f"{name:<7} {exc}")
                    continue
                decoded[name] = points
                hits = [k for k, p in enumerate(points) if force(p) > thresholds[name]]
                line = (f"{name:<7} {len(points):>2}pt  temp={agg['temperature']:5.1f}C  "
                        f"force=({agg['fx']:+6.2f},{agg['fy']:+6.2f},{agg['fz']:+6.2f})N  "
                        f"contacts={len(hits):>2}")
                # The aggregate force plus its application point is a wrench:
                # rotate the force into the link frame and take p x f about the
                # link origin. Reported in mN*m; a 1 N push ~18 mm out is ~18.
                if placements[name] and placements[name][2]:
                    px, py, pz = placements[name][2]
                    fx, fy, fz = mat_vec(force_rots[name],
                                         (agg["fx"], agg["fy"], agg["fz"]))
                    line += (f"  moment=({(py * fz - pz * fy) * 1000:+6.1f},"
                             f"{(pz * fx - px * fz) * 1000:+6.1f},"
                             f"{(px * fy - py * fx) * 1000:+6.1f})mN*m")
                # Locate the hardest-pressed point on the fingertip itself.
                if hits and placements[name]:
                    positions, axes, _ = placements[name]
                    peak = max(hits, key=lambda k: force(points[k]))
                    x, y, z = (c * 1000 for c in positions[peak])
                    fx, fy, fz = mat_vec(axes[peak], (points[peak]["fx"],
                                                      points[peak]["fy"],
                                                      points[peak]["fz"]))
                    line += (f"  peak#{peak:<2} at ({x:+6.2f},{y:+6.2f},{z:+6.2f})mm "
                             f"dir=({fx:+5.2f},{fy:+5.2f},{fz:+5.2f})")
                lines.append(line)

            # Every point's force components, five fingers side by side.
            lines.append("pt   | " + " | ".join(f"{name[:CELL_W]:^{CELL_W}}" for name in FINGERS))
            lines.append("-----+" + "+".join("-" * (CELL_W + 2) for _ in FINGERS))
            for i in range(max_points):
                lines.append(f"p{i:02d}  | " + " | ".join(
                    point_cell(decoded[name], point_counts[name], i) for name in FINGERS))

            if lines_ready:
                sys.stdout.write(f"\x1b[{display_rows}A")
            sys.stdout.write("".join(f"\x1b[2K{line}\n" for line in lines))
            sys.stdout.flush()
            lines_ready = True
            next_tick += REFRESH_S
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_tick = time.monotonic()  # fell behind; don't try to catch up
    except KeyboardInterrupt:
        pass
    finally:
        for sub in subs.values():
            sub.close()
        manager.disconnect_all()


if __name__ == "__main__":
    main()
