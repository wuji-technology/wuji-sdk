#!/usr/bin/env python3
"""Fingertip sensor reader — Wuji Hand 2.

The fingertip sensor is self-describing: first GET each finger's info metadata
(carrying a format JSON that describes the data-frame layout), then subscribe to
fingertip/<finger>/data and decode each frame per the format — the layout is
never hardcoded, it all comes from the info contract.
Thumb has 40 points, the other fingers 34, streamed at 100 Hz.

Displays one status line per finger, redrawn in place at 100 Hz; each cycle
drains the subscription queue and shows the newest frame.

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
CONTACT_N = 0.2          # a point counts as "in contact" when |F| exceeds this (newtons)
DURATION = 15            # seconds to run
REFRESH_S = 0.01         # 100 Hz display refresh, matching the stream rate
# contract field type -> struct format char; unknown types raise, never guess
FIELD_FMT = {"i8": "<b", "u8": "<B", "i16": "<h", "u16": "<H",
             "i32": "<i", "u32": "<I", "f32": "<f"}


def fetch_format(hand, finger):
    """GET the finger's info and return its format JSON (the data-frame layout).

    The SDK drives the chunked info read internally and hands back a decoded
    FingertipSensorInfo; we only read its `format` string.
    """
    info = hand.get_fingertip_info(finger)
    fmt = json.loads(info.format)
    if fmt["v"] != 1 or fmt["encoding"] != "point_array":
        raise ValueError(f"unsupported format: {fmt.get('encoding')}")
    return fmt


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


def force(p):
    """Magnitude of a point's force |F| = sqrt(fx^2 + fy^2 + fz^2)."""
    return math.sqrt(p["fx"] ** 2 + p["fy"] ** 2 + p["fz"] ** 2)


def main():
    wuji_sdk.set_log_level("error")
    manager = SdkManager.instance()
    devices = [d for d in manager.scan() if d.device_type == DeviceType.WujiHand2]
    if not devices:
        print("No Wuji Hand 2 found")
        return
    hand = manager.connect(sn=devices[0].sn, device_name="wuji_hand_2")
    print(f"Connected {hand.serial_number}")

    # Fetch each finger's info (data-frame layout) once, build its decoder.
    decoders = {}
    for i, name in enumerate(FINGERS):
        fmt = fetch_format(hand, i)
        print(f"  {name}: {fmt['point_count']} points")
        decoders[name] = make_decoder(fmt)

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
    t0 = time.monotonic()
    next_tick = t0
    try:
        while time.monotonic() - t0 < DURATION:
            for name, sub in subs.items():
                while (frame := sub.recv()) is not None:
                    latest[name] = frame
            lines = []
            for name in FINGERS:
                frame = latest[name]
                if frame is None:
                    lines.append(f"{name:<7} waiting for data...")
                    continue
                points, agg = decoders[name](bytes(frame.data))
                contacts = sum(1 for p in points if force(p) > CONTACT_N)
                lines.append(f"{name:<7} {len(points):>2}pt  temp={agg['temperature']:5.1f}C  "
                             f"force=({agg['fx']:+6.2f},{agg['fy']:+6.2f},{agg['fz']:+6.2f})N  contacts={contacts}")
            if lines_ready:
                sys.stdout.write(f"\x1b[{len(FINGERS)}A")
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
