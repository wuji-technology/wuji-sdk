#!/usr/bin/env python3
"""
Publish example - Wuji Hand grasp loop (open/close finger motion).

Auto-detect and connect to a Wuji Hand, enable motors, then continuously open and
close the four non-thumb fingers with a smooth sinusoidal curl at 200 Hz until
Ctrl+C.

The curl follows `y = (1 - cos(x)) * 0.8`, sweeping each target from 0 (open) to
~1.6 rad (closed) and back every ~2 seconds. Commands are streamed through a
LowPass realtime controller; the actual joint positions are read back for a
simple target-vs-actual readout.

Usage: python 2.grasp_loop.py
"""

import math
import sys
import time

from wuji_sdk import LowPass, SdkManager, TransportType, WujiHand

TOTAL_JOINTS = 20
JOINTS_PER_FINGER = 4  # finger-major layout: [J1, J2, J3, J4] per finger
PUB_HZ = 200
EFFORT_LIMIT = 1.5
LOWPASS_CUTOFF_HZ = 5.0
CURL_AMPLITUDE = 0.8  # peak curl ≈ 2 * 0.8 = 1.6 rad
REPORT_INTERVAL_S = 0.2

# Fingers and joints that follow the curl. Thumb (finger 0) and the second joint
# of each finger (J2) stay at 0, matching the reference open/close gesture.
ACTIVE_FINGERS = (1, 2, 3, 4)  # index, middle, ring, pinky
CURL_JOINTS = (0, 2, 3)  # J1, J3, J4 within a finger
INDEX_FINGER = 1


def curl_target(curl: float) -> list[float]:
    """Build a flat 20-joint command for the given curl amount (radians)."""
    target = [0.0] * TOTAL_JOINTS
    for finger in ACTIVE_FINGERS:
        base = finger * JOINTS_PER_FINGER
        for j in CURL_JOINTS:
            target[base + j] = curl
    return target


def finger_slice(values: list[float], finger: int) -> list[str]:
    base = finger * JOINTS_PER_FINGER
    return [f"{v:+.3f}" for v in values[base : base + JOINTS_PER_FINGER]]


def main():
    manager = SdkManager.instance()
    devices = [dev for dev in manager.scan() if dev.transport_type == TransportType.Usb]

    if not devices:
        print("No Wuji Hand devices found")
        return 1

    hand: WujiHand = manager.connect(sn=devices[0].sn, device_name="wuji_hand")
    print(f"Connected: {hand.serial_number} ({hand.handedness_name()})")

    publisher = None
    try:
        hand.set_all_effort_limit(EFFORT_LIMIT)
        hand.enable()

        print(f"Starting LowPass controller ({LOWPASS_CUTOFF_HZ:.1f} Hz) ...")
        with hand.realtime_controller(LowPass(cutoff_hz=LOWPASS_CUTOFF_HZ)) as ctrl:
            publisher = hand.joint_command().publisher()
            dt = 1.0 / PUB_HZ

            print(f"Opening/closing fingers at {PUB_HZ} Hz. Ctrl+C to stop.\n")
            x = 0.0
            n = 0
            t0 = time.monotonic()
            next_report = t0
            while True:
                curl = (1.0 - math.cos(x)) * CURL_AMPLITUDE
                target = curl_target(curl)
                publisher.send(target)
                n += 1

                now = time.monotonic()
                if now >= next_report:
                    actual = ctrl.get_actual_position()
                    print(
                        f"[{hand.device_name}] curl={curl:+.3f}  "
                        f"index_cmd={finger_slice(target, INDEX_FINGER)}  "
                        f"index_act={finger_slice(actual, INDEX_FINGER)}"
                    )
                    next_report = now + REPORT_INTERVAL_S

                x += math.pi / PUB_HZ
                wait = (t0 + dt * n) - time.monotonic()
                if wait > 0:
                    time.sleep(wait)
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        if publisher is not None:
            publisher.close()
        try:
            hand.disable()
        except Exception as exc:
            print(f"disable failed: {exc}", file=sys.stderr)
        manager.disconnect_all()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
