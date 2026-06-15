#!/usr/bin/env python3
"""
Subscription example - Wuji Hand.

Auto-detect and connect to a Wuji Hand, then subscribe to the joint_state data
stream using subscribe().

Usage: python 0.subscribe.py
"""

import time

from wuji_sdk import HandJointState, SdkManager, TransportType, WujiHand

REPORT_SECONDS = 1.0
POLL_SLEEP_SECONDS = 0.0001
FINGERS = 5
JOINTS_PER_FINGER = 4  # 5 fingers x 4 joints = 20 joints total


def main():
    manager = SdkManager.instance()
    devices = [dev for dev in manager.scan() if dev.transport_type == TransportType.Usb]

    if not devices:
        print("No Wuji Hand devices found")
        return 1

    hand: WujiHand | None = None
    sub = None
    try:
        hand = manager.connect(sn=devices[0].sn, device_name="wuji_hand")
        print(f"Connected: {hand.serial_number} ({hand.handedness_name()})")

        sub = hand.joint_state().subscribe()
        print("Subscribed to hand.joint_state(). Ctrl+C to stop.\n")

        total = 0
        window_frames = 0
        window_start = time.monotonic()
        while True:
            state: HandJointState | None = sub.recv()
            if state is None:
                time.sleep(POLL_SLEEP_SECONDS)
                continue

            total += 1
            window_frames += 1
            now = time.monotonic()
            elapsed = now - window_start
            if elapsed >= REPORT_SECONDS:
                fps = window_frames / elapsed
                print(f"[{hand.device_name}][JointState] fps={fps:5.1f} total={total}")
                # state.position holds all 20 joints in finger-major order
                # (finger1..5 x joint1..4); print one row per finger.
                for f in range(FINGERS):
                    joints = state.position[f * JOINTS_PER_FINGER : (f + 1) * JOINTS_PER_FINGER]
                    values = [f"{p:+.3f}" for p in joints]
                    print(f"  finger{f + 1}: {values}")
                window_frames = 0
                window_start = now
    except KeyboardInterrupt:
        pass
    finally:
        if sub is not None:
            sub.close()
        manager.disconnect_all()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
