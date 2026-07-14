#!/usr/bin/env python3
"""
Publish example - Wuji Hand joint command.

Auto-detect and connect to a Wuji Hand, enable motors, then stream zero-position
commands at 100 Hz for 3 seconds.

Commands are sent through a LowPass realtime controller.

Usage: python 1.publish.py
"""

import sys
import time

from wuji_sdk import (
    DeviceType,
    HandJointStates,
    JointCommand,
    LowPass,
    SdkManager,
    WujiHand,
)

TOTAL_JOINTS = 20
PUB_HZ = 100
HOLD_SECONDS = 3.0
EFFORT_LIMIT = 1.5
LOWPASS_CUTOFF_HZ = 5.0


def print_joint_state(label: str, state: HandJointStates):
    thumb = state.position[:4]
    positions = [f"{p:+.3f}" for p in thumb]
    print(f"{label}: thumb_pos={positions}")


def main():
    manager = SdkManager.instance()
    all_devices = manager.scan()
    for dev in all_devices:
        print(f"  SN={dev.sn}, Type={dev.device_type}, Address={dev.address}")
    devices = [dev for dev in all_devices if dev.device_type == DeviceType.WujiHand]

    if not devices:
        print("No Wuji Hand devices found")
        return 1

    hand: WujiHand = manager.connect(sn=devices[0].sn, device_name="wuji_hand")
    print(
        f"Connected: {hand.serial_number} "
        f"({hand.handedness_name()}, tactile_attached={hand.is_tactile_attached()})"
    )

    publisher = None
    try:
        print_joint_state("Initial joint state", hand.read_joint_state())

        hand.set_all_effort_limit(EFFORT_LIMIT)
        hand.enable()

        print(f"Starting LowPass controller ({LOWPASS_CUTOFF_HZ:.1f} Hz) ...")
        with hand.realtime_controller(LowPass(cutoff_hz=LOWPASS_CUTOFF_HZ)):
            publisher = hand.joint_command().publish()
            zeros = [JointCommand(0.0, 0.0, 0.0)] * TOTAL_JOINTS
            dt = 1.0 / PUB_HZ
            end = time.monotonic() + HOLD_SECONDS

            print(f"Holding pos=0 for {HOLD_SECONDS:.1f}s at {PUB_HZ} Hz ...")
            n = 0
            t0 = time.monotonic()
            while time.monotonic() < end:
                publisher.send(zeros)
                n += 1
                wait = (t0 + dt * n) - time.monotonic()
                if wait > 0:
                    time.sleep(wait)
            print(f"Sent {n} command frames.")
            publisher.close()
            publisher = None

        print_joint_state("Final joint state", hand.read_joint_state())
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
