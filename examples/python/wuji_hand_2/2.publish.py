#!/usr/bin/env python3
"""
Publish example — Wuji Hand 2 joint command.

Auto-detect and connect to a Wuji Hand 2, configure MIT impedance control,
enable motors, then stream zero-position commands at 200 Hz for 5 seconds —
holding every joint at pos=0 with zero feedforward velocity / effort.

This demonstrates the `hand.joint_command().publish()` typed publish API:

    publisher.send([JointCommand(pos, vel, eff), ...])  # one 20-joint frame

Each call sends one 20-element frame (one JointCommand per joint); the SDK
serialises and ships it with no response-wait, scaling to high-rate loops.

Usage: python 2.publish.py
"""

import sys
import time

from wuji_sdk import DeviceType, JointCommand, SdkManager, WujiHand2

TOTAL_JOINTS = 20
PUB_HZ = 200
HOLD_SECONDS = 5.0

# MIT impedance gains — conservative defaults that hold the joints softly at
# their commanded position. Increase KP for stiffer tracking; KD damps motion.
KP = 3.0
KD = 0.05
EFFORT_LIMIT = 1.5  # Amps


def main():
    manager = SdkManager.instance()
    devices = manager.scan()
    if not devices:
        print("No devices found")
        sys.exit(1)

    for dev in devices:
        print(f"  SN={dev.sn}, Type={dev.device_type}, Address={dev.address}")

    hand_devices = [d for d in devices if d.device_type == DeviceType.WujiHand2]
    if not hand_devices:
        print("No Wuji Hand 2 found among the scanned devices")
        sys.exit(1)

    hand: WujiHand2 = manager.connect(sn=hand_devices[0].sn, device_name="wuji_hand_2")

    n_online = hand.online_joints_count().get()
    print(f"Connected: {hand.serial_number} ({n_online} joints online)")
    if n_online == 0:
        print("No joints online, aborting.")
        manager.disconnect_all()
        sys.exit(1)

    publisher = None
    try:
        # 1. Configure control: effort limit, kp/kd. (Firmware defaults to MIT
        #    mode, so no control_mode set is needed.)
        #    Polymorphic set: a scalar broadcasts to all joints; a 20-element
        #    list applies per-joint.
        hand.effort_limit().set(EFFORT_LIMIT)
        hand.mit_params().set((KP, KD))          # (kp, kd) tuple, broadcast to all joints
        hand.enable()

        # 2. Wait for motors to reach Enabled before publishing commands.
        #    enable() is an action; sending commands too early may race ahead of
        #    the enable completing.
        #    ext_state: 0=Init 1=Ready 2=Enabled 3=Stopped
        deadline = time.monotonic() + 5.0
        diag_sub = hand.joint_diagnostics().subscribe()
        try:
            while time.monotonic() < deadline:
                time.sleep(0.2)
                frame = diag_sub.recv()
                if frame is None or not frame.joints:
                    continue
                if all(e.status_word.ext_state == 2 for e in frame.joints):
                    break
            else:
                print("Enable timeout — not all online motors reached Enabled.")
                sys.exit(1)
        finally:
            diag_sub.close()
        print("All motors enabled.")

        # 3. Open the typed publisher and stream zero commands at PUB_HZ.
        publisher = hand.joint_command().publish()
        zeros = [JointCommand(0.0, 0.0, 0.0) for _ in range(TOTAL_JOINTS)]
        dt = 1.0 / PUB_HZ
        end = time.monotonic() + HOLD_SECONDS

        print(f"Holding pos=0 for {HOLD_SECONDS:.1f}s at {PUB_HZ} Hz ...")
        n = 0
        t0 = time.monotonic()
        while time.monotonic() < end:
            # all-zero JointCommands = hold pose softly
            publisher.send(zeros)
            n += 1
            wait = (t0 + dt * n) - time.monotonic()
            if wait > 0:
                time.sleep(wait)
        print(f"Sent {n} command frames.")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        if publisher is not None:
            publisher.close()
        hand.disable()
        manager.disconnect_all()


if __name__ == "__main__":
    main()
