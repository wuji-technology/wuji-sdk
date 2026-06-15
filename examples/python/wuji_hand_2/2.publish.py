#!/usr/bin/env python3
"""
Publish example — Wuji Hand 2 joint command.

Auto-detect and connect to a Wuji Hand 2, configure MIT impedance control,
enable motors, then stream zero-position commands at 200 Hz for 5 seconds —
holding every joint at pos=0 with zero feedforward velocity / effort.

This demonstrates the `hand.joint_command_publisher()` typed publish API:

    publisher.send(positions)                      # positions only
    publisher.send(positions, velocities)          # + velocities
    publisher.send(positions, velocities, efforts) # + effort feedforward

Each call sends one 20-joint frame; the SDK serialises and ships it to the
device with no response-wait, so this pattern scales to high-rate control loops.

Usage: python 2.publish.py
"""

import sys
import time

from wuji_sdk import SdkManager, WujiHand2

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
    hand: WujiHand2 = manager.auto_connect(device_name="wuji_hand_2")

    n_online = hand.online_joints_count().get()
    print(f"Connected: {hand.serial_number} ({n_online} joints online)")
    if n_online == 0:
        print("No joints online, aborting.")
        manager.disconnect_all()
        sys.exit(1)

    publisher = None
    try:
        # 1. Configure control: MIT mode, effort limit, per-joint kp/kd matrices.
        hand.control_mode().set("mit")
        hand.effort_limit().set(EFFORT_LIMIT)
        hand.mit_params().set(kp=[[KP] * 4] * 5, kd=[[KD] * 4] * 5)
        hand.enable()

        # 2. Open the typed publisher and stream zero commands at PUB_HZ.
        publisher = hand.joint_command_publisher()
        zeros = [0.0] * TOTAL_JOINTS
        dt = 1.0 / PUB_HZ
        end = time.monotonic() + HOLD_SECONDS

        print(f"Holding pos=0 for {HOLD_SECONDS:.1f}s at {PUB_HZ} Hz ...")
        n = 0
        t0 = time.monotonic()
        while time.monotonic() < end:
            # positions + velocities + efforts (all zeros = hold pose softly)
            publisher.send(zeros, zeros, zeros)
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
