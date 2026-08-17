#!/usr/bin/env python3
"""
Publish example — Wuji Hand 2 joint command.

Auto-detect and connect to a Wuji Hand 2, configure MIT impedance control,
enable motors, then **smoothly return every joint to zero** before holding
pos=0. The smooth return linearly interpolates from the hand's current measured
position to 0 over HOME_SECONDS instead of commanding pos=0 in a single step.

Why interpolate instead of publishing pos=0 directly? Under MIT impedance
control the very first pos=0 frame has a position error of "current angle - 0",
which can be large; with a non-trivial KP that produces a large torque, so the
hand snaps back to zero abruptly. Reading the current position and ramping the
target to 0 keeps the per-frame error small, so the motion stays gentle while
KP / KD stay at their normal values (the softness comes from the target
trajectory, not from lowering the gains).

This demonstrates the `hand.joint_command().publish()` typed publish API:

    publisher.send([JointCommand(pos, vel, eff), ...])  # one 20-joint frame

Each call sends one 20-element frame (one JointCommand per joint); the SDK
serialises and ships it with no response-wait, scaling to high-rate loops.
The current position is read from the `joint_states` stream.

Usage: python 2.publish.py
"""

import sys
import time

from wuji_sdk import DeviceType, JointCommand, SdkManager, WujiHand2

TOTAL_JOINTS = 20
PUB_HZ = 200
HOME_SECONDS = 1.5   # smooth return-to-zero duration; larger is softer
HOLD_SECONDS = 5.0

# MIT impedance gains — conservative defaults that hold the joints softly at
# their commanded position. These do NOT need tuning to avoid the snap-back;
# the smooth return comes from ramping the target, not from lowering the gains.
KP = 3.0
KD = 0.05
EFFORT_LIMIT = 1.5  # Amps


def read_current_positions(hand, timeout=2.0):
    """Read one joint_states frame and return the 20 current positions (rad).

    Offline joints report position=None and are treated as 0.0 so the
    interpolation start pose is always well-defined.
    """
    sub = hand.joint_states().subscribe()
    try:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            frame = sub.recv()
            if frame is not None and frame.joints:
                return [
                    float(j.position) if j.position is not None else 0.0
                    for j in frame.joints
                ]
            time.sleep(0.01)
        raise TimeoutError("Timed out reading joint_states for the home start pose")
    finally:
        sub.close()


def smooth_home(hand, publisher, duration, hz=PUB_HZ):
    """Linearly interpolate every joint from its current position to 0.

    Publishes one JointCommand frame per tick at `hz`, ramping the target to
    zero over `duration` seconds so the per-frame error (and thus torque)
    stays small throughout the motion.
    """
    start = read_current_positions(hand)
    print(f"Home start (first 4 joints): {[f'{p:+.3f}' for p in start[:4]]} ...")
    steps = max(1, int(duration * hz))
    dt = 1.0 / hz
    t0 = time.monotonic()
    for i in range(1, steps + 1):
        a = i / steps  # 0 -> 1
        q = [s * (1.0 - a) for s in start]  # lerp each joint toward 0
        publisher.send([JointCommand(p, 0.0, 0.0) for p in q])
        wait = (t0 + dt * i) - time.monotonic()
        if wait > 0:
            time.sleep(wait)
    print(f"Smooth home complete ({duration:.1f}s, {steps} frames).")


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

        # 3. Open the typed publisher and smoothly return to zero (interpolate
        #    from the current pose so the hand does not snap back).
        publisher = hand.joint_command().publish()
        smooth_home(hand, publisher, duration=HOME_SECONDS)

        # 4. Hold pos=0 (all-zero JointCommands = hold pose softly) at PUB_HZ.
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
