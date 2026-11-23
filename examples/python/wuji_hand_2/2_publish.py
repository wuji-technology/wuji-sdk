#!/usr/bin/env python3
"""
Purpose:
    Publish joint commands to smoothly return Wuji Hand 2 to zero and hold.
Prerequisites:
    Install wuji-sdk. All 20 joints must be online. Keep the hand clear of
    people and objects; this example moves every joint.
Usage:
    python 2_publish.py
    python 2_publish.py --sn DEVICE_SN
Expected Behavior:
    Disable, select MIT_V2 mode, set physical gains and per-joint torque limits,
    then enable. Publish at 200 Hz: return to zero in 1.5 seconds and hold
    for 5 seconds. Disable and disconnect on completion, error, or Ctrl-C.
    Exit 0 on completion or Ctrl-C, 1 on execution or cleanup failure,
    and 2 on invalid arguments. Errors go to stderr.
Documentation:
    https://docs.wuji.tech/docs/en/wuji-hand/latest/sdk-practice/publish-command
"""

import argparse
import math
import signal
import sys
import time

from wuji_sdk import ControlMode, JointCommand, SdkManager, WujiHand2

JOINT_COUNT = 20
PUB_HZ = 200
HOME_SECONDS = 1.5
HOLD_SECONDS = 5.0
# MIT_V2 starting gains: Kp 0.5..2.0 N*m/rad, Kd 0.01..0.05 N*m*s/rad.
# Retune for the joint and load; these are recommendations, not device limits.
KP = 1.0
KD = 0.02
TORQUE_LIMITS_NM = [
    0.25, 0.15, 0.12, 0.08,
    0.25, 0.08, 0.08, 0.08,
    0.25, 0.08, 0.08, 0.08,
    0.25, 0.08, 0.08, 0.08,
    0.25, 0.08, 0.08, 0.08,
]


def verify_mit_v2_parameters(hand):
    """Reject missing or unapplied physical parameters before enabling."""
    limits = hand.effort_limit().get()
    gains = hand.mit_params().get()
    if len(limits) != JOINT_COUNT or len(gains) != JOINT_COUNT:
        raise RuntimeError("MIT_V2 requires 20 readable parameter entries")
    for index, (limit, gain) in enumerate(zip(limits, gains)):
        if (limit is None or gain is None
                or not math.isclose(limit, TORQUE_LIMITS_NM[index], rel_tol=1e-4, abs_tol=1e-6)
                or not math.isclose(gain.kp, KP, rel_tol=1e-4, abs_tol=1e-6)
                or not math.isclose(gain.kd, KD, rel_tol=1e-4, abs_tol=1e-6)):
            raise RuntimeError(f"MIT_V2 parameter mismatch at joint {index}")


def connect_device(manager, sn):
    if sn is not None:
        return manager.connect(sn=sn, device_name="example")
    return manager.auto_connect(device_name="example")


def wait_enabled(hand):
    sub = hand.joint_diagnostics().subscribe()
    try:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            frame = None
            while (latest := sub.recv()) is not None:
                frame = latest
            if frame is not None and len(frame.joints) == JOINT_COUNT:
                if all(joint.status_word.ext_state == 2 for joint in frame.joints):
                    return
            time.sleep(0.01)
        raise TimeoutError("waiting for all joints to be enabled")
    finally:
        sub.close()


def read_positions(hand):
    sub = hand.joint_states().subscribe()
    try:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            frame = sub.recv()
            if frame is not None:
                positions = [None] * JOINT_COUNT
                for joint in frame.joints:
                    index = WujiHand2.nid_to_joint_index(joint.nid)
                    if positions[index] is not None:
                        raise RuntimeError("duplicate joint in joint_states")
                    if joint.position is None or not math.isfinite(joint.position):
                        raise RuntimeError("invalid joint position")
                    positions[index] = joint.position
                if all(position is not None for position in positions):
                    return positions
            time.sleep(0.01)
        raise TimeoutError("waiting for positions from all 20 joints")
    finally:
        sub.close()


def publish_home(publisher, start):
    home_steps = int(HOME_SECONDS * PUB_HZ)
    hold_steps = int(HOLD_SECONDS * PUB_HZ)
    started = time.monotonic()
    for step in range(1, home_steps + hold_steps + 1):
        scale = max(0.0, 1.0 - step / home_steps)
        publisher.send([JointCommand(position * scale, 0.0, 0.0) for position in start])
        time.sleep(max(0.0, started + step / PUB_HZ - time.monotonic()))


def main():
    parser = argparse.ArgumentParser(allow_abbrev=False)
    parser.add_argument("--sn", metavar="DEVICE_SN", help="connect to this serial number")
    args = parser.parse_args()
    if args.sn is not None and not args.sn.strip():
        parser.error("--sn must not be empty")

    manager = None
    hand = None
    publisher = None
    result = 0
    operation = "initialize SDK"
    try:
        manager = SdkManager.instance()
        operation = "connect"
        device = connect_device(manager, args.sn)
        operation = "check device type"
        if not isinstance(device, WujiHand2):
            raise RuntimeError(f"expected Wuji Hand 2, got {type(device).__name__}")
        operation = "verify serial number"
        if args.sn is not None and device.serial_number != args.sn:
            raise RuntimeError("connected serial number differs from --sn")
        hand = device
        operation = "read serial number"
        print(f"Connected: {hand.serial_number}")
        operation = "check online joints"
        if int(hand.online_joints_count().get()) != JOINT_COUNT:
            raise RuntimeError("MIT_V2 requires 20/20 online joints")
        # Disable joints before switching control mode.
        operation = "disable joints"
        hand.disable()
        operation = "set control mode"
        hand.control_mode().set(ControlMode.MitV2)
        operation = "set MIT params"
        hand.mit_params().set((KP, KD))
        operation = "set effort limit"
        hand.effort_limit().set(TORQUE_LIMITS_NM)
        operation = "verify MIT_V2 parameters"
        verify_mit_v2_parameters(hand)
        operation = "enable joints"
        hand.enable()
        operation = "wait for joints to be enabled"
        wait_enabled(hand)
        operation = "read joint positions"
        start = read_positions(hand)
        operation = "create publisher for joint command"
        publisher = hand.joint_command().publish()
        operation = "publish home commands"
        publish_home(publisher, start)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print(f"{operation} failed: {error}", file=sys.stderr)
        result = 1
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if publisher is not None:
            try:
                publisher.close()
            except Exception as error:
                print(f"close publisher failed: {error}", file=sys.stderr)
                result = 1
        if hand is not None:
            try:
                hand.disable()
            except Exception as error:
                print(f"disable failed: {error}", file=sys.stderr)
                result = 1
        if manager is not None:
            try:
                manager.disconnect_all()
            except Exception as error:
                print(f"disconnect failed: {error}", file=sys.stderr)
                result = 1
    return result


if __name__ == "__main__":
    sys.exit(main())
