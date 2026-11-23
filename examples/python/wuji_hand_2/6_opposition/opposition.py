#!/usr/bin/env python3
"""
Purpose:
    Replay the recorded thumb-opposition trajectory in position mode.
Prerequisites:
    Install wuji-sdk and use firmware 3.0.0 or later. All 20 joints must be
    online. Keep the hand clear of people and objects. Keep the data directory beside this source file.
Usage:
    python opposition.py
    python opposition.py --sn DEVICE_SN
Expected Behavior:
    The SDK may report a thumb abduction stall warning during this motion.
    Select the replay by device handedness. Disable, select position mode,
    set the command cutoff to 40 Hz, and enable. Ramp to the first recorded pose in 1 second, then publish
    every recorded pose once at 1 kHz. Disable and disconnect on completion,
    error, or Ctrl-C. Exit 0 on completion or Ctrl-C, 1 on execution or
    cleanup failure, and 2 on invalid arguments.
"""

import argparse
import math
import signal
import struct
from pathlib import Path
import sys
import time

from wuji_sdk import ControlMode, JointCommand, SdkManager, WujiHand2

JOINT_COUNT = 20
PUB_HZ = 1000
COMMAND_CUTOFF_HZ = 40.0
HEADER = struct.Struct("<8sHBBI")
FRAME = struct.Struct("<I20f")
SIDE_TO_ID = {"right": 1, "left": 2}


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


def open_replay(side):
    if side not in SIDE_TO_ID:
        raise RuntimeError(f"unsupported handedness: {side}")
    path = Path(__file__).parent / "data" / f"{side}.replay"
    stream = path.open("rb")
    try:
        magic, version, side_id, joints, count = HEADER.unpack(stream.read(HEADER.size))
        if (magic != b"WJH2RPL\0" or version != 1 or side_id != SIDE_TO_ID[side]
                or joints != JOINT_COUNT or count == 0):
            raise RuntimeError("invalid replay header")
        if path.stat().st_size != HEADER.size + count * FRAME.size:
            raise RuntimeError("replay file length does not match frame count")
        return stream, count
    except BaseException:
        stream.close()
        raise


def read_pose(stream):
    positions = FRAME.unpack(stream.read(FRAME.size))[1:]
    if not all(math.isfinite(position) for position in positions):
        raise RuntimeError("invalid position in replay")
    return positions


def publish_replay(publisher, start, stream, count, first):
    started = time.monotonic()
    for step in range(1, PUB_HZ + 1):
        ratio = 0.5 * (1.0 - math.cos(math.pi * step / PUB_HZ))
        positions = [current + (target - current) * ratio for current, target in zip(start, first)]
        publisher.send([JointCommand(position, 0.0, 0.0) for position in positions])
        time.sleep(max(0.0, started + step / PUB_HZ - time.monotonic()))
    started = time.monotonic()
    for step in range(count):
        publisher.send([JointCommand(position, 0.0, 0.0) for position in read_pose(stream)])
        if step + 1 < count:
            time.sleep(max(0.0, started + (step + 1) / PUB_HZ - time.monotonic()))


def main():
    parser = argparse.ArgumentParser(allow_abbrev=False)
    parser.add_argument("--sn", metavar="DEVICE_SN", help="connect to this serial number")
    args = parser.parse_args()
    if args.sn == "":
        parser.error("--sn must not be empty")

    manager = None
    hand = None
    publisher = None
    stream = None
    result = 0
    operation = "initialize SDK"
    try:
        manager = SdkManager.instance()
        operation = "connect"
        device = connect_device(manager, args.sn)
        operation = "check device type"
        if not isinstance(device, WujiHand2):
            raise RuntimeError(f"expected Wuji Hand 2, got {type(device).__name__}")
        hand = device
        operation = "read serial number"
        print(f"Connected: {hand.serial_number}")
        operation = "open replay"
        stream, count = open_replay(hand.handedness().get())
        operation = "read replay pose"
        first = read_pose(stream)
        stream.seek(HEADER.size)
        # Disable joints before switching control mode.
        operation = "disable joints"
        hand.disable()
        operation = "set control mode"
        hand.control_mode().set(ControlMode.Position)
        operation = "set position command low pass cutoff frequency"
        hand.position_command_low_pass_cutoff_frequency().set(COMMAND_CUTOFF_HZ)
        operation = "enable joints"
        hand.enable()
        operation = "wait for joints to be enabled"
        wait_enabled(hand)
        operation = "read joint positions"
        start = read_positions(hand)
        operation = "create publisher for joint command"
        publisher = hand.joint_command().publish()
        operation = "publish replay"
        publish_replay(publisher, start, stream, count, first)
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
        if stream is not None:
            try:
                stream.close()
            except Exception as error:
                print(f"close replay failed: {error}", file=sys.stderr)
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
