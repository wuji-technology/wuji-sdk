#!/usr/bin/env python3
"""
Purpose:
    Publish a cosine sweep in position mode.
Prerequisites:
    Install wuji-sdk and use firmware 3.0.0 or later. All 20 joints must be
    online. Keep the hand clear of people and objects.
Usage:
    python 8_position_mode.py
    python 8_position_mode.py --sn DEVICE_SN
Expected Behavior:
    The SDK may report a thumb abduction stall warning during this motion.
    Disable, select position mode, scale each joint's effort limit to 80%
    of its current value, verify it, set the command cutoff to 40 Hz, and enable.
    Ramp to zero in 1 second at 1 kHz, then move all joints from zero to
    0.35 rad and back twice at 120 Hz. Disable and restore the original effort
    limits before disconnecting.
    Disconnect on completion, error, or Ctrl-C. Exit 0 on completion or
    Ctrl-C, 1 on execution or cleanup failure, and 2 on invalid arguments.
"""

import argparse
import math
import signal
import sys
import time

from wuji_sdk import ControlMode, JointCommand, SdkManager, WujiHand2

JOINT_COUNT = 20
PUB_HZ = 120
RAMP_HZ = 1000
AMPLITUDE = 0.35
PERIOD_SECONDS = 4
CYCLES = 2
COMMAND_CUTOFF_HZ = 40.0
EFFORT_LIMIT_SCALE = 0.8


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


def read_effort_limits(resource):
    limits = resource.get()
    if len(limits) != JOINT_COUNT or any(
        value is None or not math.isfinite(value) or value < 0 for value in limits
    ):
        raise RuntimeError("effort_limit must report a valid value for all 20 joints")
    return list(limits)


def write_effort_limits(resource, limits):
    resource.set(limits)
    reported = read_effort_limits(resource)
    if any(not math.isclose(written, read, rel_tol=1e-5, abs_tol=1e-6)
           for written, read in zip(limits, reported)):
        raise RuntimeError("effort_limit readback does not match the values written")


def publish_sweep(publisher, start):
    started = time.monotonic()
    for step in range(1, RAMP_HZ + 1):
        scale = 0.5 * (1.0 + math.cos(math.pi * step / RAMP_HZ))
        publisher.send([JointCommand(position * scale, 0.0, 0.0) for position in start])
        time.sleep(max(0.0, started + step / RAMP_HZ - time.monotonic()))
    started = time.monotonic()
    steps = PERIOD_SECONDS * PUB_HZ * CYCLES
    for step in range(steps + 1):
        position = AMPLITUDE * 0.5 * (1.0 - math.cos(2.0 * math.pi * step / (PERIOD_SECONDS * PUB_HZ)))
        commands = [JointCommand(position, 0.0, 0.0) for _ in range(JOINT_COUNT)]
        publisher.send(commands)
        if step < steps:
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
    original_limits = None
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
        # Disable joints before switching control mode.
        operation = "disable joints"
        hand.disable()
        operation = "set control mode"
        hand.control_mode().set(ControlMode.Position)
        operation = "read effort limits"
        original_limits = read_effort_limits(hand.effort_limit())
        operation = "write effort limits"
        write_effort_limits(hand.effort_limit(), [value * EFFORT_LIMIT_SCALE for value in original_limits])
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
        operation = "publish sweep"
        publish_sweep(publisher, start)
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
                if original_limits is not None:
                    print("Warning: original effort limits may not be restored. "
                          "Check and restore them, or power-cycle the device to restore default limits, "
                          "before running this example again.", file=sys.stderr)
                result = 1
            else:
                if original_limits is not None:
                    try:
                        write_effort_limits(hand.effort_limit(), original_limits)
                    except Exception as error:
                        print(f"restore effort limits failed: {error}", file=sys.stderr)
                        print("Warning: original effort limits may not be restored. "
                              "Check and restore them, or power-cycle the device to restore default limits, "
                              "before running this example again.", file=sys.stderr)
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
