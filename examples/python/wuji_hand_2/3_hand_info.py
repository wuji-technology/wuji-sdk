#!/usr/bin/env python3
"""
Purpose:
    Read hand identity, joint effort limits, and one diagnostics snapshot.
Prerequisites:
    Install wuji-sdk and connect a reachable Wuji Hand 2.
Usage:
    python 3_hand_info.py
    python 3_hand_info.py --sn DEVICE_SN
Expected Behavior:
    Print handedness, IP, online count, per-joint limits, and diagnostics with fault descriptions. The hand does not move.
    Disconnect on completion, error, or Ctrl-C. Exit 0 on completion or
    Ctrl-C, 1 on execution or cleanup failure, and 2 on invalid arguments.
"""

import argparse
import sys
import time

from wuji_sdk import SdkManager, WujiHand2


def connect_device(manager, sn):
    if sn is not None:
        return manager.connect(sn=sn, device_name="example")
    return manager.auto_connect(device_name="example")


def main():
    parser = argparse.ArgumentParser(allow_abbrev=False)
    parser.add_argument("--sn", metavar="DEVICE_SN", help="connect to this serial number")
    args = parser.parse_args()
    if args.sn == "":
        parser.error("--sn must not be empty")

    manager = None
    sub = None
    result = 0
    operation = "initialize SDK"
    try:
        manager = SdkManager.instance()
        operation = "connect"
        device = connect_device(manager, args.sn)
        operation = "check device type"
        if not isinstance(device, WujiHand2):
            raise RuntimeError(f"expected Wuji Hand 2, got {type(device).__name__}")
        operation = "read serial number"
        print(f"Connected: {device.serial_number}")
        operation = "read handedness"
        print(f"Handedness: {device.handedness().get()}")
        operation = "read ip"
        print(f"IP: {device.ip().get()}")
        operation = "read online joints count"
        print(f"Online joints: {device.online_joints_count().get()}")
        print("effort_limit units: A in MIT mode; N m in Position mode.")
        operation = "read effort limit"
        for index, limit in enumerate(device.effort_limit().get()):
            value = "offline" if limit is None else f"{limit:.3f}"
            print(f"Joint {index}: {value}")
        operation = "subscribe joint diagnostics"
        sub = device.joint_diagnostics().subscribe()
        deadline = time.monotonic() + 5.0
        operation = "receive stream frame"
        frame = sub.recv()
        while frame is None and time.monotonic() < deadline:
            time.sleep(0.01)
            operation = "receive stream frame"
            frame = sub.recv()
        if frame is None:
            raise TimeoutError("waiting for joint diagnostics")
        for joint in frame.joints:
            print(f"NID {joint.nid}: {joint.status_word}, current={joint.current:.2f} A, "
                  f"voltage={joint.vbus_v_fb:.1f} V, temperature={joint.mcu_temp_c_fb:.1f} C, "
                  f"error=0x{joint.error_code_current:04X}")
            if joint.error_code_current:
                operation = "describe joint fault"
                info = WujiHand2.describe_error(joint.error_code_current)
                if info is None:
                    print("  Unknown fault code")
                else:
                    print(f"  {info['name']}: {info['severity']}, {info['clear_policy']}")
                    print(f"  Description: {info['desc']}")
                    print(f"  Cause: {info['cause']}")
                    print(f"  Resolution: {info['resolution']}")
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print(f"{operation} failed: {error}", file=sys.stderr)
        result = 1
    finally:
        if sub is not None:
            try:
                sub.close()
            except Exception as error:
                print(f"close subscription failed: {error}", file=sys.stderr)
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
