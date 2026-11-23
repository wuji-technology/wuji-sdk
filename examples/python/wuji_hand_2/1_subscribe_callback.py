#!/usr/bin/env python3
"""
Purpose:
    Subscribe to Wuji Hand 2 joint states and IMU data with a callback.
Prerequisites:
    Install wuji-sdk and make one Wuji Hand 2 reachable. Use --sn if several
    devices are discoverable.
Usage:
    python 1_subscribe_callback.py
    python 1_subscribe_callback.py --sn DEVICE_SN
Expected Behavior:
    Print joint NID/position pairs and IMU acceleration/angular velocity until Ctrl-C.
    No motor control. Exit 0 on completion or Ctrl-C, 1 on execution or cleanup
    failure, and 2 on invalid arguments. Errors go to stderr.
    The callback runs on an SDK worker thread.
"""

import argparse
import time
import sys

from wuji_sdk import SdkManager, WujiHand2


def connect_device(manager, sn):
    if sn is not None:
        return manager.connect(sn=sn, device_name="example")
    return manager.auto_connect(device_name="example")


def on_joint_states(frame):
    print(f"seq={frame.header.seq} joints={[(joint.nid, joint.position) for joint in frame.joints]}")


def on_imu(frame):
    a = frame.linear_acceleration
    w = frame.angular_velocity
    print(f"IMU seq={frame.header.seq} acceleration=[{a.x:.3f}, {a.y:.3f}, {a.z:.3f}] "
          f"angular_velocity=[{w.x:.3f}, {w.y:.3f}, {w.z:.3f}]")


def main():
    parser = argparse.ArgumentParser(allow_abbrev=False)
    parser.add_argument("--sn", metavar="DEVICE_SN", help="connect to this serial number")
    args = parser.parse_args()
    if args.sn == "":
        parser.error("--sn must not be empty")

    manager = None
    subscriptions = []
    errors = []
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
        operation = "subscribe joint states"
        subscriptions.append(device.joint_states().subscribe_with_callback(on_joint_states, on_error=errors.append))
        operation = "subscribe imu"
        subscriptions.append(device.imu().subscribe_with_callback(on_imu, on_error=errors.append))
        operation = "receive subscribed frames"
        while all(sub.is_active for sub in subscriptions) and not errors:
            time.sleep(1)
        raise RuntimeError(errors[0] if errors else "subscription ended")
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print(f"{operation} failed: {error}", file=sys.stderr)
        result = 1
    finally:
        for sub in reversed(subscriptions):
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
