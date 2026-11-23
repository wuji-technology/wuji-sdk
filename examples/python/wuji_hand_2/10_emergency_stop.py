#!/usr/bin/env python3
"""
Purpose:
    Send a whole-hand emergency stop.
Prerequisites:
    Install wuji-sdk and connect a reachable Wuji Hand 2.
Usage:
    python 10_emergency_stop.py
    python 10_emergency_stop.py --sn DEVICE_SN
Expected Behavior:
    Send the emergency-stop command and print confirmation.
    Disconnect on completion, error, or Ctrl-C. Exit 0 on completion or
    Ctrl-C, 1 on execution or cleanup failure, and 2 on invalid arguments.
    Errors go to stderr.
"""

import argparse
import sys

from wuji_sdk import SdkManager, WujiHand2


def connect_device(manager, sn):
    if sn is not None:
        return manager.connect(sn=sn, device_name="example")
    return manager.auto_connect(device_name="example")


def main():
    parser = argparse.ArgumentParser(allow_abbrev=False)
    parser.add_argument("--sn", metavar="DEVICE_SN")
    args = parser.parse_args()
    if args.sn == "":
        parser.error("--sn must not be empty")

    manager = None
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
        operation = "send emergency stop"
        device.emergency_stop()
        print("Emergency stop sent")
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print(f"{operation} failed: {error}", file=sys.stderr)
        result = 1
    finally:
        if manager is not None:
            try:
                manager.disconnect_all()
            except Exception as error:
                print(f"disconnect failed: {error}", file=sys.stderr)
                result = 1
    return result


if __name__ == "__main__":
    sys.exit(main())
