#!/usr/bin/env python3
"""
Purpose:
    Explicitly clear faults on all joints.
Prerequisites:
    Install wuji-sdk and connect a reachable Wuji Hand 2.
    Resolve the fault cause before clearing faults.
Usage:
    python 11_clear_fault.py
    python 11_clear_fault.py --sn DEVICE_SN
Expected Behavior:
    Send the clear-fault command and print confirmation. The joints remain disabled.
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
        operation = "disable joints"
        device.disable()
        operation = "clear joint faults"
        device.clear_fault()
        print("Clear fault sent")
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
