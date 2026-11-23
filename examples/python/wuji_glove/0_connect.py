#!/usr/bin/env python3
"""
Purpose:
    Scan for devices, then connect to one Wuji Glove and print its identity.
Prerequisites:
    Install wuji-sdk and connect a reachable Wuji Glove running firmware 1.0.4
    or later. Automatic connection requires exactly one discovered device;
    use --sn when several are present.
Usage:
    python 0_connect.py
    python 0_connect.py --sn DEVICE_SN
Expected Behavior:
    Print the device count and each device's serial number, type, and address.
    Print the connected serial number, firmware version, hardware version,
    and handedness, then list parameters and topics and disconnect.
    The SN connection demonstrates explicit timeout and retry options.
    Exit 0 on completion or Ctrl-C with successful cleanup, 1 on an operation
    or cleanup failure, and 2 on invalid arguments. Errors go to stderr.
"""

from __future__ import annotations

import argparse
import sys

from wuji_sdk import ConnectOptions, SdkManager, WujiGlove


def connect_device(manager, sn):
    # By address: return manager.connect(address="192.168.1.100:50001", device_name="example")
    # By handedness, restricted to this device type:
    # from wuji_sdk import DeviceType, Handedness
    # return manager.connect(handedness=Handedness.Left, device_type=DeviceType.WujiGlove,
    #                        device_name="example")
    if sn is not None:
        options = ConnectOptions(timeout_ms=1000, retry_count=3)
        return manager.connect(sn=sn, device_name="example", options=options)
    return manager.auto_connect(device_name="example")


def main():
    parser = argparse.ArgumentParser(allow_abbrev=False)
    parser.add_argument("--sn", metavar="DEVICE_SN", help="connect to this serial number")
    args = parser.parse_args()
    if args.sn == "":
        parser.error("--sn must not be empty")

    manager = None
    result = 0
    operation = "initialize SDK"
    try:
        manager = SdkManager.instance()
        operation = "scan devices"
        devices = manager.scan()
        print(f"Found {len(devices)} device(s)")
        for found in devices:
            print(f"SN={found.sn}, Type={found.device_type}, Address={found.address}")
        operation = "connect"
        device = connect_device(manager, args.sn)
        operation = "check device type"
        if not isinstance(device, WujiGlove):
            raise RuntimeError(f"expected Wuji Glove, got {type(device).__name__}")
        operation = "read serial number"
        print(f"Connected: {device.serial_number}")
        operation = "read firmware version"
        print(f"Firmware version: {device.version().get()}")
        operation = "read hardware version"
        hardware = device.hw_version().get()
        print(f"Hardware version: {hardware.major}.{hardware.minor}.{hardware.patch}")
        operation = "read hand side"
        print(f"Handedness: {device.hand_side().get()}")
        operation = "list parameters"
        for param in device.params():
            print(f"Parameter: {param.path}, readable={param.can_get}, writable={param.can_set}")
        operation = "list topics"
        for topic in device.topics():
            print(f"Topic: {topic.path}")
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
