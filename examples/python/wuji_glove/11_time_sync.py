#!/usr/bin/env python3
"""
Purpose:
    Synchronize the glove clock with the host clock.
Prerequisites:
    Install wuji-sdk and connect a reachable Wuji Glove.
Usage:
    python 11_time_sync.py
    python 11_time_sync.py --sn DEVICE_SN
Expected Behavior:
    Print clock offset, round-trip time, and synchronization timestamp in microseconds.
    Disconnect on completion, error, or Ctrl-C. Exit 0 on completion or
    Ctrl-C, 1 on execution or cleanup failure, and 2 on invalid arguments.
    Errors go to stderr.
"""

import argparse
import sys

from wuji_sdk import SdkManager, WujiGlove


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
        if not isinstance(device, WujiGlove):
            raise RuntimeError(f"expected Wuji Glove, got {type(device).__name__}")
        operation = "read serial number"
        print(f"Connected: {device.serial_number}")
        operation = "synchronize device time"
        sync = device.sync_time()
        print(f"offset_us={sync.offset_us} round_trip_us={sync.round_trip_us} synced_at_us={sync.synced_at_us}")
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
