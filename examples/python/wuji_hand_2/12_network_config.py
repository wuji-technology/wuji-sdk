#!/usr/bin/env python3
"""
Purpose:
    Read or update the device IP address and data port.
Prerequisites:
    Install wuji-sdk and connect a reachable Wuji Hand 2.
Usage:
    python 12_network_config.py
    python 12_network_config.py --sn DEVICE_SN --ip 192.168.1.100 --port 50001
    --port accepts 7400-65535, excluding the discovery port 50000.
Expected Behavior:
    Read and print the IP address and data port. Write only settings supplied
    with --ip or --port. Written settings are not restored on exit.
    IP and port changes take effect after a device reboot; this example does not reboot.
    If the new IP is outside the computer's subnet, discovery and connection may fail.
    Configure a computer interface on the device's new /24 subnet before reconnecting.
    Exit 0 on completion or Ctrl-C, 1 on execution or cleanup failure,
    and 2 on invalid arguments. Errors go to stderr.
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
    parser.add_argument("--ip", metavar="IP_ADDRESS")
    parser.add_argument("--port", type=int, metavar="PORT")
    args = parser.parse_args()
    if args.sn == "":
        parser.error("--sn must not be empty")

    if args.ip == "":
        parser.error("--ip must not be empty")
    if args.port is not None and (not 7400 <= args.port <= 65535 or args.port == 50000):
        parser.error("--port must be between 7400 and 65535, excluding 50000")

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
        operation = "read ip"
        print(f"IP: {device.ip().get()}")
        operation = "read port"
        print(f"port: {device.port().get()}")
        if args.port is not None:
            operation = "set port"
            device.port().set(args.port)
            print(f"port setting sent: {args.port}")
        if args.ip is not None:
            operation = "set ip"
            device.ip().set(args.ip)
            print(f"IP setting sent: {args.ip}")
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
