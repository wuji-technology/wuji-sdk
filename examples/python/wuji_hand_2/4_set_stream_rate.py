#!/usr/bin/env python3
"""
Purpose:
    Set the Wuji Hand 2 joint-state stream rate.
Prerequisites:
    Install wuji-sdk and make one Wuji Hand 2 reachable. Use --sn if several
    devices are discoverable.
Usage:
    python 4_set_stream_rate.py
    python 4_set_stream_rate.py --sn DEVICE_SN
Expected Behavior:
    Request 100 Hz, print the applied rate and five frames within 5 seconds, then restore the default.
    No motor control. Exit 0 on completion or Ctrl-C, 1 on execution or cleanup
    failure, and 2 on invalid arguments. Errors go to stderr.
    The rate is shared with joint_diagnostics and other subscribers.
    The device may quantize the request; 0 restores its default rate.
"""

import argparse
import asyncio
import sys

from wuji_sdk import SdkManager, WujiHand2


def connect_device(manager, sn):
    if sn is not None:
        return manager.connect(sn=sn, device_name="example")
    return manager.auto_connect(device_name="example")


REQUESTED_HZ = 100


async def receive_frames(sub):
    for _ in range(5):
        frame = await sub.recv_async()
        print(f"joint_states frame: {len(frame.joints)} joints")


def main():
    parser = argparse.ArgumentParser(allow_abbrev=False)
    parser.add_argument("--sn", metavar="DEVICE_SN", help="connect to this serial number")
    args = parser.parse_args()
    if args.sn == "":
        parser.error("--sn must not be empty")

    manager = None
    sub = None
    rate_changed = False
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
        sub = device.joint_states().subscribe()
        operation = "set stream rate"
        actual = sub.set_rate(REQUESTED_HZ)
        rate_changed = True
        print(f"joint_states: requested {REQUESTED_HZ} Hz, device applied {actual} Hz")
        operation = "receive five joint-state frames"
        asyncio.run(asyncio.wait_for(receive_frames(sub), timeout=5.0))
    except KeyboardInterrupt:
        pass
    except asyncio.TimeoutError:
        print(f"{operation} failed: timed out waiting for five frames within 5 seconds", file=sys.stderr)
        result = 1
    except Exception as error:
        print(f"{operation} failed: {error}", file=sys.stderr)
        result = 1
    finally:
        if sub is not None:
            if rate_changed:
                try:
                    print(f"restored to the device default: {sub.set_rate(0)} Hz")
                except Exception as error:
                    print(f"restore stream rate failed: {error}", file=sys.stderr)
                    result = 1
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
