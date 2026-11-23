#!/usr/bin/env python3
"""
Purpose:
    Set a subscription stream rate.
Prerequisites:
    Install wuji-sdk and connect a reachable Wuji Glove.
Usage:
    python 6_set_stream_rate.py
    python 6_set_stream_rate.py --sn DEVICE_SN
Expected Behavior:
    Request 30 Hz for emf_poses, print five frames within 5 seconds, then restore the default rate.
    The applied rate affects every subscriber of this stream.
    Exit 0 on completion or Ctrl-C, 1 on execution or cleanup failure,
    and 2 on invalid arguments. Errors go to stderr.
"""

from __future__ import annotations

import argparse
import asyncio
import sys

from wuji_sdk import SdkManager, WujiGlove


def connect_device(manager, sn):
    if sn is not None:
        return manager.connect(sn=sn, device_name="example")
    return manager.auto_connect(device_name="example")


async def receive_frames(sub):
    for _ in range(5):
        frame = await sub.recv_async()
        if frame is None:
            raise RuntimeError("emf_poses stream ended")
        print(f"emf_poses: {len(frame.poses)} poses")


async def run(args):
    manager = None
    sub = None
    rate_changed = False
    result = 0
    operation = "initialize SDK"
    try:
        manager = SdkManager.instance()
        operation = "connect"
        glove = connect_device(manager, args.sn)
        operation = "check device type"
        if not isinstance(glove, WujiGlove):
            raise RuntimeError(f"expected Wuji Glove, got {type(glove).__name__}")
        operation = "read serial number"
        print(f"Connected: {glove.serial_number}")
        operation = "subscribe emf poses"
        sub = glove.emf_poses().subscribe()
        operation = "set stream rate"
        actual = sub.set_rate(30)
        rate_changed = True
        print(f"emf_poses: requested 30 Hz, device applied {actual} Hz")
        operation = "receive five emf-pose frames"
        await asyncio.wait_for(receive_frames(sub), timeout=5.0)
    except asyncio.CancelledError:
        pass
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
                    print(f"Restored default rate: {sub.set_rate(0)} Hz")
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


def main():
    parser = argparse.ArgumentParser(allow_abbrev=False)
    parser.add_argument("--sn", metavar="DEVICE_SN")
    args = parser.parse_args()
    if args.sn == "":
        parser.error("--sn must not be empty")
    result = 0

    async def execute():
        nonlocal result
        result = await run(args)

    try:
        asyncio.run(execute())
    except KeyboardInterrupt:
        pass
    return result


if __name__ == "__main__":
    sys.exit(main())
