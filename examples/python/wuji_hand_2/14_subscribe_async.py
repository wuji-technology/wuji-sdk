#!/usr/bin/env python3
"""
Purpose:
    Receive Wuji Hand 2 joint states and IMU data with async/await.
Prerequisites:
    Install wuji-sdk and make one Wuji Hand 2 reachable. Use --sn if several
    devices are discoverable.
Usage:
    python 14_subscribe_async.py
    python 14_subscribe_async.py --sn DEVICE_SN
Expected Behavior:
    Print joint NID/position pairs and IMU acceleration/angular velocity until Ctrl-C.
    No motor control. Exit 0 on completion or Ctrl-C, 1 on execution or cleanup
    failure, and 2 on invalid arguments. Errors go to stderr.
"""

import argparse
import asyncio
import sys

from wuji_sdk import SdkManager, WujiHand2


def connect_device(manager, sn):
    if sn is not None:
        return manager.connect(sn=sn, device_name="example")
    return manager.auto_connect(device_name="example")


async def receive_joint_states(sub):
    while True:
        frame = await sub.recv_async()
        if frame is None:
            raise RuntimeError("joint_states stream ended")
        print(f"seq={frame.header.seq} joints={[(joint.nid, joint.position) for joint in frame.joints]}")


async def receive_imu(sub):
    while True:
        frame = await sub.recv_async()
        if frame is None:
            raise RuntimeError("imu stream ended")
        a = frame.linear_acceleration
        w = frame.angular_velocity
        print(f"IMU seq={frame.header.seq} acceleration=[{a.x:.3f}, {a.y:.3f}, {a.z:.3f}] "
              f"angular_velocity=[{w.x:.3f}, {w.y:.3f}, {w.z:.3f}]")


async def receive_all(joint_states_sub, imu_sub):
    tasks = [asyncio.create_task(receive_joint_states(joint_states_sub)),
             asyncio.create_task(receive_imu(imu_sub))]
    try:
        await asyncio.gather(*tasks)
    finally:
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)


def main():
    parser = argparse.ArgumentParser(allow_abbrev=False)
    parser.add_argument("--sn", metavar="DEVICE_SN", help="connect to this serial number")
    args = parser.parse_args()
    if args.sn == "":
        parser.error("--sn must not be empty")

    manager = None
    subscriptions = []
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
        joint_states_sub = device.joint_states().subscribe()
        subscriptions.append(joint_states_sub)
        operation = "subscribe imu"
        imu_sub = device.imu().subscribe()
        subscriptions.append(imu_sub)
        operation = "receive subscribed frames"
        asyncio.run(receive_all(joint_states_sub, imu_sub))
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
