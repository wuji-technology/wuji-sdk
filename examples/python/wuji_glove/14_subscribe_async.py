#!/usr/bin/env python3
"""
Purpose:
    Subscribe to glove data streams asynchronously.
Prerequisites:
    Install wuji-sdk and connect a reachable Wuji Glove.
Usage:
    python 14_subscribe_async.py
    python 14_subscribe_async.py --sn DEVICE_SN
Expected Behavior:
    Print tactile, tactile-zone, EMF pose, joint angle, skeleton, and
    point-cloud summaries, palm raw/fused IMU, fingertip poses,
    and global dynamic/static coordinate transforms until Ctrl-C.
    Exit 0 on completion or Ctrl-C, 1 on execution or cleanup failure,
    and 2 on invalid arguments. Errors go to stderr.
"""

from __future__ import annotations

import argparse
import sys
import asyncio

from wuji_sdk import SdkManager, WujiGlove


def avg(lst: list[float]) -> float:
    return sum(lst) / len(lst) if lst else 0.0


async def receive_tactile(sub):
    while True:
        frame = await sub.recv_async()
        if frame is None:
            raise RuntimeError("subscription ended")
        print(f"[Tactile] max={max(frame.data):.1f}")


async def receive_tactile_zones(sub):
    while True:
        zones = await sub.recv_async()
        if zones is None:
            raise RuntimeError("subscription ended")
        print(f"[Zones] palm={avg(zones.palm):.1f} thumb={avg(zones.thumb):.1f}")


async def receive_emf_poses(sub):
    while True:
        poses = await sub.recv_async()
        if poses is None:
            raise RuntimeError("subscription ended")
        if poses.poses:
            pos = poses.poses[0].pose.position
            print(f"[EmfPoses] thumb=[{pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f}]")


async def receive_hand_joint_angles(sub):
    while True:
        angles = await sub.recv_async()
        if angles is None:
            raise RuntimeError("subscription ended")
        confs = [f.confidence for f in angles.fingers]
        print(f"[JointAngles] conf={[f'{c:.2f}' for c in confs]}")


async def receive_hand_skeleton(sub):
    while True:
        skeleton = await sub.recv_async()
        if skeleton is None:
            raise RuntimeError("subscription ended")
        wrist = skeleton.joints[0]
        pos = wrist.pose.position
        print(f"[Skeleton] wrist=[{pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f}] joints={len(skeleton.joints)}")


async def receive_tactile_point_cloud(sub):
    while True:
        cloud = await sub.recv_async()
        if cloud is None:
            raise RuntimeError("subscription ended")
        print(f"[PointCloud] points={cloud.point_count()} frame={cloud.frame_id}")


async def receive_imu(sub, stream):
    while True:
        frame = await sub.recv_async()
        if frame is None:
            raise RuntimeError("subscription ended")
        a = frame.linear_acceleration
        w = frame.angular_velocity
        q = frame.orientation
        print(f"[{stream}] acceleration=[{a.x:.3f}, {a.y:.3f}, {a.z:.3f}] "
              f"angular_velocity=[{w.x:.3f}, {w.y:.3f}, {w.z:.3f}] "
              f"orientation=[{q.x:.4f}, {q.y:.4f}, {q.z:.4f}, {q.w:.4f}]")


async def receive_tip_poses(sub):
    while True:
        frame = await sub.recv_async()
        if frame is None:
            raise RuntimeError("subscription ended")
        for index, tip in enumerate(frame.poses):
            print(f"[TipPoses] finger={index} position={tip.pose.position}")


async def receive_transforms(sub, stream):
    while True:
        frame = await sub.recv_async()
        if frame is None:
            raise RuntimeError("subscription ended")
        if not frame.transforms:
            print(f"[{stream}] (no transforms)")
            continue
        transform = frame.transforms[0]
        print(f"[{stream}] count={len(frame.transforms)} "
              f"first={transform.parent_frame_id}->{transform.child_frame_id} "
              f"translation={transform.translation}")


def connect_device(manager, sn):
    if sn is not None:
        return manager.connect(sn=sn, device_name="example")
    return manager.auto_connect(device_name="example")


async def run(args):
    manager = None
    subscriptions = []
    tasks = []
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
        operation = "subscribe tactile"
        sub = glove.tactile().subscribe()
        subscriptions.append(sub)
        tasks.append(asyncio.create_task(receive_tactile(sub)))
        operation = "subscribe tactile zones"
        sub = glove.tactile_zones().subscribe()
        subscriptions.append(sub)
        tasks.append(asyncio.create_task(receive_tactile_zones(sub)))
        operation = "subscribe emf poses"
        sub = glove.emf_poses().subscribe()
        subscriptions.append(sub)
        tasks.append(asyncio.create_task(receive_emf_poses(sub)))
        operation = "subscribe hand joint angles"
        sub = glove.hand_joint_angles().subscribe()
        subscriptions.append(sub)
        tasks.append(asyncio.create_task(receive_hand_joint_angles(sub)))
        operation = "subscribe hand skeleton"
        sub = glove.hand_skeleton().subscribe()
        subscriptions.append(sub)
        tasks.append(asyncio.create_task(receive_hand_skeleton(sub)))
        operation = "subscribe tactile point cloud"
        sub = glove.tactile_point_cloud().subscribe()
        subscriptions.append(sub)
        tasks.append(asyncio.create_task(receive_tactile_point_cloud(sub)))
        operation = "subscribe imu palm"
        sub = glove.imu_palm().subscribe()
        subscriptions.append(sub)
        tasks.append(asyncio.create_task(receive_imu(sub, "imu_palm")))
        operation = "subscribe imu data palm"
        sub = glove.imu_data_palm().subscribe()
        subscriptions.append(sub)
        tasks.append(asyncio.create_task(receive_imu(sub, "imu_data_palm")))
        operation = "subscribe tip poses"
        sub = glove.tip_poses().subscribe()
        subscriptions.append(sub)
        tasks.append(asyncio.create_task(receive_tip_poses(sub)))
        operation = "subscribe tf"
        sub = manager.tf().subscribe()
        subscriptions.append(sub)
        tasks.append(asyncio.create_task(receive_transforms(sub, "tf")))
        operation = "subscribe tf static"
        sub = manager.tf_static().subscribe()
        subscriptions.append(sub)
        tasks.append(asyncio.create_task(receive_transforms(sub, "tf_static")))
        operation = "receive subscribed frames"
        await asyncio.gather(*tasks)
    except asyncio.CancelledError:
        pass
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print(f"{operation} failed: {error}", file=sys.stderr)
        result = 1
    finally:
        for task in tasks:
            task.cancel()
        await asyncio.gather(*tasks, return_exceptions=True)
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
