#!/usr/bin/env python3
"""
Async subscription example.

Auto-detect and connect to Wuji Gloves, subscribe to data streams
(tactile, tactile_zones, emf_poses, hand_joint_angles, hand_skeleton,
tactile_point_cloud) using async/await with recv_async().

Usage:
  python 1.subscribe_async.py
  python 1.subscribe_async.py --hand-model-path /absolute/path/to/hand.urdf
  python 1.subscribe_async.py --sn <serial_number> --hand-model-path /absolute/path/to/hand.urdf
"""

import argparse
import asyncio
from pathlib import Path
from wuji_sdk import SdkManager, DeviceType, WujiGlove, TactileFrame, TactileZones, EmfPoseArray, HandJointAngles, HandSkeleton, PointCloud


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--hand-model-path",
        type=Path,
        default=None,
        help="Optional custom hand URDF for online IK (requires a named SDK user).",
    )
    parser.add_argument("--sn", default=None, help="Optional Wuji Glove serial number.")
    args = parser.parse_args()

    if args.hand_model_path is not None:
        args.hand_model_path = args.hand_model_path.expanduser().resolve()
        if not args.hand_model_path.is_file():
            parser.error(f"--hand-model-path is not readable: {args.hand_model_path}")
    return args


async def print_tactile(glove: WujiGlove):
    sub = glove.tactile().subscribe()
    while True:
        frame: TactileFrame = await sub.recv_async()
        print(f"[{glove.device_name}][Tactile] max={max(frame.data):.1f}")


def avg(lst: list[float]) -> float:
    return sum(lst) / len(lst) if lst else 0.0


async def print_tactile_zones(glove: WujiGlove):
    sub = glove.tactile_zones().subscribe()
    while True:
        zones: TactileZones = await sub.recv_async()
        print(f"[{glove.device_name}][Zones] palm={avg(zones.palm):.1f} thumb={avg(zones.thumb):.1f}")


async def print_emf_poses(glove: WujiGlove):
    sub = glove.emf_poses().subscribe()
    while True:
        poses: EmfPoseArray = await sub.recv_async()
        if poses.poses:
            pos = poses.poses[0].pose.position
            print(f"[{glove.device_name}][EmfPoses] thumb=[{pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f}]")


async def print_hand_joint_angles(glove: WujiGlove):
    sub = glove.hand_joint_angles().subscribe()
    while True:
        angles: HandJointAngles = await sub.recv_async()
        confs = [f.confidence for f in angles.fingers]
        print(f"[{glove.device_name}][JointAngles] conf={[f'{c:.2f}' for c in confs]}")


async def print_hand_skeleton(glove: WujiGlove):
    sub = glove.hand_skeleton().subscribe()
    while True:
        skeleton: HandSkeleton = await sub.recv_async()
        wrist = skeleton.joints[0]
        pos = wrist.pose.position
        print(f"[{glove.device_name}][Skeleton] wrist=[{pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f}] joints={len(skeleton.joints)}")


async def print_tactile_point_cloud(glove: WujiGlove):
    sub = glove.tactile_point_cloud().subscribe()
    while True:
        cloud: PointCloud = await sub.recv_async()
        print(f"[{glove.device_name}][PointCloud] points={cloud.point_count()} frame={cloud.frame_id}")


async def main():
    args = parse_args()
    manager = SdkManager.instance()
    devices = manager.scan()

    if not devices:
        print("No devices found")
        return
    if args.sn:
        devices = [dev for dev in devices if dev.sn == args.sn]
        if not devices:
            print(f"Device not found: {args.sn}")
            return

    print(f"Found {len(devices)} device(s)")
    for dev in devices:
        print(f"  SN={dev.sn}, Type={dev.device_type}, Address={dev.address}")

    gloves = [d for d in devices if d.device_type == DeviceType.WujiGlove]
    if not gloves:
        print("No Wuji Glove found among the scanned devices")
        return

    tasks = []
    for i, dev in enumerate(gloves):
        glove = manager.connect(sn=dev.sn, device_name=f"glove_{i}")
        print(f"Connected: {glove.serial_number} (FW={glove.version().get()}, {glove.hand_side().get()})")
        if args.hand_model_path is not None:
            glove.hand_model_path().set(str(args.hand_model_path))
            configured_path = glove.hand_model_path().get()
            print(f"Set hand_model_path for {glove.serial_number}: {configured_path}")
        tasks.extend([
            print_tactile(glove),
            print_tactile_zones(glove),
            print_emf_poses(glove),
            print_hand_joint_angles(glove),
            print_hand_skeleton(glove),
            print_tactile_point_cloud(glove),
        ])

    print(f"Subscribed to {len(tasks)} streams. Ctrl+C to stop.\n")
    await asyncio.gather(*tasks)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
