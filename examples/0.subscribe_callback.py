#!/usr/bin/env python3
"""
Callback subscription example.

Auto-detect and connect to Wuji Gloves, subscribe to data streams
(tactile, tactile_zones, emf_poses, hand_joint_angles, hand_skeleton,
tactile_point_cloud) using subscribe_with_callback().

Callbacks run in background, allowing non-blocking data processing.

Usage: python 0.subscribe_callback.py
"""

import time
from functools import partial
from wuji_sdk import SdkManager, TactileFrame, TactileZones, EmfPoseArray, HandJointAngles, HandSkeleton, PointCloud


def avg(lst: list[float]) -> float:
    return sum(lst) / len(lst) if lst else 0.0


def on_tactile(device_name: str, frame: TactileFrame):
    print(f"[{device_name}][Tactile] max={max(frame.data):.1f}")


def on_tactile_zones(device_name: str, zones: TactileZones):
    print(f"[{device_name}][Zones] palm={avg(zones.palm):.1f} thumb={avg(zones.thumb):.1f}")


def on_emf_poses(device_name: str, poses: EmfPoseArray):
    if poses.poses:
        pos = poses.poses[0].pose.position
        print(f"[{device_name}][EmfPoses] thumb=[{pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f}]")


def on_hand_joint_angles(device_name: str, angles: HandJointAngles):
    confs = [f.confidence for f in angles.fingers]
    print(f"[{device_name}][JointAngles] conf={[f'{c:.2f}' for c in confs]}")


def on_hand_skeleton(device_name: str, skeleton: HandSkeleton):
    wrist = skeleton.joints[0]
    pos = wrist.pose.position
    print(f"[{device_name}][Skeleton] wrist=[{pos[0]:+.3f}, {pos[1]:+.3f}, {pos[2]:+.3f}] joints={len(skeleton.joints)}")


def on_tactile_point_cloud(device_name: str, cloud: PointCloud):
    print(f"[{device_name}][PointCloud] points={cloud.point_count()} frame={cloud.frame_id}")


def main():
    manager = SdkManager.instance()
    devices = manager.scan()

    if not devices:
        print("No devices found")
        return

    print(f"Found {len(devices)} device(s)")
    for dev in devices:
        print(f"  SN={dev.sn}, Address={dev.address}")

    subscriptions = []
    for i, dev in enumerate(devices):
        glove = manager.connect(sn=dev.sn, device_name=f"glove_{i}")
        print(f"Connected: {glove.serial_number} (FW={glove.version().get()}, {glove.hand_side().get()})")
        subscriptions.append(glove.tactile().subscribe_with_callback(partial(on_tactile, glove.device_name)))
        subscriptions.append(glove.tactile_zones().subscribe_with_callback(partial(on_tactile_zones, glove.device_name)))
        subscriptions.append(glove.emf_poses().subscribe_with_callback(partial(on_emf_poses, glove.device_name)))
        subscriptions.append(glove.hand_joint_angles().subscribe_with_callback(partial(on_hand_joint_angles, glove.device_name)))
        subscriptions.append(glove.hand_skeleton().subscribe_with_callback(partial(on_hand_skeleton, glove.device_name)))
        subscriptions.append(glove.tactile_point_cloud().subscribe_with_callback(partial(on_tactile_point_cloud, glove.device_name)))

    print(f"Subscribed to {len(subscriptions)} streams. Ctrl+C to stop.\n")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        for sub in subscriptions:
            sub.close()


if __name__ == "__main__":
    main()
