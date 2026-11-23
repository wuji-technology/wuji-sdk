#!/usr/bin/env python3
"""
Purpose:
    Subscribe to glove data streams with callbacks.
Prerequisites:
    Install wuji-sdk and connect a reachable Wuji Glove.
Usage:
    python 1_subscribe_callback.py
    python 1_subscribe_callback.py --sn DEVICE_SN
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
import time
from functools import partial

from wuji_sdk import (SdkManager, WujiGlove, TactileFrame, TactileZones,
                      EmfPoseArray, HandJointAngles, HandSkeleton, PointCloud)


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


def on_imu(device_name, stream, frame):
    a = frame.linear_acceleration
    w = frame.angular_velocity
    q = frame.orientation
    print(f"[{device_name}][{stream}] acceleration=[{a.x:.3f}, {a.y:.3f}, {a.z:.3f}] "
          f"angular_velocity=[{w.x:.3f}, {w.y:.3f}, {w.z:.3f}] "
          f"orientation=[{q.x:.4f}, {q.y:.4f}, {q.z:.4f}, {q.w:.4f}]")


def on_tip_poses(device_name, frame):
    for index, tip in enumerate(frame.poses):
        print(f"[{device_name}][TipPoses] finger={index} position={tip.pose.position}")


def on_transforms(stream, frame):
    if not frame.transforms:
        print(f"[{stream}] (no transforms)")
        return
    transform = frame.transforms[0]
    print(f"[{stream}] count={len(frame.transforms)} "
          f"first={transform.parent_frame_id}->{transform.child_frame_id} "
          f"translation={transform.translation}")


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
    subscriptions = []
    callback_errors = []
    result = 0

    def callback(render, frame):
        try:
            render(frame)
        except Exception as error:
            callback_errors.append(error)

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
        subscriptions.append(glove.tactile().subscribe_with_callback(
            partial(callback, partial(on_tactile, glove.device_name))))
        operation = "subscribe tactile zones"
        subscriptions.append(glove.tactile_zones().subscribe_with_callback(
            partial(callback, partial(on_tactile_zones, glove.device_name))))
        operation = "subscribe emf poses"
        subscriptions.append(glove.emf_poses().subscribe_with_callback(
            partial(callback, partial(on_emf_poses, glove.device_name))))
        operation = "subscribe hand joint angles"
        subscriptions.append(glove.hand_joint_angles().subscribe_with_callback(
            partial(callback, partial(on_hand_joint_angles, glove.device_name))))
        operation = "subscribe hand skeleton"
        subscriptions.append(glove.hand_skeleton().subscribe_with_callback(
            partial(callback, partial(on_hand_skeleton, glove.device_name))))
        operation = "subscribe tactile point cloud"
        subscriptions.append(glove.tactile_point_cloud().subscribe_with_callback(
            partial(callback, partial(on_tactile_point_cloud, glove.device_name))))
        operation = "subscribe imu palm"
        subscriptions.append(glove.imu_palm().subscribe_with_callback(
            partial(callback, partial(on_imu, glove.device_name, "imu_palm"))))
        operation = "subscribe imu data palm"
        subscriptions.append(glove.imu_data_palm().subscribe_with_callback(
            partial(callback, partial(on_imu, glove.device_name, "imu_data_palm"))))
        operation = "subscribe tip poses"
        subscriptions.append(glove.tip_poses().subscribe_with_callback(
            partial(callback, partial(on_tip_poses, glove.device_name))))
        operation = "subscribe tf"
        subscriptions.append(manager.tf().subscribe_with_callback(
            partial(callback, partial(on_transforms, "tf"))))
        operation = "subscribe tf static"
        subscriptions.append(manager.tf_static().subscribe_with_callback(
            partial(callback, partial(on_transforms, "tf_static"))))
        operation = "receive subscribed frames"
        while True:
            if callback_errors:
                raise callback_errors[0]
            if any(not sub.is_active or sub.error_count for sub in subscriptions):
                raise RuntimeError("subscription ended")
            time.sleep(0.1)
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
