#!/usr/bin/env python3
"""
Callback subscription example — Wuji Hand 2.

Subscribe to the joint_states stream with subscribe_with_callback();
callbacks run in background threads.

Usage: python 0.subscribe_callback.py
"""

import time
from functools import partial

from wuji_sdk import SdkManager, DeviceType, WujiHand2, JointStateFrame


def on_joint_states(device_name: str, frame: JointStateFrame):
    # JointStateFrame: .header / .num_joints / .joints
    #   header (FrameHeader): .seq / .timestamp_us / .frame_id
    # Each JointStateEntry: .nid / .position / .velocity / .effort
    # frame.joints is a variable-length list (online joints only); use .nid to
    # identify each joint.
    thumb = frame.joints[:4]
    positions = [f"{j.position:+.3f}" for j in thumb]
    h = frame.header  # FrameHeader: common header shared across all frame types
    print(
        f"[{device_name}][JointState] seq={h.seq} t={h.timestamp_us}us "
        f"frame={h.frame_id!r} thumb_pos={positions}"
    )


def main():
    manager = SdkManager.instance()
    devices = manager.scan()

    if not devices:
        print("No devices found")
        return

    print(f"Found {len(devices)} device(s)")
    for dev in devices:
        print(f"  SN={dev.sn}, Type={dev.device_type}, Address={dev.address}")

    hand_devices = [d for d in devices if d.device_type == DeviceType.WujiHand2]
    if not hand_devices:
        print("No Wuji Hand 2 found among the scanned devices")
        return

    hands: list[WujiHand2] = []
    subscriptions = []
    try:
        for i, dev in enumerate(hand_devices):
            hand = manager.connect(sn=dev.sn, device_name=f"hand_{i}")
            print(f"Connected: {hand.serial_number} ({hand.online_joints_count().get()} joints online)")
            hands.append(hand)
            subscriptions.append(
                hand.joint_states().subscribe_with_callback(partial(on_joint_states, hand.device_name))
            )

        print(f"Subscribed to {len(subscriptions)} streams. Ctrl+C to stop.\n")

        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        for sub in subscriptions:
            sub.close()
        manager.disconnect_all()


if __name__ == "__main__":
    main()
