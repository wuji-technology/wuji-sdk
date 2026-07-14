#!/usr/bin/env python3
"""
Async subscription example — Wuji Hand 2.

Auto-detect and connect to Wuji Hand 2 devices, then subscribe to the joint_states
data stream using async/await with recv_async(). Read-only — no motor enable.

Usage: python 1.subscribe_async.py
"""

import asyncio

from wuji_sdk import SdkManager, DeviceType, WujiHand2, JointStateFrame


async def print_joint_state(hand: WujiHand2):
    sub = hand.joint_states().subscribe()
    while True:
        frame: JointStateFrame = await sub.recv_async()
        thumb = frame.joints[:4]
        positions = [f"{j.position:+.3f}" for j in thumb]
        h = frame.header  # FrameHeader: seq / timestamp_us / frame_id
        print(
            f"[{hand.device_name}][JointState] seq={h.seq} t={h.timestamp_us}us "
            f"frame={h.frame_id!r} thumb_pos={positions}"
        )


async def main():
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
    tasks = []
    for i, dev in enumerate(hand_devices):
        hand = manager.connect(sn=dev.sn, device_name=f"hand_{i}")
        print(f"Connected: {hand.serial_number} ({hand.online_joints_count().get()} joints online)")
        hands.append(hand)
        tasks.append(print_joint_state(hand))

    print(f"Subscribed to {len(tasks)} streams. Ctrl+C to stop.\n")

    # Feedback subscription only — no motor actuation, so no enable / disable needed.
    await asyncio.gather(*tasks)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
