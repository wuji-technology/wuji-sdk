#!/usr/bin/env python3
"""
Async subscription example — Wuji Hand 2.

Auto-detect and connect to Wuji Hand 2 devices, enable motors, then subscribe to
the joint_state data stream using async/await with recv_async().

Usage: python 1.subscribe_async.py
"""

import asyncio

from wuji_sdk import SdkManager, WujiHand2, JointState


async def print_joint_state(hand: WujiHand2):
    sub = hand.joint_state().subscribe()
    while True:
        state: JointState = await sub.recv_async()
        thumb = state.joints[:4]
        positions = [f"{j.actual_pos:+.3f}" for j in thumb]
        print(f"[{hand.device_name}][JointState] cycle={state.cycle_id} thumb_pos={positions}")


async def main():
    manager = SdkManager.instance()
    devices = manager.scan()

    if not devices:
        print("No devices found")
        return

    print(f"Found {len(devices)} device(s)")
    for dev in devices:
        print(f"  SN={dev.sn}, Address={dev.address}")

    hands: list[WujiHand2] = []
    tasks = []
    for i, dev in enumerate(devices):
        hand = manager.connect(sn=dev.sn, device_name=f"hand_{i}")
        print(f"Connected: {hand.serial_number} ({hand.online_joints_count().get()} joints online)")
        hand.enable()
        hands.append(hand)
        tasks.append(print_joint_state(hand))

    print(f"Subscribed to {len(tasks)} streams. Ctrl+C to stop.\n")

    try:
        await asyncio.gather(*tasks)
    finally:
        for hand in hands:
            hand.disable()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
