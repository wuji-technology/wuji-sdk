#!/usr/bin/env python3
"""
Async subscription example.

Auto-detect and connect to Wuji Gloves, subscribe to data streams
(tactile, tactile_zones, emf_poses) using async/await with recv_async().

Usage: python 1.subscribe_async.py
"""

import asyncio
from wuji_sdk import SdkManager, WujiGlove, TactileFrame, TactileZones, EmfPoseArray


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


async def main():
    manager = SdkManager.instance()
    devices = manager.scan()

    if not devices:
        print("No devices found")
        return

    print(f"Found {len(devices)} device(s)")
    for dev in devices:
        print(f"  SN={dev.sn}, Address={dev.address}")

    tasks = []
    for i, dev in enumerate(devices):
        glove = manager.connect(sn=dev.sn, device_name=f"glove_{i}")
        print(f"Connected: {glove.serial_number} (FW={glove.version().get()}, {glove.hand_side().get()})")
        tasks.extend([
            print_tactile(glove),
            print_tactile_zones(glove),
            print_emf_poses(glove),
        ])

    print(f"Subscribed to {len(tasks)} streams. Ctrl+C to stop.\n")
    await asyncio.gather(*tasks)


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
