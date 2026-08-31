#!/usr/bin/env python3
"""
Recording example.

Connect to a Wuji Glove and record sensor data to an MCAP file.
Uses TopicRecorder with LZ4 compression. The resulting file can be
viewed with Foxglove Studio or `mcap info <file>`.

Usage: python 2.recording.py
"""

import asyncio
import os
from datetime import datetime

from wuji_sdk import SdkManager, DeviceType, TopicRecorder


async def main():
    manager = SdkManager.instance()
    devices = manager.scan()

    if not devices:
        print("No devices found")
        return

    for dev in devices:
        print(f"  SN={dev.sn}, Type={dev.device_type}, Address={dev.address}")

    gloves = [d for d in devices if d.device_type == DeviceType.WujiGlove]
    if not gloves:
        print("No Wuji Glove found among the scanned devices")
        return

    try:
        glove = manager.connect(sn=gloves[0].sn, device_name="glove_0")
        print(f"Connected: {glove.serial_number}")

        # Create a recorder with LZ4 compression
        recorder = TopicRecorder(compression="lz4")

        # Register channels — each .subscribe() feeds data into the recorder
        recorder.record(glove.tactile().subscribe())
        recorder.record(glove.emf_poses().subscribe())
        recorder.record(glove.hand_skeleton().subscribe())

        # Start recording to an MCAP file
        os.makedirs("./data", exist_ok=True)
        path = f"./data/{datetime.now().strftime('%Y%m%d_%H%M%S')}.mcap"
        print(f"Recording to {path} ...")
        handle = await recorder.start(path)

        try:
            # Record for 10 seconds, then stop
            await asyncio.sleep(10)
        finally:
            stop_task = asyncio.ensure_future(handle.stop())
            try:
                summary = await asyncio.shield(stop_task)
            except asyncio.CancelledError:
                summary = await stop_task
            print(f"Done — {summary.total_frames} frames, "
                  f"{summary.file_size / 1024 / 1024:.2f} MB, "
                  f"{summary.duration_s:.1f}s")
    finally:
        manager.disconnect_all()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\nStopped by user")
