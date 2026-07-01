#!/usr/bin/env python3
"""
Recording example.

Connect to a Wuji Glove and record sensor data to an MCAP file.
Uses TopicRecorder with LZ4 compression. The resulting file can be
viewed with Foxglove Studio or `mcap info <file>`.

Usage:
  python 2.recording.py
  python 2.recording.py --sn <serial_number>
"""

import argparse
import asyncio
import os
from datetime import datetime

from wuji_sdk import SdkManager, TopicRecorder

from _device_selection import connect_first_glove, scan_contains


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sn", default=None, help="Optional Wuji Glove serial number.")
    return parser.parse_args()


async def main():
    args = parse_args()
    manager = SdkManager.instance()
    glove, devices = connect_first_glove(manager, sn=args.sn, device_name="glove_0")

    if not devices:
        print("No devices found")
        return
    if args.sn and not scan_contains(devices, args.sn):
        print(f"Device not found: {args.sn}")
        return
    if glove is None:
        suffix = f" matching SN={args.sn}" if args.sn else ""
        print(f"No Wuji Glove devices found{suffix}")
        return
    print(f"Connected: {glove.serial_number}")

    try:
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
