#!/usr/bin/env python3
"""
Purpose:
    Record glove streams to an MCAP file.
Prerequisites:
    Install wuji-sdk and connect a reachable Wuji Glove.
Usage:
    python 2_recording.py --output recording.mcap
    python 2_recording.py --output recording.mcap --sn DEVICE_SN

    Options:
        --compression {lz4,zstd,none}: compression format (default: lz4).
Expected Behavior:
    Record tactile, emf_poses, and hand_skeleton for 30 seconds, pause for
    10 seconds, then record for 30 more seconds. Print live status and quality metrics,
    quality alerts when raised, and the final recording summary.
    Choose a new output path for each recording; existing files are overwritten.
    Exit 0 on completion or Ctrl-C, 1 on execution or cleanup failure,
    and 2 on invalid arguments. Errors go to stderr.
"""

from __future__ import annotations

import argparse
import asyncio
import sys

from wuji_sdk import SdkManager, WujiGlove, TopicRecorder


def connect_device(manager, sn):
    if sn is not None:
        return manager.connect(sn=sn, device_name="example")
    return manager.auto_connect(device_name="example")


async def print_status(stream):
    async for status in stream:
        print(f"Status: {status.state}, frames={status.frame_count}, duration={status.duration_s:.1f} s")


async def print_metrics(stream):
    async for metrics in stream:
        print(f"Quality: drop_rate={metrics.frame_drop_rate:.3f}, "
              f"jitter={metrics.frame_jitter_us:.1f} us, "
              f"sync_offset={metrics.sync_offset_ms:.3f} ms, sync_rate={metrics.sync_rate:.3f}")


async def print_alerts(stream):
    async for alert in stream:
        print(f"Alert: {alert.metric}, value={alert.current_value:.3f}, "
              f"threshold={alert.threshold:.3f}: {alert.message}", file=sys.stderr)


async def record_for_duration(handle):
    await asyncio.sleep(30)
    await handle.pause()
    print("Paused")
    await asyncio.sleep(10)
    await handle.resume()
    print("Resumed")
    await asyncio.sleep(30)


async def run(args):
    manager = None
    handle = None
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
        operation = "create recorder"
        recorder = TopicRecorder(compression=args.compression)
        operation = "select recording topics"
        for topic in (glove.tactile(), glove.emf_poses(), glove.hand_skeleton()):
            operation = "subscribe recording stream"
            sub = topic.subscribe()
            subscriptions.append(sub)
            operation = "add recording stream"
            recorder.record(sub)
        operation = "start recording"
        handle = await recorder.start(args.output)
        print(f"Recording to {args.output}")
        operation = "subscribe recording status"
        tasks.append(asyncio.create_task(print_status(handle.subscribe_status())))
        operation = "subscribe recording metrics"
        tasks.append(asyncio.create_task(print_metrics(handle.subscribe_metrics())))
        operation = "subscribe recording alerts"
        tasks.append(asyncio.create_task(print_alerts(handle.subscribe_alerts())))
        operation = "record and monitor streams"
        recording_task = asyncio.create_task(record_for_duration(handle))
        tasks.append(recording_task)
        done, _ = await asyncio.wait(tasks, return_when=asyncio.FIRST_COMPLETED)
        for task in done:
            await task
        if recording_task not in done:
            raise RuntimeError("recording monitor ended before recording completed")
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
        pending = asyncio.gather(*tasks, return_exceptions=True)
        try:
            outcomes = await asyncio.shield(pending)
        except asyncio.CancelledError:
            outcomes = await pending
        for outcome in outcomes:
            if isinstance(outcome, Exception) and result == 0:
                print(f"recording task failed: {outcome}", file=sys.stderr)
                result = 1
        if handle is not None:
            try:
                stop_task = asyncio.ensure_future(handle.stop())
                try:
                    summary = await asyncio.shield(stop_task)
                except asyncio.CancelledError:
                    summary = await stop_task
                print(f"Recorded {summary.total_frames} frames, {summary.file_size} bytes, "
                      f"{summary.duration_s:.1f} seconds")
                for channel in summary.channels:
                    print(f"{channel.channel_name}: {channel.frame_count} frames")
            except Exception as error:
                print(f"stop recording failed: {error}", file=sys.stderr)
                result = 1
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
    parser.add_argument("--output", required=True)
    parser.add_argument("--compression", choices=("lz4", "zstd", "none"), default="lz4")
    args = parser.parse_args()
    if args.sn == "":
        parser.error("--sn must not be empty")
    if not args.output:
        parser.error("--output must not be empty")
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
