#!/usr/bin/env python3
"""
Purpose:
    Calibrate the glove tactile contact model.
Prerequisites:
    Install a wuji-sdk build with model training support on Linux or macOS.
    Connect a Wuji Glove and prepare to perform the prompted motions.
Usage:
    python 7_tactile_calibration.py
    python 7_tactile_calibration.py --sn DEVICE_SN

    Options:
        --seconds-per-pose SECONDS: recording time per motion (default: 10).
        --epochs N: training epochs (default: 60).
        --no-install: train without installing the model (default: install).
        --sensitivity VALUE: optional sensitivity override.
        --timeout-s SECONDS: calibration timeout (default: 1800).
Expected Behavior:
    Confirm each motion, then keep it or retry. Train and install the model
    unless --no-install is set, and print the calibration summary.
    Exit 0 on completion or Ctrl-C, 1 on execution or cleanup failure,
    and 2 on invalid arguments. Errors go to stderr.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import select
import sys
import threading

from wuji_sdk import SdkManager, WujiGlove, WujiException


MOTION_TEXT = {
    "four_finger_L_thumb_in": "Curl four fingers into an L, thumb tucked in; vary the pressure.",
    "claw_curl": "Curl all fingers into a claw and flex repeatedly.",
    "abduction": "Spread and close the fingers repeatedly.",
    "finger_to_palm": "Press each fingertip to the palm in turn.",
}


class Prompt:
    def __init__(self):
        self.stopping = threading.Event()
        self.aborted = False

    def read(self, message):
        print(message, end="", flush=True)
        while not self.stopping.is_set():
            if select.select([sys.stdin], [], [], 0.1)[0]:
                line = sys.stdin.readline()
                if not line:
                    raise EOFError("terminal input ended")
                return line.strip().lower()
        return "q"

    def __call__(self, prompt):
        name = prompt.get("step_name", "motion")
        if prompt["kind"] == "pose_ready":
            print(f"Motion {prompt.get('step_index', 0) + 1}/{prompt.get('step_total', 0)}: "
                  f"{MOTION_TEXT.get(name, name)}")
            answer = self.read("Press Enter when ready, or q to stop: ")
        else:
            for warning in prompt.get("warnings", []):
                print(f"  {warning}")
            answer = self.read("Keep this motion? [Enter/r/q] ")
            while answer not in ("", "r", "q"):
                answer = self.read("Press Enter to keep, r to retry, or q to stop: ")
        if answer == "q":
            self.aborted = True
            return "abort"
        return "retry" if answer == "r" else "proceed"


def on_feedback(feedback):
    state = feedback.get("state", "")
    if state == "collect":
        print(f"Collect: {feedback.get('collect_elapsed', 0):.1f}/"
              f"{feedback.get('collect_target', 0):.1f}s, "
              f"{feedback.get('frames_collected', 0)} frames")
    elif state == "train":
        print(f"Train: epoch {feedback.get('epoch', 0)}/{feedback.get('epoch_total', 0)}")
    else:
        print(f"Stage: {state}")


def connect_device(manager, sn):
    if sn is not None:
        return manager.connect(sn=sn, device_name="example")
    return manager.auto_connect(device_name="example")


async def run(args):
    manager = None
    task = None
    prompt = Prompt()
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
        operation = "calibrate tactile model"
        task = asyncio.ensure_future(glove.calibrate_tactile(
            seconds_per_pose=args.seconds_per_pose, epochs=args.epochs,
            install=not args.no_install, sensitivity=args.sensitivity,
            timeout_s=args.timeout_s, on_feedback=on_feedback, on_pose_prompt=prompt))
        summary = await asyncio.shield(task)
        print(json.dumps(summary, indent=2, default=str))
    except asyncio.CancelledError:
        prompt.stopping.set()
        if task is not None:
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
            except Exception as error:
                print(f"cancel calibration failed: {error}", file=sys.stderr)
                result = 1
    except WujiException as error:
        if not prompt.aborted:
            print(f"{operation} failed: {error}", file=sys.stderr)
            result = 1
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print(f"{operation} failed: {error}", file=sys.stderr)
        result = 1
    finally:
        prompt.stopping.set()
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
    parser.add_argument("--seconds-per-pose", type=float, default=10.0)
    parser.add_argument("--epochs", type=int, default=60)
    parser.add_argument("--no-install", action="store_true")
    parser.add_argument("--sensitivity", type=float)
    parser.add_argument("--timeout-s", type=float, default=1800.0)
    args = parser.parse_args()
    if args.sn == "":
        parser.error("--sn must not be empty")
    if any(not math.isfinite(value) or value <= 0 for value in (args.seconds_per_pose, args.timeout_s)):
        parser.error("durations must be positive and finite")
    if args.epochs < 1 or args.epochs > 4294967295:
        parser.error("--epochs must be in 1..4294967295")
    if args.sensitivity is not None and (not math.isfinite(args.sensitivity) or args.sensitivity <= 0):
        parser.error("--sensitivity must be positive and finite")
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
