#!/usr/bin/env python3
"""
Purpose:
    Calibrate a glove hand model.
Prerequisites:
    Install wuji-sdk, connect a Wuji Glove, and select a named SDK user.
Usage:
    python 5_calibration.py
    python 5_calibration.py --sn DEVICE_SN
    python 5_calibration.py --mode api

    Options:
        --mode {terminal,api}: display mode (default: terminal).
        --skip-constraints: skip pose constraints (default: check constraints).
        --timeout-s SECONDS: calibration timeout (default: 900).
Expected Behavior:
    Show pose instructions, progress, and adjustment hints in a terminal screen.
    Use --mode api for scrolling text; redirected output also uses text.
    Print the saved hand model after calibration.
    Calibration uses the current SDK user and stores one model per hand side.
    Exit 0 on completion or Ctrl-C, 1 on execution or cleanup failure,
    and 2 on invalid arguments. Errors go to stderr.
"""

from __future__ import annotations

import argparse
import asyncio
import json
import math
import shutil
import sys
import time
from typing import Any

from wuji_sdk import SdkManager, WujiGlove


class ApiProgress:
    def __init__(self) -> None:
        self.last_line = ""
        self.last_state: tuple[int, str, str] | None = None
        self.last_errors: list[str] = []
        self.last_print = 0.0

    def __call__(self, fb: dict[str, Any]) -> None:
        pose = str(fb.get("step_name", ""))
        state = str(fb.get("state", ""))
        step = int(fb.get("step_index", 0) or 0) + 1
        total = int(fb.get("step_total", 0) or 0)

        if state == "done":
            suffix = "finishing calibration" if step >= total else "open palm before the next pose"
            print(f"[{step}/{total}] collected {pose}; {suffix}")
            return
        if state == "waiting_movement":
            print(f"[next {step}/{total}] open palm, then move into {pose}")
            return

        line = (
            f"[{step}/{total}] {pose} {state} {progress_bar(progress_value(fb))} "
            f"overall {overall_count(fb)} {elapsed_text(fb)}"
        ).strip()
        frames = fb.get("frames_collected")
        if frames is not None:
            line += f" frames={frames}"

        self.print_progress(line, (step, pose, state))
        errors = diagnostic_messages(fb)
        if errors and errors != self.last_errors:
            print("Adjust:")
            for error in errors[:8]:
                print(f"  {error}")
        self.last_errors = errors

    def print_progress(self, line: str, state: tuple[int, str, str]) -> None:
        now = time.monotonic()
        if line == self.last_line or (state == self.last_state and now - self.last_print < 1.0):
            return
        self.last_line = line
        self.last_state = state
        self.last_print = now
        print(line)


def progress_value(fb: dict[str, Any]) -> float:
    try:
        if "progress" in fb:
            return max(0.0, min(1.0, float(fb["progress"])))
    except (TypeError, ValueError):
        return 0.0

    state = fb.get("state")
    if state == "done":
        return 1.0

    elapsed_key, target_key = {
        "collecting": ("collect_elapsed", "collect_target"),
        "waiting_stable": ("hold_elapsed", "hold_target"),
    }.get(state, ("", ""))
    try:
        target = float(fb.get(target_key, 1.0))
        return max(0.0, min(1.0, float(fb.get(elapsed_key, 0.0)) / target)) if target > 0 else 0.0
    except (TypeError, ValueError):
        return 0.0


def progress_bar(value: float, width: int = 18) -> str:
    done = int(round(max(0.0, min(1.0, value)) * width))
    return "[" + "=" * min(done, width) + ("." * max(0, width - done)) + "]"


def overall_count(fb: dict[str, Any]) -> str:
    total = int(fb.get("step_total", 0) or 0)
    step = int(fb.get("step_index", 0) or 0)
    done = step + 1 if fb.get("state") == "done" else step
    return f"{min(done, total)}/{total}" if total else "0/0"


def elapsed_text(fb: dict[str, Any]) -> str:
    fields = {
        "collecting": ("collect_elapsed", "collect_target"),
        "waiting_stable": ("hold_elapsed", "hold_target"),
    }.get(fb.get("state"))
    if not fields:
        return ""
    try:
        return f"{float(fb.get(fields[0], 0.0)):.1f}/{float(fb.get(fields[1], 0.0)):.1f}s"
    except (TypeError, ValueError):
        return ""


def diagnostic_messages(fb: dict[str, Any]) -> list[str]:
    messages = []
    if fb.get("variance_ok") is False:
        messages.append("motion: hand is moving too much; hold wrist and fingers still")

    metrics = fb.get("metrics") or []
    grouped: dict[str, list[str]] = {}
    for metric in metrics if isinstance(metrics, list) else []:
        if not isinstance(metric, dict):
            continue
        label = str(metric.get("label", "metric"))
        unit = str(metric.get("unit", ""))
        scale = 1000.0 if unit == "m" else 1.0
        unit = "mm" if unit == "m" else unit
        error = float(metric.get("error", 0.0) or 0.0) * scale
        if error <= 0.0001:
            continue
        finger = metric.get("finger") or "global"
        finger_b = metric.get("finger_b")
        group = f"{finger}-{finger_b}" if finger_b else str(finger)
        hint = metric.get("hint", "adjust pose")
        grouped.setdefault(group, []).append(f"{label}: off by {error:.1f}{unit}; {hint}")

    for group in sorted(grouped):
        messages.append(f"{group}:")
        messages.extend(f"  {message}" for message in grouped[group])
    if fb.get("constraints_ok") is False and not grouped:
        messages.append("pose: not inside target; adjust the current pose")
    return messages


POSE_GUIDES = {
    "flat_open": ("Flat open hand", "Straighten all fingers, palm flat, wrist still."),
    "pinch_index": ("Pinch thumb + index", "Touch thumb tip to index fingertip."),
    "pinch_middle": ("Pinch thumb + middle", "Touch thumb tip to middle fingertip."),
    "pinch_ring": ("Pinch thumb + ring", "Touch thumb tip to ring fingertip."),
    "pinch_pinky": ("Pinch thumb + pinky", "Touch thumb tip to pinky fingertip."),
    "four_finger_bend_90": ("Four-finger 90-degree bend", "Bend four fingers about 90 degrees."),
    "thumb_to_index_dip": ("Thumb presses index DIP base", "Press thumb near the index DIP joint base."),
    "index_to_thumb_emf": ("Index points to thumb EMF side", "Point index fingertip toward thumb sensor side."),
}


def pose_text(name: str) -> tuple[str, str]:
    return POSE_GUIDES.get(name, (name.replace("_", " ").title(), "Move into the pose and hold still."))


class TerminalProgress:
    def __init__(self) -> None:
        self.enabled = sys.stdout.isatty()
        self.fallback = ApiProgress()
        self.active = False
        self.last_render = 0.0
        self.last_signature = ""

    def start(self) -> None:
        if not self.enabled or self.active:
            return
        sys.stdout.write("\033[?1049h\033[?25l\033[2J\033[H")
        sys.stdout.flush()
        self.active = True

    def stop(self) -> None:
        if not self.active:
            return
        sys.stdout.write("\033[0m\033[?25h\033[?1049l")
        sys.stdout.flush()
        self.active = False

    def __call__(self, fb: dict[str, Any]) -> None:
        if not self.enabled:
            self.fallback(fb)
            return

        state = str(fb.get("state", ""))
        pose = str(fb.get("step_name", ""))
        errors = diagnostic_messages(fb)
        signature = "\n".join([
            pose,
            state,
            overall_count(fb),
            progress_bar(progress_value(fb)),
            elapsed_text(fb),
            "\n".join(errors),
        ])
        now = time.monotonic()
        if signature == self.last_signature:
            return
        if state not in {"done", "waiting_movement"} and now - self.last_render < 0.1:
            return
        self.last_signature = signature
        self.last_render = now
        self.render(fb, errors)

    def render(self, fb: dict[str, Any], errors: list[str]) -> None:
        width = min(shutil.get_terminal_size((100, 28)).columns, 100)
        sep = "-" * width
        pose = str(fb.get("step_name", ""))
        state = str(fb.get("state", ""))
        step = int(fb.get("step_index", 0) or 0) + 1
        total = int(fb.get("step_total", 0) or 0)
        title, guide = pose_text(pose)

        print("\033[2J\033[H", end="")
        print("Wuji Glove Hand Model Calibration")
        print(sep)

        if state == "done":
            print(f"Collected {step}/{total}: {title}")
            print()
            print("Final pose collected. Finishing calibration..." if step >= total else "Open your palm before the next pose.")
            print(sep)
            print("Keep the wrist relaxed. Move only when the next pose appears.")
            sys.stdout.flush()
            return

        if state == "waiting_movement":
            print(f"Next {step}/{total}: {title}")
            print()
            print("Open your palm, then move into the next pose.")
            print(f"Pose guide: {guide}")
            print(sep)
            print("Move only after your palm is fully open.")
            sys.stdout.flush()
            return

        print(f"Pose {step}/{total}: {title}")
        print(f"Guide: {guide}")
        print(f"State: {state}")
        print(f"Overall: {overall_count(fb)}")
        print(f"Current: {progress_bar(progress_value(fb), width=28)} {elapsed_text(fb)}")
        frames = fb.get("frames_collected")
        if frames is not None:
            print(f"Frames: {frames}")
        print(sep)
        if errors:
            print("Adjust:")
            for error in errors[:8]:
                print(f"  {error}")
        else:
            print("OK - keep holding still.")
        print(sep)
        print("Press Ctrl+C to cancel.")
        sys.stdout.flush()


def connect_device(manager, sn):
    if sn is not None:
        return manager.connect(sn=sn, device_name="example")
    return manager.auto_connect(device_name="example")


async def run(args):
    manager = None
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
        operation = "read current SDK user"
        user = manager.current_user()
        if user["is_default"]:
            raise RuntimeError("select a named SDK user before calibration")
        print(f"SDK user: {user['display_name']}")
        operation = "calibrate hand model"
        progress = TerminalProgress() if args.mode == "terminal" else ApiProgress()
        try:
            if args.mode == "terminal":
                progress.start()
            summary = await glove.calibrate(skip_constraints=args.skip_constraints,
                                            timeout_s=args.timeout_s, on_feedback=progress)
        finally:
            if args.mode == "terminal":
                progress.stop()
        print(json.dumps(summary, indent=2, default=str))
    except asyncio.CancelledError:
        pass
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print(f"{operation} failed: {error}", file=sys.stderr)
        result = 1
    finally:
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
    parser.add_argument("--mode", choices=["terminal", "api"], default="terminal")
    parser.add_argument("--skip-constraints", action="store_true")
    parser.add_argument("--timeout-s", type=float, default=900.0)
    args = parser.parse_args()
    if args.sn == "":
        parser.error("--sn must not be empty")
    if not math.isfinite(args.timeout_s) or args.timeout_s <= 0:
        parser.error("--timeout-s must be positive and finite")
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
