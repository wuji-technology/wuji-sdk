#!/usr/bin/env python3
"""
Wuji Glove hand model calibration example.

This example uses the SDK's remembered current local user. To create, inspect,
or switch users, run 4.user.py first.

Calibration produces one hand model per side for the current SDK user
(left_hand.urdf / right_hand.urdf). Any glove of the same side connected under
the same user loads the same model.

Usage:
  python 5.calibration.py --sn <serial_number>
  python 5.calibration.py --mode api --sn <serial_number>
"""

import argparse
import asyncio
import json
import shutil
import sys
import time
from typing import Any

from wuji_sdk import DeviceType, SdkManager, WujiException, set_log_level


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sn", default=None, help="Optional Wuji Glove serial number.")
    parser.add_argument("--device-name", default="my_glove")
    parser.add_argument(
        "--mode",
        choices=["terminal", "api"],
        default="terminal",
        help="terminal shows an optional prompt screen; api shows callback/result flow.",
    )
    parser.add_argument("--blocking", action="store_true", help="Use calibrate_blocking().")
    parser.add_argument("--dry-run", action="store_true", help="Print current SDK user and exit.")
    parser.add_argument("--skip-constraints", action="store_true")
    parser.add_argument("--timeout-s", type=float, default=900.0)
    parser.add_argument(
        "--log-level",
        default="warn",
        choices=["trace", "debug", "info", "warn", "warning", "error", "off"],
    )
    return parser.parse_args()


def print_current_user(manager: SdkManager) -> None:
    user = manager.current_user()
    print("current SDK user:")
    print(f"  user_id:      {user.get('user_id')!r}")
    print(f"  display_name: {user.get('display_name')}")
    print(f"  description:  {user.get('description')}")
    print(f"  is_default:   {user.get('is_default')}")
    print("\nCalibration results are stored per SDK user and hand side.")
    if user.get("is_default"):
        # Calibration requires a named user profile; guide before the error hits.
        print(
            "\nYou are on the default SDK user, which cannot run calibration.\n"
            "Create and switch to a user profile first, e.g.:\n"
            '    user = manager.create_user("user1")\n'
            '    manager.switch_user(user["user_id"])\n'
            "or run 4.user.py to manage profiles, then re-run this example."
        )


def connect_glove(manager: SdkManager, args: argparse.Namespace):
    if args.sn:
        print(f"Connecting to Wuji Glove SN={args.sn}...")
        return manager.connect(sn=args.sn, device_name=args.device_name)
    print("Scanning for Wuji Glove...")
    devices = manager.scan()
    for dev in devices:
        print(f"  SN={dev.sn}, Type={dev.device_type}, Address={dev.address}")
    gloves = [d for d in devices if d.device_type == DeviceType.WujiGlove]
    if not gloves:
        raise SystemExit("No Wuji Glove found among the scanned devices")
    return manager.connect(sn=gloves[0].sn, device_name=args.device_name)


def print_result(result: dict[str, Any]) -> None:
    print("\nCalibration result:")
    print(json.dumps(result, indent=2, ensure_ascii=False, default=str))


# =============================================================================
# SDK API usage: this is the part applications usually copy
# =============================================================================


def calibration_options(args: argparse.Namespace, on_feedback) -> dict[str, Any]:
    return {
        "skip_constraints": args.skip_constraints,
        "timeout_s": args.timeout_s,
        "on_feedback": on_feedback,
    }


async def call_calibrate(glove, args: argparse.Namespace, on_feedback) -> dict[str, Any]:
    return await glove.calibrate(**calibration_options(args, on_feedback))


def call_calibrate_blocking(glove, args: argparse.Namespace, on_feedback) -> dict[str, Any]:
    return glove.calibrate_blocking(**calibration_options(args, on_feedback))


async def run_api_async(manager: SdkManager, args: argparse.Namespace) -> None:
    glove = connect_glove(manager, args)
    result = await call_calibrate(glove, args, ApiProgress())
    print_result(result)


def run_api_blocking(manager: SdkManager, args: argparse.Namespace) -> None:
    glove = connect_glove(manager, args)
    result = call_calibrate_blocking(glove, args, ApiProgress())
    print_result(result)


# =============================================================================
# API callback helpers: small text output for user-owned integrations
# =============================================================================


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


# =============================================================================
# Optional terminal prompt: demo UI only, not required for SDK integration
# =============================================================================


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


async def run_terminal_async(manager: SdkManager, args: argparse.Namespace) -> None:
    glove = connect_glove(manager, args)
    progress = TerminalProgress()
    progress.start()
    try:
        result = await call_calibrate(glove, args, progress)
    finally:
        progress.stop()
    print_result(result)


def run_terminal_blocking(manager: SdkManager, args: argparse.Namespace) -> None:
    glove = connect_glove(manager, args)
    progress = TerminalProgress()
    progress.start()
    try:
        result = call_calibrate_blocking(glove, args, progress)
    finally:
        progress.stop()
    print_result(result)


# =============================================================================
# CLI dispatch
# =============================================================================


def main() -> None:
    args = parse_args()
    set_log_level(args.log_level)
    manager = SdkManager.instance()

    try:
        print_current_user(manager)
        if args.dry_run:
            print("\nDry run complete: no device connection or calibration was performed.")
            return
        if args.mode == "api":
            if args.blocking:
                run_api_blocking(manager, args)
            else:
                asyncio.run(run_api_async(manager, args))
        elif args.blocking:
            run_terminal_blocking(manager, args)
        else:
            asyncio.run(run_terminal_async(manager, args))
    except KeyboardInterrupt:
        print("\nCalibration cancelled.")
        raise SystemExit(130)
    except (WujiException, RuntimeError) as e:
        print(f"Error: {e}")
        raise SystemExit(1) from e
    finally:
        manager.disconnect_all()


if __name__ == "__main__":
    main()
