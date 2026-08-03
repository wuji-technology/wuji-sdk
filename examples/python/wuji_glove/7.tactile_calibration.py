#!/usr/bin/env python3
"""
Wuji Glove tactile calibration example.

This example shows the stable v1 happy path: a terminal-guided calibration that
records the required motions, trains a contact model, optionally installs it,
and prints the final summary. The model is keyed by glove serial number and is
loaded automatically at runtime after installation.

The training step requires an SDK build with the model-export feature.

Usage:
  python 7.tactile_calibration.py --sn <serial_number>
"""

import argparse
import json
import sys
from typing import Any

from wuji_sdk import ConnectOptions, DeviceType, SdkManager, WujiException, set_log_level

MOTION_TEXT = {
    "four_finger_L_thumb_in": "Curl four fingers into an L, thumb tucked in; vary the pressure.",
    "claw_curl": "Curl all fingers into a claw and flex repeatedly.",
    "abduction": "Spread and close the fingers repeatedly.",
    "finger_to_palm": "Press each fingertip to the palm in turn.",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sn", default=None, help="Optional Wuji Glove serial number.")
    parser.add_argument("--device-name", default="my_glove")
    parser.add_argument("--seconds-per-pose", type=float, default=10.0)
    parser.add_argument("--epochs", type=int, default=60)
    parser.add_argument(
        "--no-install",
        action="store_true",
        help="Train without committing the model to the runtime directory.",
    )
    parser.add_argument("--sensitivity", type=float, default=None)
    parser.add_argument("--timeout-s", type=float, default=1800.0)
    parser.add_argument(
        "--log-level",
        default="warn",
        choices=["trace", "debug", "info", "warn", "warning", "error", "off"],
    )
    return parser.parse_args()


def connect_glove(manager: SdkManager, args: argparse.Namespace):
    # Calibration owns one local device session; it does not need to advertise
    # the device through the optional multi-process Zenoh bridge.
    options = ConnectOptions(enable_bridge=False)
    if args.sn:
        print(f"Connecting to Wuji Glove SN={args.sn}...")
        return manager.connect(sn=args.sn, device_name=args.device_name, options=options)

    print("Scanning for Wuji Glove...")
    devices = manager.scan()
    for dev in devices:
        print(f"  SN={dev.sn}, Type={dev.device_type}, Address={dev.address}")
    gloves = [d for d in devices if d.device_type == DeviceType.WujiGlove]
    if not gloves:
        raise SystemExit("No Wuji Glove found among the scanned devices")
    return manager.connect(sn=gloves[0].sn, device_name=args.device_name, options=options)


def print_intro(sn: str, args: argparse.Namespace) -> None:
    print("\n" + "=" * 72)
    print("Wuji Glove tactile calibration")
    print("=" * 72)
    print(f"  serial number:     {sn}")
    print(f"  seconds per pose:  {args.seconds_per_pose:.1f}")
    print(f"  epochs:            {args.epochs}")
    print(f"  install model:     {not args.no_install}")
    print("\nYou will be asked to confirm before each motion and then keep, redo, or abort after it.")


def _format_recording(feedback: dict[str, Any]) -> str:
    elapsed = feedback.get("collect_elapsed", 0.0)
    target = feedback.get("collect_target", 0.0)
    frames = feedback.get("frames_collected", 0)
    return f"  recording: {elapsed:5.1f}/{target:5.1f}s frames={frames}"


def _format_train(feedback: dict[str, Any]) -> str:
    epoch = feedback.get("epoch", 0)
    total = feedback.get("epoch_total", 0)
    best = feedback.get("best_val")
    suffix = f" best_val={best:.4f}" if best is not None else ""
    return f"  train: epoch {epoch}/{total}{suffix}"


class TerminalProgress:
    def __init__(self) -> None:
        self._last_state = ""
        self._line_open = False

    def _close_line(self) -> None:
        if self._line_open:
            print()
            self._line_open = False

    def __call__(self, feedback: dict[str, Any]) -> None:
        state = feedback.get("state", "")
        if state == "collect" and "collect_elapsed" in feedback:
            print("\r" + _format_recording(feedback), end="")
            self._line_open = True
            sys.stdout.flush()
            return
        if state == "train" and feedback.get("epoch", 0) >= 1:
            if self._last_state != "train":
                self._close_line()
                print("\nStage: train")
            print("\r" + _format_train(feedback), end="")
            self._line_open = True
            sys.stdout.flush()
            self._last_state = state
            return
        self._close_line()
        if state in {"check", "install", "verify", "done"}:
            print(f"Stage: {state}")
        self._last_state = state


class TerminalPrompt:
    def __call__(self, prompt: dict[str, Any]) -> str | None:
        kind = prompt.get("kind")
        step = int(prompt.get("step_index", 0)) + 1
        total = int(prompt.get("step_total", 0))
        name = str(prompt.get("step_name", "motion"))

        if kind == "pose_ready":
            print("\n" + "=" * 72)
            print(f"Motion {step}/{total}: {name}")
            print("=" * 72)
            print("  " + MOTION_TEXT.get(name, "Follow the motion guide."))
            input("Press Enter when you are ready to record this motion...")
            return "proceed"

        if kind == "pose_review":
            warnings = prompt.get("warnings") or []
            print("\n" + "=" * 72)
            print(f"Review motion {step}/{total}: {name}")
            print("=" * 72)
            if warnings:
                print("Quality warnings:")
                for warning in warnings:
                    print(f"  - {warning}")
            while True:
                answer = input("Keep this motion? [Enter/r/q] ").strip().lower()
                if answer in {"", "y", "yes", "k", "keep"}:
                    return "proceed"
                if answer in {"r", "redo", "retry"}:
                    return "retry"
                if answer in {"q", "quit", "abort", "stop"}:
                    return "abort"
                print("Please press Enter to keep, r to redo, or q to stop.")

        return "proceed"


def calibration_options(args: argparse.Namespace) -> dict[str, Any]:
    options: dict[str, Any] = {
        "seconds_per_pose": args.seconds_per_pose,
        "epochs": args.epochs,
        "install": not args.no_install,
        "timeout_s": args.timeout_s,
        "on_feedback": TerminalProgress(),
        "on_pose_prompt": TerminalPrompt(),
    }
    if args.sensitivity is not None:
        options["sensitivity"] = args.sensitivity
    return options


def print_result(result: dict[str, Any]) -> None:
    print("\n" + "=" * 72)
    print("Tactile calibration summary")
    print("=" * 72)
    print(f"  poses collected: {result.get('poses_collected')}")
    print(f"  epochs:          {result.get('epochs')}")
    print(f"  best val loss:   {result.get('best_val_loss')}")
    print(f"  alive taxels:    {result.get('alive_taxels')}")
    print(f"  dead pct:        {result.get('dead_pct')}")
    print(f"  installed:       {result.get('installed')}")
    if result.get("model_dir"):
        print(f"  model dir:       {result['model_dir']}")
    if result.get("verified_alive_taxels") is not None:
        print(f"  verified taxels: {result['verified_alive_taxels']}")
    print("\nRaw result:")
    print(json.dumps(result, indent=2, ensure_ascii=True, default=str))


def main() -> int:
    args = parse_args()
    set_log_level(args.log_level)
    manager = SdkManager.instance()
    glove = connect_glove(manager, args)
    sn = args.sn or glove.serial_number
    print_intro(sn, args)

    try:
        result = glove.calibrate_tactile_blocking(**calibration_options(args))
    except WujiException as exc:
        print(f"\nTactile calibration failed: {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\nTactile calibration interrupted.", file=sys.stderr)
        return 130

    print_result(result)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
