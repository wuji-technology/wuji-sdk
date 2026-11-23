#!/usr/bin/env python3
"""
Purpose:
    View live tactile residual data.
Prerequisites:
    Install wuji-sdk on Linux or macOS and connect a calibrated Wuji Glove.
Usage:
    python 9_tactile_residual_view.py
    python 9_tactile_residual_view.py --sn DEVICE_SN

    Options:
        --seconds SECONDS: run duration (default: 0, until q or Ctrl-C).
        --threshold VALUE: initial display threshold (default: 0.5).
Expected Behavior:
    Render a 24x31 terminal grid. Use + / - to adjust threshold, q to quit,
    or --seconds to set a duration (0 waits for q or Ctrl-C).
    Exit 0 on completion or Ctrl-C, 1 on execution or cleanup failure,
    and 2 on invalid arguments. Errors go to stderr.
"""

from __future__ import annotations

import argparse
import math
import select
import sys
import termios
import time
import tty

from wuji_sdk import SdkManager, WujiGlove, TactileResidual


TACTILE_ROWS = 24
TACTILE_COLS = 31
MASKED_VALUE = -1.0  # invalid/masked sentinel
MASKED_EPS = 1e-6
THRESHOLD_STEP = 0.1
RENDER_INTERVAL_S = 0.1
RAMP = ".:-=+*"  # increasing residual magnitude below the threshold


def clamp_threshold(value: float) -> float:
    return max(0.0, float(value))


def cell_for(value: float, threshold: float) -> str:
    """Map a residual taxel to a glyph; the consumer owns the threshold."""
    if abs(value - MASKED_VALUE) < MASKED_EPS:
        return " "
    if threshold > 0.0 and value >= threshold:
        return "#"
    if value <= 0.0 or threshold <= 0.0:
        return "."
    idx = int(value / threshold * len(RAMP))
    return RAMP[min(idx, len(RAMP) - 1)]


class ResidualView:
    """Renders tactile_residual frames as a grid; throttled to ~10 Hz."""

    def __init__(self, threshold: float) -> None:
        self._threshold = threshold
        self._last_render = 0.0

    def set_threshold_display(self, value: float) -> None:
        self._threshold = value

    def __call__(self, frame: TactileResidual) -> None:
        now = time.monotonic()
        if now - self._last_render < RENDER_INTERVAL_S:
            return
        self._last_render = now

        data = frame.data
        limit = min(len(data), TACTILE_ROWS * TACTILE_COLS)
        threshold = self._threshold

        above = 0
        peak = float("-inf")
        peak_idx = 0
        for i in range(limit):
            v = data[i]
            if abs(v - MASKED_VALUE) < MASKED_EPS:
                continue
            if threshold > 0.0 and v >= threshold:
                above += 1
            if v > peak:
                peak = v
                peak_idx = i
        if peak == float("-inf"):
            peak = 0.0

        lines = ["\x1b[H\x1b[2J"]
        lines.append(
            f"above={above:<3d} threshold={threshold:.1f} peak={peak:+.3f} "
            f"row={peak_idx // TACTILE_COLS} col={peak_idx % TACTILE_COLS} "
            f"seq={frame.header.seq} len={len(data)}  (+/- threshold, q quit)"
        )
        for row in range(TACTILE_ROWS):
            base = row * TACTILE_COLS
            cells = []
            for col in range(TACTILE_COLS):
                idx = base + col
                v = data[idx] if idx < limit else -1.0
                cells.append(cell_for(v, threshold))
            lines.append("".join(cells))
        sys.stdout.write("\n".join(lines) + "\n")
        sys.stdout.flush()


def connect_device(manager, sn):
    if sn is not None:
        return manager.connect(sn=sn, device_name="example")
    return manager.auto_connect(device_name="example")


def main():
    parser = argparse.ArgumentParser(allow_abbrev=False)
    parser.add_argument("--sn", metavar="DEVICE_SN")
    parser.add_argument("--seconds", type=float, default=0.0)
    parser.add_argument("--threshold", type=float, default=0.5)
    args = parser.parse_args()
    if args.sn == "":
        parser.error("--sn must not be empty")
    if not math.isfinite(args.seconds) or args.seconds < 0:
        parser.error("--seconds must be finite and nonnegative")
    if not math.isfinite(args.threshold):
        parser.error("--threshold must be finite")

    manager = None
    sub = None
    old_settings = None
    result = 0
    callback_errors = []

    def on_frame(frame):
        try:
            view(frame)
        except Exception as error:
            callback_errors.append(error)

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
        value = clamp_threshold(args.threshold)
        view = ResidualView(value)
        operation = "subscribe tactile residual"
        sub = glove.tactile_residual().subscribe_with_callback(on_frame)
        interactive = sys.stdin is not None and sys.stdin.isatty()
        if interactive:
            operation = "read terminal settings"
            old_settings = termios.tcgetattr(sys.stdin.fileno())
            operation = "configure terminal"
            tty.setcbreak(sys.stdin.fileno())
        deadline = time.monotonic() + args.seconds if args.seconds else None
        while deadline is None or time.monotonic() < deadline:
            operation = "receive subscribed frames"
            if callback_errors:
                raise callback_errors[0]
            if not sub.is_active:
                raise RuntimeError("subscription ended")
            if interactive and select.select([sys.stdin], [], [], 0)[0]:
                key = sys.stdin.read(1)
                if key in ("q", "Q"):
                    break
                if key in ("+", "=", "-", "_"):
                    value = clamp_threshold(value + (0.1 if key in ("+", "=") else -0.1))
                    view.set_threshold_display(value)
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print(f"{operation} failed: {error}", file=sys.stderr)
        result = 1
    finally:
        if old_settings is not None:
            try:
                termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_settings)
            except Exception as error:
                print(f"restore terminal failed: {error}", file=sys.stderr)
                result = 1
        if sub is not None:
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


if __name__ == "__main__":
    sys.exit(main())
