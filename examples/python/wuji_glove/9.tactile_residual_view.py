#!/usr/bin/env python3
"""
Wuji Glove standalone live tactile residual view.

Renders the `tactile_residual` stream as a 24x31 terminal grid, the Python
counterpart of the C example `7_tactile_residual_view.c`. This is an
observation tool, not an alternative tactile-calibration workflow.

`tactile_residual` is the continuous signal behind `tactile_binary`: a signed
per-taxel residual (positive = harder than the calibrated baseline, ~0 = none,
negative = lighter). Unlike `tactile_binary`, contact detection is left to the
consumer -- you apply your own threshold. Adjust it live with + / - and watch
the grid respond; the printed peak value guides where to set it.

Legend: # = at/above your threshold, . : - = + * = increasing residual below
it, space = invalid/masked taxel.

The contact model is keyed by glove serial number and is loaded automatically
at runtime once calibrated. If the grid stays near zero, run
`7.tactile_calibration.py` first.

Usage:
  python 9.tactile_residual_view.py --sn <serial_number>
  python 9.tactile_residual_view.py --sn <serial_number> --seconds 60 --threshold 0.8
"""

import argparse
import sys
import threading
import time

from wuji_sdk import DeviceType, SdkManager, TactileResidual, set_log_level

TACTILE_ROWS = 24
TACTILE_COLS = 31
MASKED_VALUE = -1.0  # invalid/masked sentinel
MASKED_EPS = 1e-6
THRESHOLD_STEP = 0.1
RENDER_INTERVAL_S = 0.1
RAMP = ".:-=+*"  # increasing residual magnitude below the threshold


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sn", default=None, help="Optional Wuji Glove serial number.")
    parser.add_argument("--device-name", default="my_glove")
    parser.add_argument(
        "--seconds",
        type=float,
        default=0.0,
        help="Run for this many seconds (0 = until q / Ctrl-C).",
    )
    parser.add_argument(
        "--threshold",
        type=float,
        default=0.5,
        help="Your contact threshold on the residual; taxels at/above it render '#'.",
    )
    parser.add_argument(
        "--log-level",
        default="warn",
        choices=["trace", "debug", "info", "warn", "warning", "error", "off"],
    )
    return parser.parse_args()


def connect_glove(manager: SdkManager, args: argparse.Namespace):
    if args.sn:
        print(f"Connecting to Wuji Glove SN={args.sn}...")
        return manager.connect(sn=args.sn, device_name=args.device_name)

    print("Scanning for Wuji Glove...")
    gloves = [d for d in manager.scan() if d.device_type == DeviceType.WujiGlove]
    if not gloves:
        raise SystemExit("No Wuji Glove found among the scanned devices")
    return manager.connect(sn=gloves[0].sn, device_name=args.device_name)


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
        self._lock = threading.Lock()

    def set_threshold_display(self, value: float) -> None:
        with self._lock:
            self._threshold = value

    def __call__(self, frame: TactileResidual) -> None:
        now = time.monotonic()
        if now - self._last_render < RENDER_INTERVAL_S:
            return
        self._last_render = now

        data = frame.data
        limit = min(len(data), TACTILE_ROWS * TACTILE_COLS)
        with self._lock:
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


def _read_key():
    """Non-blocking single-char read; None if nothing ready (POSIX cbreak)."""
    import select

    ready, _, _ = select.select([sys.stdin], [], [], 0)
    if not ready:
        return None
    return sys.stdin.read(1)


def run_interactive(view: ResidualView, threshold: float, deadline: float | None) -> None:
    import termios
    import tty

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while True:
            if deadline is not None and time.monotonic() >= deadline:
                break
            key = _read_key()
            if key in ("q", "Q"):
                break
            if key in ("+", "="):
                threshold = clamp_threshold(threshold + THRESHOLD_STEP)
                view.set_threshold_display(threshold)
            elif key in ("-", "_"):
                threshold = clamp_threshold(threshold - THRESHOLD_STEP)
                view.set_threshold_display(threshold)
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def run_noninteractive(deadline: float | None) -> None:
    print("(stdin is not a TTY: threshold is fixed to --threshold; Ctrl-C to stop)")
    try:
        while deadline is None or time.monotonic() < deadline:
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass


def main() -> int:
    args = parse_args()
    set_log_level(args.log_level)
    manager = SdkManager.instance()
    glove = connect_glove(manager, args)
    print(
        "Subscribing to tactile_residual. "
        "Calibrate the glove first (7.tactile_calibration.py) if the grid stays near zero."
    )

    threshold = clamp_threshold(args.threshold)
    view = ResidualView(threshold)
    sub = glove.tactile_residual().subscribe_with_callback(callback=view)

    deadline = time.monotonic() + args.seconds if args.seconds > 0 else None
    interactive = False
    try:
        interactive = sys.stdin is not None and sys.stdin.isatty()
    except Exception:  # noqa: BLE001 - be conservative if stdin is odd
        interactive = False

    sys.stdout.write("\x1b[H\x1b[2Jwaiting for tactile_residual frames... (+/- threshold, q quit)\n")
    sys.stdout.flush()

    try:
        if interactive:
            run_interactive(view, threshold, deadline)
        else:
            run_noninteractive(deadline)
    finally:
        sub.close()
        print("\nStopping.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
