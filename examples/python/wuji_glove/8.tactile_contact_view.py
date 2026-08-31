#!/usr/bin/env python3
"""
Wuji Glove standalone live tactile contact view.

Renders the `tactile_binary` stream as a 24x31 terminal grid, the Python
counterpart of the C example `6_tactile_contact_view.c`. This is an
observation tool, not an alternative tactile-calibration workflow.

Legend: # = contact, . = hand surface, space = invalid/masked taxel.
On an interactive terminal, press + / - to tune sensitivity and q to quit;
otherwise it runs until the duration elapses or Ctrl-C.

The contact model is keyed by glove serial number and is loaded automatically
at runtime once calibrated. If the grid stays near zero, run
`7.tactile_calibration.py` first.

Usage:
  python 8.tactile_contact_view.py --sn <serial_number>
  python 8.tactile_contact_view.py --sn <serial_number> --seconds 60 --sensitivity 1.5
"""

import argparse
import sys
import threading
import time

from wuji_sdk import DeviceType, SdkManager, TactileBinary, WujiException, set_log_level

TACTILE_ROWS = 24
TACTILE_COLS = 31
SENSITIVITY_MIN = 0.5
SENSITIVITY_MAX = 3.0
SENSITIVITY_PARAM = "algorithms.tactile_binary.sensitivity"
RENDER_INTERVAL_S = 0.1


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
        "--sensitivity",
        type=float,
        default=1.0,
        help=f"Contact sensitivity in [{SENSITIVITY_MIN}, {SENSITIVITY_MAX}]; >1 is more sensitive.",
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


def clamp_sensitivity(value: float) -> float:
    return max(SENSITIVITY_MIN, min(SENSITIVITY_MAX, float(value)))


def set_sensitivity(glove, value: float) -> float:
    """Clamp + push sensitivity to the device; never raise (keep the view alive)."""
    value = clamp_sensitivity(value)
    try:
        glove.set(SENSITIVITY_PARAM, value)
    except WujiException as exc:
        print(f"\n(failed to set sensitivity {value:.1f}: {exc})", file=sys.stderr)
    return value


class ContactView:
    """Renders tactile_binary frames as a grid; throttled to ~10 Hz."""

    def __init__(self, sensitivity: float) -> None:
        self._sensitivity = sensitivity
        self._last_render = 0.0
        self._lock = threading.Lock()

    def set_sensitivity_display(self, value: float) -> None:
        with self._lock:
            self._sensitivity = value

    def __call__(self, frame: TactileBinary) -> None:
        now = time.monotonic()
        if now - self._last_render < RENDER_INTERVAL_S:
            return
        self._last_render = now

        data = frame.data
        limit = min(len(data), TACTILE_ROWS * TACTILE_COLS)
        contacts = 0
        peak = -1.0
        peak_idx = 0
        for i in range(limit):
            v = data[i]
            if v > 0.5:
                contacts += 1
            if v > peak:
                peak = v
                peak_idx = i

        with self._lock:
            sensitivity = self._sensitivity

        lines = ["\x1b[H\x1b[2J"]
        lines.append(
            f"contacts={contacts:<3d} sensitivity={sensitivity:.1f} peak={peak:.3f} "
            f"row={peak_idx // TACTILE_COLS} col={peak_idx % TACTILE_COLS} "
            f"len={len(data)}  (+/- sensitivity, q quit)"
        )
        for row in range(TACTILE_ROWS):
            base = row * TACTILE_COLS
            cells = []
            for col in range(TACTILE_COLS):
                idx = base + col
                v = data[idx] if idx < limit else -1.0
                cells.append("#" if v > 0.5 else (" " if v < -0.5 else "."))
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


def run_interactive(glove, view: ContactView, sensitivity: float, deadline: float | None) -> None:
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
                sensitivity = set_sensitivity(glove, sensitivity + 0.1)
                view.set_sensitivity_display(sensitivity)
            elif key in ("-", "_"):
                sensitivity = set_sensitivity(glove, sensitivity - 0.1)
                view.set_sensitivity_display(sensitivity)
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def run_noninteractive(deadline: float | None) -> None:
    print("(stdin is not a TTY: sensitivity is fixed to --sensitivity; Ctrl-C to stop)")
    try:
        while deadline is None or time.monotonic() < deadline:
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass


def main() -> int:
    args = parse_args()
    set_log_level(args.log_level)
    manager = SdkManager.instance()
    try:
        glove = connect_glove(manager, args)
        print(
            "Subscribing to tactile_binary. "
            "Calibrate the glove first (7.tactile_calibration.py) if the grid stays near zero."
        )

        sensitivity = set_sensitivity(glove, args.sensitivity)
        view = ContactView(sensitivity)
        sub = glove.tactile_binary().subscribe_with_callback(callback=view)

        deadline = time.monotonic() + args.seconds if args.seconds > 0 else None
        interactive = False
        try:
            interactive = sys.stdin is not None and sys.stdin.isatty()
        except Exception:  # noqa: BLE001 - be conservative if stdin is odd
            interactive = False

        sys.stdout.write("\x1b[H\x1b[2Jwaiting for tactile_binary frames... (+/- sensitivity, q quit)\n")
        sys.stdout.flush()

        try:
            if interactive:
                run_interactive(glove, view, sensitivity, deadline)
            else:
                run_noninteractive(deadline)
        finally:
            sub.close()
            print("\nStopping.")
        return 0
    finally:
        manager.disconnect_all()


if __name__ == "__main__":
    raise SystemExit(main())
