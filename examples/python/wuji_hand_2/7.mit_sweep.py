#!/usr/bin/env python3
"""Send a fixed cosine sweep on Hand 2 joint 0."""

from __future__ import annotations

import math
import signal
import sys
import time


JOINT_COUNT = 20
AMPLITUDE_RAD = 0.02
SWEEP_INTERVALS = 100
COMMAND_INTERVAL_S = 0.02
_STOP_REQUESTED = False


def generate_sweep():
    """Generate the fixed 101-command joint-0 sweep."""
    frames = []
    for frame_index in range(SWEEP_INTERVALS + 1):
        phase = 2.0 * math.pi * frame_index / SWEEP_INTERVALS
        position = [0.0] * JOINT_COUNT
        position[0] = AMPLITUDE_RAD * 0.5 * (1.0 - math.cos(phase))
        frames.append(tuple(position))
    return tuple(frames)


def send_sweep(frames, publisher, sdk, *, sleep=time.sleep):
    """Send the fixed sequence with a 20 ms command interval."""
    for frame_index, position in enumerate(frames):
        if _STOP_REQUESTED:
            raise InterruptedError("operator stop requested")
        publisher.send(
            [sdk.JointCommand(value, 0.0, 0.0) for value in position]
        )
        if frame_index + 1 < len(frames):
            sleep(COMMAND_INTERVAL_S)


def close_resource(resource, label, errors):
    """Close one SDK resource and collect cleanup failures."""
    if resource is None:
        return
    try:
        resource.close()
    except Exception as error:
        errors.append(f"close {label}: {error}")


def run_device():
    """Connect, check 20 online joints, enable, and send the sweep."""
    try:
        import wuji_sdk
    except ModuleNotFoundError as error:
        raise RuntimeError("wuji-sdk package is not installed") from error

    manager = wuji_sdk.SdkManager.instance()
    hand = None
    publisher = None
    cleanup_errors = []
    primary_error = None
    exit_code = 0

    try:
        hand = manager.auto_connect(device_name="hand_2_mit_sweep")
        online = int(hand.online_joints_count().get())
        if online != JOINT_COUNT:
            raise RuntimeError(f"expected 20/20 online joints, got {online}/20")
        try:
            hand.enable()
        except Exception as error:
            raise RuntimeError(f"enable failed: {error}") from error
        publisher = hand.joint_command().publish()
        send_sweep(generate_sweep(), publisher, wuji_sdk)
    except (KeyboardInterrupt, InterruptedError):
        exit_code = 130
        primary_error = "operator stop requested"
    except Exception as error:
        exit_code = 1
        primary_error = str(error)
    finally:
        if hand is not None:
            try:
                hand.disable()
            except Exception as error:
                cleanup_errors.append(f"disable: {error}")
        close_resource(publisher, "publisher", cleanup_errors)
        try:
            manager.disconnect_all()
        except Exception as error:
            cleanup_errors.append(f"disconnect all: {error}")

    if exit_code == 0 and cleanup_errors:
        exit_code = 1
    if primary_error is not None:
        print(f"error: {primary_error}", file=sys.stderr)
    for error in cleanup_errors:
        print(f"cleanup error: {error}", file=sys.stderr)
    return exit_code


def on_sigint(_signum, _frame):
    """Request cooperative cleanup after Ctrl+C."""
    global _STOP_REQUESTED
    _STOP_REQUESTED = True


def main():
    """Run the fixed physical-device transaction."""
    global _STOP_REQUESTED
    _STOP_REQUESTED = False
    return run_device()


if __name__ == "__main__":
    signal.signal(signal.SIGINT, on_sigint)
    raise SystemExit(main())
