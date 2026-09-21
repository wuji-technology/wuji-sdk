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
RAMP_DURATION_S = 1.0
RAMP_INTERVAL_S = 0.001
CAPTURE_TIMEOUT_S = 2.0
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


def ramp_frames(start, target, steps):
    """Blend from start to target with a cosine ease-in-out profile."""
    frames = []
    for step in range(1, steps + 1):
        ratio = 0.5 * (1.0 - math.cos(math.pi * step / steps))
        frames.append(
            tuple(current + (goal - current) * ratio
                  for current, goal in zip(start, target))
        )
    return tuple(frames)


def positions_in_joint_order(entries, nid_to_joint_index=None):
    """Scatter one frame into flat-20 order via WujiHand2.nid_to_joint_index.

    Returns None unless the frame carries exactly the 20 joint nids — a
    tactile-slot or out-of-range nid rejects the frame instead of being
    silently misplaced.

    `nid_to_joint_index` is a test hook: the default None resolves to the
    real `WujiHand2.nid_to_joint_index`, and any injected callable must raise
    WujiException for a non-joint nid (which rejects the frame).
    """
    import wuji_sdk

    if nid_to_joint_index is None:
        nid_to_joint_index = wuji_sdk.WujiHand2.nid_to_joint_index
    positions = [None] * JOINT_COUNT
    for entry in entries:
        try:
            index = nid_to_joint_index(entry.nid)
        except wuji_sdk.WujiException:
            return None
        if positions[index] is not None:
            return None
        positions[index] = float(entry.position)
    if any(position is None for position in positions):
        return None
    return tuple(positions)


def capture_start_positions(hand, *, timeout=CAPTURE_TIMEOUT_S,
                            sleep=time.sleep, monotonic=time.monotonic,
                            nid_to_joint_index=None):
    """Drain joint_states to the newest frame; return flat-20 positions."""
    subscription = hand.joint_states().subscribe()
    try:
        deadline = monotonic() + timeout
        while monotonic() < deadline:
            if _STOP_REQUESTED:
                raise InterruptedError("operator stop requested")
            frame = None
            while True:  # keep only the newest queued frame
                newer = subscription.recv()
                if newer is None:
                    break
                frame = newer
            if frame is not None and len(frame.joints) == JOINT_COUNT:
                positions = positions_in_joint_order(
                    frame.joints, nid_to_joint_index
                )
                if positions is not None:
                    return positions
            sleep(0.005)
    finally:
        subscription.close()
    raise RuntimeError("no complete joint_states frame within capture timeout")


def send_sweep(frames, publisher, sdk, *, sleep=time.sleep,
               interval=COMMAND_INTERVAL_S):
    """Send the fixed sequence with the given command interval."""
    for frame_index, position in enumerate(frames):
        if _STOP_REQUESTED:
            raise InterruptedError("operator stop requested")
        publisher.send(
            [sdk.JointCommand(value, 0.0, 0.0) for value in position]
        )
        if frame_index + 1 < len(frames):
            sleep(interval)


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
        hand.enable()
        publisher = hand.joint_command().publish()
        sweep = generate_sweep()
        start = capture_start_positions(hand)
        ramp_steps = round(RAMP_DURATION_S / RAMP_INTERVAL_S)
        send_sweep(ramp_frames(start, sweep[0], ramp_steps), publisher,
                   wuji_sdk, interval=RAMP_INTERVAL_S)
        send_sweep(sweep, publisher, wuji_sdk)
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
