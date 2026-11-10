#!/usr/bin/env python3
"""Drive Wuji Hand 2 in position mode with a scaled effort limit.

Switches the whole hand to position mode, scales each joint's effort limit,
sets the position-command low-pass cutoff, and opens and closes the hand with
position targets. The effort limit and the cutoff are restored on exit; the
hand stays in position mode.

The effort limit follows the control mode: joint output torque in N·m in
position mode, current in A in MIT mode. Each mode keeps its own limit.

Requires firmware 3.0.0 or later. Earlier firmware rejects the switch to
position mode and the example exits.
"""

from __future__ import annotations

import math
import signal
import sys
import time


JOINT_COUNT = 20
JOINTS_PER_FINGER = 4
AMPLITUDE_RAD = 0.35
SWEEP_PERIOD_S = 4.0
SWEEP_CYCLES = 2
COMMAND_INTERVAL_S = 1.0 / 120.0
COMMAND_CUTOFF_HZ = 40.0
# Fraction of the position-mode effort limit the hand reports at startup.
EFFORT_LIMIT_SCALE = 0.8
RAMP_DURATION_S = 1.0
RAMP_INTERVAL_S = 0.001
ENABLE_TIMEOUT_S = 5.0
CAPTURE_TIMEOUT_S = 2.0
# joint_diagnostics ext_state: 0=Init 1=Ready 2=Enabled 3=Stopped
EXT_STATE_ENABLED = 2
_STOP_REQUESTED = False


def generate_sweep():
    """Generate the fixed open/close sweep; all 20 joints share one cosine."""
    intervals = round(SWEEP_PERIOD_S / COMMAND_INTERVAL_S)
    frames = []
    for frame_index in range(intervals * SWEEP_CYCLES + 1):
        phase = 2.0 * math.pi * frame_index / intervals
        position = AMPLITUDE_RAD * 0.5 * (1.0 - math.cos(phase))
        frames.append((position,) * JOINT_COUNT)
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


def format_limits(values):
    """Format per-joint limits as one group of four per finger."""
    fingers = [
        values[first:first + JOINTS_PER_FINGER]
        for first in range(0, len(values), JOINTS_PER_FINGER)
    ]
    return " | ".join(
        " ".join(f"{value:.3f}" for value in finger) for finger in fingers
    )


def read_effort_limits(resource):
    """Read all 20 effort limits; reject an offline or invalid joint."""
    limits = resource.get()
    if len(limits) != JOINT_COUNT or any(
        value is None or not math.isfinite(value) or value < 0.0
        for value in limits
    ):
        raise RuntimeError(
            "effort_limit must report a valid value for all 20 joints"
        )
    return [float(value) for value in limits]


def write_effort_limits(resource, limits):
    """Write all 20 effort limits and return the values the hand reports.

    set() returns once the hand accepts the request; a joint that misses the
    write keeps its previous limit. The limits are read back and every joint
    must report the value written.
    """
    resource.set(limits)
    reported = read_effort_limits(resource)
    missed = [
        joint
        for joint, (written, read) in enumerate(zip(limits, reported))
        if not math.isclose(written, read, rel_tol=1e-5, abs_tol=1e-6)
    ]
    if missed:
        raise RuntimeError(
            f"effort_limit write did not reach joints {missed}; "
            f"they report {[round(reported[joint], 3) for joint in missed]}"
        )
    return reported


def read_command_cutoff(resource):
    """Read the whole-hand position-command low-pass cutoff in Hz."""
    cutoff = resource.get()
    if cutoff is None or not math.isfinite(cutoff) or cutoff < 0.0:
        raise RuntimeError(
            "position_command_low_pass_cutoff_frequency must be finite and >= 0"
        )
    return float(cutoff)


def wait_enabled(hand, timeout=ENABLE_TIMEOUT_S):
    """Block until the newest diagnostics frame reports all 20 joints Enabled.

    joint_diagnostics carries online joints only and recv() returns the oldest
    queued frame, so each round drains to the newest frame and a short frame
    never passes. A joint that engages partway through the startup ramp would
    take the rest of it as a step.
    """
    subscription = hand.joint_diagnostics().subscribe()
    seen = None
    try:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if _STOP_REQUESTED:
                raise InterruptedError("operator stop requested")
            frame = None
            while True:  # keep only the newest queued frame
                newer = subscription.recv()
                if newer is None:
                    break
                frame = newer
            if frame is not None:
                seen = len(frame.joints)
                if seen == JOINT_COUNT and all(
                    entry.status_word.ext_state == EXT_STATE_ENABLED
                    for entry in frame.joints
                ):
                    return
            time.sleep(0.05)
    finally:
        subscription.close()
    if seen is None:
        raise RuntimeError("enable timeout: no joint_diagnostics frame arrived")
    raise RuntimeError(
        f"enable timeout: the newest diagnostics frame reported {seen} of "
        f"{JOINT_COUNT} joints, not all of them Enabled"
    )


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
        # send() takes one JointCommand per joint; position mode reads only
        # position, so velocity and effort stay 0.
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


def connect_hand(manager, device_name, sdk):
    """Scan and connect one Wuji Hand 2 selected from the discovered devices.

    Only Wuji Hand 2 devices are eligible. When several are present, list
    their serial numbers and prompt for a choice. Enter, or a closed stdin,
    picks the first.
    """
    devices = manager.scan()
    if not devices:
        raise RuntimeError(
            "no device found; check that the device is powered on and "
            "reachable on the network"
        )
    print(f"Found {len(devices)} device(s)")
    for dev in devices:
        print(f"  SN={dev.sn}, Type={dev.device_type}, Address={dev.address}")
    hands = [
        dev for dev in devices if dev.device_type == sdk.DeviceType.WujiHand2
    ]
    if not hands:
        raise RuntimeError("no Wuji Hand 2 found among the scanned devices")
    index = 0
    if len(hands) > 1:
        print("Wuji Hand 2 devices:")
        for i, dev in enumerate(hands):
            print(f"  [{i}] SN={dev.sn}, Address={dev.address}")
        def _interrupt_on_sigint(_signum, _frame):
            """Turn Ctrl+C at the prompt into KeyboardInterrupt."""
            raise KeyboardInterrupt

        previous_sigint_handler = signal.getsignal(signal.SIGINT)
        signal.signal(signal.SIGINT, _interrupt_on_sigint)
        try:
            while True:
                try:
                    choice = input(
                        f"Select Wuji Hand 2 to connect [0-{len(hands) - 1}], "
                        "Enter picks 0: "
                    ).strip()
                except EOFError:
                    print(
                        "No terminal input available - connecting the first "
                        "Wuji Hand 2."
                    )
                    break
                if choice == "":
                    break
                try:
                    selected = int(choice)
                except ValueError:
                    selected = -1
                if not 0 <= selected < len(hands):
                    print(f"Invalid selection: {choice!r}")
                    continue
                index = selected
                break
        finally:
            signal.signal(signal.SIGINT, previous_sigint_handler)
    chosen = hands[index]
    print(f"Connecting Wuji Hand 2: SN={chosen.sn}, Address={chosen.address}")
    return manager.connect(sn=chosen.sn, device_name=device_name)


def run_device():
    """Connect, configure position mode, send the sweep, and restore."""
    try:
        import wuji_sdk
    except ModuleNotFoundError as error:
        raise RuntimeError("wuji-sdk package is not installed") from error

    manager = wuji_sdk.SdkManager.instance()
    hand = None
    publisher = None
    restores = []
    cleanup_errors = []
    primary_error = None
    exit_code = 0

    try:
        hand = connect_hand(manager, "hand_2_position_mode", wuji_sdk)
        online = int(hand.online_joints_count().get())
        if online != JOINT_COUNT:
            raise RuntimeError(f"expected 20/20 online joints, got {online}/20")
        # Position mode: each joint's firmware position loop tracks the
        # commanded angle. The firmware changes mode only while the joints are
        # disabled, so disable the hand, switch, then enable.
        hand.disable()
        hand.control_mode().set(wuji_sdk.ControlMode.Position)
        print(f"control mode: {hand.control_mode().get()}")

        # Read the limits after the switch so the snapshot, the scaled write
        # and the restore on exit are all position-mode values in N·m. They
        # differ per joint, so each joint is scaled from its own value.
        effort_limit = hand.effort_limit()
        cutoff = hand.position_command_low_pass_cutoff_frequency()
        original_limits = read_effort_limits(effort_limit)
        original_cutoff = read_command_cutoff(cutoff)
        scaled_limits = [value * EFFORT_LIMIT_SCALE for value in original_limits]
        # Register each restore before its write: a failed reply can follow a
        # write that already reached the hand.
        restores.append(
            (
                "effort limit",
                lambda: write_effort_limits(effort_limit, original_limits),
            )
        )
        written_limits = write_effort_limits(effort_limit, scaled_limits)
        print(f"effort limit read (N*m):    {format_limits(original_limits)}")
        print(f"effort limit scale:         {EFFORT_LIMIT_SCALE}")
        print(f"effort limit written (N*m): {format_limits(written_limits)}")
        # The firmware low-pass filters incoming position targets at its
        # control rate; a lower cutoff smooths more and delays more.
        restores.append(
            ("command cutoff", lambda: cutoff.set(original_cutoff))
        )
        cutoff.set(COMMAND_CUTOFF_HZ)
        print(f"command cutoff: {read_command_cutoff(cutoff):.1f} Hz")

        hand.enable()
        wait_enabled(hand)
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
        for label, restore in reversed(restores):
            try:
                restore()
            except Exception as error:
                cleanup_errors.append(f"restore {label}: {error}")
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
