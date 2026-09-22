#!/usr/bin/env python3
"""Stream a recorded Hand 2 thumb-opposition trajectory at 1 kHz."""

from __future__ import annotations

import math
import signal
import struct
import sys
import time
from pathlib import Path


JOINT_COUNT = 20
MAGIC = b"WJH2RPL\0"
VERSION = 1
HEADER = struct.Struct("<8sHBBI")
FRAME = struct.Struct("<I20f")
SIDE_TO_ID = {"right": 1, "left": 2}
REPLAY_INTERVAL_S = 0.001
RAMP_DURATION_S = 1.0
CAPTURE_TIMEOUT_S = 2.0
_STOP_REQUESTED = False


def _open_replay(side, *, data_dir=None):
    """Open one replay and validate its fixed-size container."""
    directory = Path(data_dir) if data_dir is not None else Path(__file__).parent / "data"
    path = directory / f"{side}.replay"
    try:
        stream = path.open("rb")
    except OSError as error:
        raise RuntimeError(f"replay file not found: {path}") from error
    try:
        header_bytes = stream.read(HEADER.size)
        if len(header_bytes) != HEADER.size:
            raise RuntimeError("replay header length is invalid")
        magic, version, side_id, joint_count, frame_count = HEADER.unpack(
            header_bytes
        )
        if magic != MAGIC:
            raise RuntimeError("replay magic is invalid")
        if version != VERSION:
            raise RuntimeError("replay version is unsupported")
        if side_id != SIDE_TO_ID[side]:
            raise RuntimeError("replay side does not match")
        if joint_count != JOINT_COUNT:
            raise RuntimeError("replay joint count is invalid")
        expected_size = HEADER.size + frame_count * FRAME.size
        if path.stat().st_size != expected_size:
            raise RuntimeError("replay file length does not match frame count")
        return stream, frame_count
    except Exception:
        stream.close()
        raise


def _read_qpos(stream):
    """Read one fixed-size replay frame and return its qpos."""
    frame_bytes = stream.read(FRAME.size)
    if len(frame_bytes) != FRAME.size:
        raise RuntimeError("replay frame read is short")
    values = FRAME.unpack(frame_bytes)
    return values[1:]


def _ramp_frames(start, target, steps):
    """Blend from start to target with a cosine ease-in-out profile."""
    frames = []
    for step in range(1, steps + 1):
        ratio = 0.5 * (1.0 - math.cos(math.pi * step / steps))
        frames.append(
            tuple(current + (goal - current) * ratio
                  for current, goal in zip(start, target))
        )
    return tuple(frames)


def _positions_in_joint_order(entries, nid_to_joint_index=None):
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


def _capture_start_positions(hand, *, timeout=CAPTURE_TIMEOUT_S,
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
                positions = _positions_in_joint_order(
                    frame.joints, nid_to_joint_index
                )
                if positions is not None:
                    return positions
            sleep(0.005)
    finally:
        subscription.close()
    raise RuntimeError("no complete joint_states frame within capture timeout")


def _play_replay(
    stream,
    frame_count,
    send,
    *,
    sleep=time.sleep,
    stop_requested=lambda: False,
    ramp_start=None,
):
    """Stream every qpos once with a fixed 1 ms interval.

    With ramp_start, first ease from that pose to the opening frame.
    """
    if ramp_start is not None and frame_count > 0:
        first_qpos = _read_qpos(stream)
        stream.seek(HEADER.size)
        ramp_steps = round(RAMP_DURATION_S / REPLAY_INTERVAL_S)
        for qpos in _ramp_frames(ramp_start, first_qpos, ramp_steps):
            if stop_requested():
                raise InterruptedError("operator stop requested")
            send(qpos)
            sleep(REPLAY_INTERVAL_S)
    for frame_index in range(frame_count):
        if stop_requested():
            raise InterruptedError("operator stop requested")
        send(_read_qpos(stream))
        if frame_index + 1 < frame_count:
            sleep(REPLAY_INTERVAL_S)


def _side_for_hand(hand):
    """Return the replay side selected by device handedness."""
    handedness = str(hand.handedness().get()).lower()
    if handedness not in SIDE_TO_ID:
        raise RuntimeError(f"unsupported Hand 2 handedness: {handedness}")
    return handedness


def _run_device(*, data_dir=None):
    """Connect, check online joints, enable, stream, and clean up."""
    try:
        import wuji_sdk
    except ModuleNotFoundError as error:
        raise RuntimeError("wuji-sdk package is not installed") from error

    manager = wuji_sdk.SdkManager.instance()
    hand = None
    replay_stream = None
    publisher = None
    try:
        hand = manager.auto_connect("hand_2_opposition")
        online = hand.online_joints_count().get()
        if online != JOINT_COUNT:
            raise RuntimeError(f"expected 20 online joints, found {online}")
        replay_stream, frame_count = _open_replay(
            _side_for_hand(hand), data_dir=data_dir
        )
        hand.enable()
        publisher = hand.joint_command().publish()

        def send(qpos):
            """Publish one flat-20 MIT position command."""
            publisher.send(
                [
                    wuji_sdk.JointCommand(float(position), 0.0, 0.0)
                    for position in qpos
                ]
            )

        _play_replay(
            replay_stream,
            frame_count,
            send,
            ramp_start=_capture_start_positions(hand),
            stop_requested=lambda: _STOP_REQUESTED,
        )
    finally:
        cleanup_errors = []
        primary_error_active = sys.exc_info()[0] is not None
        if hand is not None:
            try:
                hand.disable()
            except Exception as error:
                cleanup_errors.append(f"disable: {error}")
        if publisher is not None:
            try:
                publisher.close()
            except Exception as error:
                cleanup_errors.append(f"close publisher: {error}")
        if replay_stream is not None:
            try:
                replay_stream.close()
            except Exception as error:
                cleanup_errors.append(f"close replay: {error}")
        try:
            manager.disconnect_all()
        except Exception as error:
            cleanup_errors.append(f"disconnect all: {error}")
        if cleanup_errors:
            message = "; ".join(cleanup_errors)
            if primary_error_active:
                print(f"cleanup error: {message}", file=sys.stderr)
            else:
                raise RuntimeError(f"cleanup failed: {message}")
    return 0


def _on_sigint(_signum, _frame):
    """Request an interrupt without calling SDK functions in signal context."""
    global _STOP_REQUESTED
    _STOP_REQUESTED = True


def main():
    """Run the physical replay."""
    global _STOP_REQUESTED
    _STOP_REQUESTED = False
    try:
        return _run_device()
    except (KeyboardInterrupt, InterruptedError):
        return 130
    except Exception as error:
        print(f"error: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    signal.signal(signal.SIGINT, _on_sigint)
    raise SystemExit(main())
