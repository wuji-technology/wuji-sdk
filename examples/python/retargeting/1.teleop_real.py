#!/usr/bin/env python3
"""
Retargeting example - live teleoperation from a Wuji Glove.

The SDK exposes only the pure retarget interface (``RetargetSession``). Teleop —
read keypoints → retarget → drive the hand — is plain example code you can adapt,
shown end to end below: read the glove's hand keypoints, retarget each frame, and
drive a connected Wuji Hand or Wuji Hand 2.

To drive from a different keypoint source (camera / MediaPipe / VR), replace the
glove read with your own: ``RetargetSession.step`` accepts any ``(21, 3)`` float32
array of MediaPipe-format landmarks and returns a 20-value joint command already
in firmware order (the per-model joint-order permutation is applied inside the
SDK), so the result is sent to the hand as-is.

The session is built with ``RetargetSession.for_hand(model, side=SIDE)`` — the hand
model selects the builtin tuning config internally, so there is no config path to
manage. Configuring and enabling the hand's motors is the caller's responsibility,
shown here in configure_wuji_hand_2 / configure_wuji_hand.

This example is set up for a **right**-hand rig (glove + hand). For a left-hand
setup, change ``SIDE`` below — it picks the retargeter's handedness basis.

This example deliberately runs the **glove** on the SDK built-in default hand
URDF rather than a per-user IK calibration: per-user calibration still has some
rough edges, and the built-in model currently gives more reliable teleop. It
switches to the default SDK user before connecting — the default user runs the
glove on the built-in default hand URDF — and restores the previously selected
user on exit. See ``use_builtin_urdf_user``. If the built-in URDF doesn't track
your hand well, create a non-default SDK user, switch to it, then calibrate the
glove under it — calibrating under the default user has no effect, since the
default user always runs on the built-in URDF.

Install the retargeting runtime dependencies first:

    pip install "wuji-sdk[retarget]"

Usage:
    python 1.teleop_real.py
"""

import contextlib
import time

import numpy as np

from wuji_sdk import (
    DeviceType,
    Handedness,
    JointCommand,
    LowPass,
    SdkManager,
    WujiHand2,
    retargeting,
)

FPS = 120  # target loop rate (ceiling); a slow frame lowers the rate, never bursts.
# Handedness of the rig (glove + hand); use Handedness.Left for a left-hand setup.
SIDE = Handedness.Right


def configure_wuji_hand_2(hand):
    """Configure + enable a Wuji Hand 2; return the retarget HandModel."""
    # MIT impedance: hold each joint softly at its commanded position. The
    # firmware defaults to MIT control mode (control mode is not set from Python).
    hand.effort_limit().set(1.5)  # Amps, broadcast to all joints
    hand.mit_params().set((3.0, 0.05))  # (kp, kd) tuple, broadcast to all joints
    hand.enable()
    return retargeting.HandModel.WujiHand2


def configure_wuji_hand(hand):
    """Configure + enable a first-gen Wuji Hand; return the retarget HandModel."""
    hand.set_all_effort_limit(1.5)  # Amps
    hand.enable()
    return retargeting.HandModel.WujiHand


def use_builtin_urdf_user(manager):
    """Switch to the default SDK user so the glove uses the built-in URDF.

    The default SDK user runs the glove on the built-in default hand URDF rather
    than a per-user IK calibration. Returns the previously selected user so
    main() can restore it on exit.
    """
    previous = manager.current_user()
    manager.switch_to_default_user()
    return previous


def read_keypoints(skeleton_sub):
    """Read the glove's latest hand_skeleton frame as a (21, 3) float32 array.

    The glove publishes hand_skeleton faster than this loop consumes it, and
    recv() returns the OLDEST unread frame — taking one per loop falls behind.
    Drain every queued frame, keep only the latest. Returns None if no frame is
    available this tick.
    """
    latest = None
    while True:
        frame = skeleton_sub.recv()
        if frame is None:
            break
        latest = frame
    if latest is None:
        return None
    return np.array([j.pose.position for j in latest.joints], dtype=np.float32)


def teleop(glove, model, send):
    """Build the retargeting session + glove source, then loop read → retarget → send.

    Called *after* the hand's command channel is already open (see main). Building
    the session (RetargetSession.for_hand) is a heavy, multi-second call; opening
    the command channel is a blocking device round-trip. Doing the round-trip first
    (on idle CPU, right after enable) keeps the just-enabled hand responsive, so the
    session build here does not race a heartbeat/round-trip timeout.
    """
    session = retargeting.RetargetSession.for_hand(model, side=SIDE)
    skeleton_sub = glove.hand_skeleton().subscribe()

    budget = 1.0 / FPS
    last_kp = None
    while True:
        frame_start = time.monotonic()
        kp = read_keypoints(skeleton_sub)
        if kp is None:
            kp = last_kp  # no fresh frame: hold the last pose
            if kp is None:
                time.sleep(budget)  # nothing yet — wait out the budget
                continue
        else:
            last_kp = kp
        qpos = session.step(kp)  # (20,) firmware order — send as-is
        send(qpos.tolist())
        dt = time.monotonic() - frame_start
        if dt < budget:
            time.sleep(budget - dt)


def run_teleop(manager) -> int:
    """Connect a glove + hand and teleoperate until interrupted.

    """

    hand_dev = None
    glove_dev = None
    for d in manager.scan():
        print(f"  SN={d.sn}, Type={d.device_type}, Address={d.address}")
        if d.device_type in (DeviceType.WujiHand2, DeviceType.WujiHand):
            hand_dev = d
        elif d.device_type == DeviceType.WujiGlove:
            glove_dev = d

    if hand_dev is None or glove_dev is None:
        print("No Wuji Hand / Wuji Hand 2 found" if hand_dev is None else "No Wuji Glove found")
        return 1

    hand = manager.connect(sn=hand_dev.sn, device_name=hand_dev.sn)
    glove = manager.connect(sn=glove_dev.sn, device_name=glove_dev.sn)

    is_hand2 = isinstance(hand, WujiHand2)
    model = configure_wuji_hand_2(hand) if is_hand2 else configure_wuji_hand(hand)

    print("Teleoperating (Ctrl+C to stop)...")
    try:
        # Open the hand's command channel FIRST — right after enabling the motors,
        # before teleop() builds the heavy retargeting session. Opening the channel
        # is a blocking device round-trip; doing it up front (while the just-enabled
        # hand is responsive) avoids a round-trip/heartbeat timeout during the
        # multi-second session build.
        if is_hand2:
            # Wuji Hand 2: send MIT position commands via the joint_command publisher.
            # send() takes one JointCommand per joint; position only (velocity/effort 0).
            publisher = hand.joint_command().publish()
            teleop(
                glove,
                model,
                lambda q: publisher.send([JointCommand(p, 0.0, 0.0) for p in q]),
            )
        else:
            # First-gen Wuji Hand: drive through a low-pass realtime controller.
            with hand.realtime_controller(LowPass(cutoff_hz=5.0)) as controller:
                teleop(glove, model, controller.set_target_position)
    except KeyboardInterrupt:
        pass
    finally:
        with contextlib.suppress(Exception):
            hand.disable()
        manager.disconnect_all()

    return 0


def main() -> int:
    manager = SdkManager.instance()

    # Run the glove on the SDK built-in default hand URDF — per-user IK
    # calibration still has rough edges, so the built-in model gives more
    # reliable teleop. The default SDK user uses the built-in URDF, so switch to
    # it before connecting and restore the previous user on exit.
    previous_user = use_builtin_urdf_user(manager)
    exit_code = 0
    try:
        exit_code = run_teleop(manager)
    finally:
        try:
            manager.switch_user(previous_user["user_id"])
        except Exception as exc:
            print(f"Failed to restore previous SDK user: {exc}")
            exit_code = 1

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
