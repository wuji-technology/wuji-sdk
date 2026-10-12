#!/usr/bin/env python3
"""
Retargeting example - live teleoperation from a Wuji Glove.

The SDK exposes only the pure retarget interface (``RetargetSession``). Teleop —
read keypoints → retarget → drive the hand — is plain example code you can adapt,
shown end to end below: read the glove's hand keypoints, retarget each frame, and
drive a connected Wuji Hand or Wuji Hand 2.

To drive from a different keypoint source (camera / MediaPipe / VR), replace the
glove read with your own: ``RetargetSession.step`` accepts any ``(21, 3)`` float32
array of MediaPipe-format landmarks and returns a 20-value joint command in
firmware order, so the result is sent to the hand as-is.

The session is built with ``RetargetSession.for_hand(model, side=SIDE)`` — the hand
model selects the builtin tuning config internally, so there is no config path to
manage. Configuring and enabling the hand's motors is the caller's responsibility,
shown here in configure_wuji_hand_2 / configure_wuji_hand.

On a Wuji Hand 2 the loop starts with a ``RAMP_SEC`` ease from the hand's current
pose to the glove pose. Commanding the first retargeted frame directly would put
the whole startup difference into one frame, which the motors feel as a step
input. Keep this ramp when adapting the example.

This example is set up for a **right**-hand rig (glove + hand). For a left-hand
setup, change ``SIDE`` below — it picks the retargeter's handedness basis.

Before connecting, the example lists the SDK users and lets you pick which one
to run under (see ``select_user``); pressing Enter keeps the current user, and
the previously selected user is restored on exit. The default user runs the
**glove** on the SDK built-in default hand URDF; a named user runs it with that
user's calibrated hand model, or on the built-in URDF as well when that user has
no calibration yet. To teleoperate with a calibrated hand model, create a user
and calibrate the glove under it first (``wuji_glove/4.user.py`` and
``wuji_glove/5.calibration.py``) — calibrating under the default user has no
effect, since the default user always runs on the built-in URDF.

Install the retargeting runtime dependencies first (numpy for keypoint/qpos arrays):

    pip install wuji-sdk numpy

Usage:
    python 1.teleop_real.py
    python 1.teleop_real.py --hand-sn <hand SN> --glove-sn <glove SN>

Without --hand-sn / --glove-sn the example drives whichever hand and glove the
scan reports last, which is an arbitrary one when several are reachable. Pass
the serial numbers whenever more than one hand or glove shares the network.
"""

import argparse
import contextlib
import time

import numpy as np

from wuji_sdk import (
    DeviceType,
    HandModel,
    Handedness,
    JointCommand,
    LowPass,
    RetargetSession,
    SdkManager,
    WujiHand2,
)

FPS = 120  # target loop rate (ceiling); a slow frame lowers the rate, never bursts.
# Handedness of the rig (glove + hand); use Handedness.Left for a left-hand setup.
SIDE = Handedness.Right
# Seconds spent easing from the hand's current pose to the glove pose on startup.
RAMP_SEC = 1.0
# Both streams carry online joints only, and joint_command().send rejects any
# length but the full one, so the startup reads below refuse short frames.
TOTAL_JOINTS = 20


def configure_wuji_hand_2(hand):
    """Configure + enable a Wuji Hand 2; return the retarget HandModel."""
    # MIT impedance: hold each joint softly at its commanded position. The
    # firmware defaults to MIT control mode (control mode is not set from Python).
    hand.effort_limit().set(1.5)  # Amps, broadcast to all joints
    hand.mit_params().set((5.0, 0.1))  # (kp, kd) tuple, broadcast to all joints
    hand.enable()
    return HandModel.WujiHand2


def configure_wuji_hand(hand):
    """Configure + enable a first-gen Wuji Hand; return the retarget HandModel."""
    hand.set_all_effort_limit(1.5)  # Amps
    hand.enable()
    return HandModel.WujiHand


def select_user(manager):
    """Interactively pick the SDK user to run under, and switch to it.

    Switching happens before any device is connected, so the glove reads the
    selected user's hand model. Returns the previously selected user so main()
    can restore it on exit. Pressing Enter — or a closed stdin, as in a piped
    run — keeps the current user.
    """
    previous = manager.current_user()
    users = manager.list_users()
    print("SDK users:")
    for i, user in enumerate(users):
        tags = []
        if user["is_default"]:
            tags.append("built-in URDF")
        if user["user_id"] == previous["user_id"]:
            tags.append("current")
        suffix = f" ({', '.join(tags)})" if tags else ""
        print(f"  [{i}] {user['display_name']}{suffix}")

    while True:
        try:
            choice = input(f"Select user [0-{len(users) - 1}], Enter keeps current: ").strip()
        except EOFError:
            print("No terminal input available — keeping the current SDK user.")
            choice = ""
        if not choice:
            selected = previous
            break
        try:
            index = int(choice)
        except ValueError:
            index = -1
        # Reject negatives explicitly: users[-1] would silently pick the last user.
        if not 0 <= index < len(users):
            print(f"Invalid selection: {choice!r}")
            continue
        selected = users[index]
        manager.switch_user(selected["user_id"])
        break

    print(f"Running as SDK user: {selected['display_name']}")
    return previous


def drain_latest(sub):
    """Return the newest queued frame, discarding the older ones, or None.

    Every stream here publishes far faster than these loops poll it, and recv()
    returns the OLDEST unread frame. Taking one frame per tick therefore walks
    through a backlog instead of reading the present — a startup gate written
    that way opens on a state the hand left up to a full buffer ago.
    """
    latest = None
    while True:
        frame = sub.recv()
        if frame is None:
            return latest
        latest = frame


def read_keypoints(skeleton_sub):
    """Read the glove's latest hand_skeleton frame as a (21, 3) float32 array.

    Returns None if no frame is available this tick. See drain_latest for why
    every read here takes the newest queued frame rather than the first one.
    """
    latest = drain_latest(skeleton_sub)
    if latest is None:
        return None
    return np.array([j.pose.position for j in latest.joints], dtype=np.float32)


def wait_enabled(hand, timeout=5.0):
    """Block until every joint reports Enabled.

    ext_state: 0=Init 1=Ready 2=Enabled 3=Stopped. Ramping into motors that are
    not holding position yet would land the whole ramp at once when they engage.
    The judgement is made on the newest frame (see drain_latest).
    """
    deadline = time.monotonic() + timeout
    diag_sub = hand.joint_diagnostics().subscribe()
    had_frame = False
    seen = 0
    try:
        while time.monotonic() < deadline:
            time.sleep(0.05)
            frame = drain_latest(diag_sub)
            if frame is None:
                continue
            # had_frame separates "nothing arrived" from "an empty frame" for the
            # message below — seen leaves both at 0.
            had_frame = True
            seen = len(frame.joints)
            # All-Enabled on a short frame would start the ramp while the missing
            # joints are still coming up, and they would take the step it avoids.
            if seen != TOTAL_JOINTS:
                continue
            if all(e.status_word.ext_state == 2 for e in frame.joints):
                return
    finally:
        diag_sub.close()
    if not had_frame:
        raise RuntimeError(
            "Enable timeout — no joint_diagnostics frame arrived; "
            "is the hand publishing?"
        )
    raise RuntimeError(
        f"Enable timeout — the newest diagnostics frame reported {seen} of "
        f"{TOTAL_JOINTS} joints with not all of them Enabled"
    )


def read_current_pose(hand, timeout=3.0):
    """Return the hand's current joint positions, one per joint.

    A short frame is an error rather than something to ramp from: zip() against
    it would silently produce a command joint_command().send rejects.
    """
    sub = hand.joint_states().subscribe()
    try:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            frame = drain_latest(sub)
            if frame is not None:
                if len(frame.joints) != TOTAL_JOINTS:
                    raise RuntimeError(
                        f"joint_states reports {len(frame.joints)} online joints, "
                        f"expected {TOTAL_JOINTS} — every joint must be online to ramp"
                    )
                return [j.position for j in frame.joints]
            time.sleep(0.002)
        raise RuntimeError("Timed out waiting for joint_states — is the hand publishing?")
    finally:
        sub.close()


def ramp_to_glove(start, session, skeleton_sub, send, budget, timeout=5.0):
    """Ease from the hand's current pose to the glove pose, then return the last keypoints.

    The first retargeted frame is otherwise a step input, and a Wuji Hand 2 has
    no smoothing channel of its own, so the example ramps in software.
    The endpoint is re-retargeted every frame: the operator keeps moving, so a
    frozen target would end the ramp with a step.
    """
    deadline = time.monotonic() + timeout
    kp = None
    while kp is None and time.monotonic() < deadline:
        kp = read_keypoints(skeleton_sub)
        if kp is None:
            time.sleep(budget)
    if kp is None:
        raise RuntimeError("Timed out waiting for the glove's first hand_skeleton frame")

    steps = max(int(RAMP_SEC * FPS), 1)
    start = list(start)
    for step in range(1, steps + 1):
        frame_start = time.monotonic()
        fresh = read_keypoints(skeleton_sub)
        if fresh is not None:
            kp = fresh
        goal = session.step(kp)
        alpha = step / steps
        send([a + alpha * (b - a) for a, b in zip(start, goal)])
        dt = time.monotonic() - frame_start
        if dt < budget:
            time.sleep(budget - dt)
    return kp


def teleop(glove, model, send, start=None):
    """Build the retargeting session + glove source, then loop read → retarget → send.

    Called *after* the hand's command channel is already open (see run_teleop):
    open the command channel before session initialization so device
    communication remains active during setup.

    ``start`` is the hand's current pose; when given, the first RAMP_SEC seconds
    ease from it to the glove pose instead of stepping there in one frame.
    """
    session = RetargetSession.for_hand(model, side=SIDE)
    skeleton_sub = glove.hand_skeleton().subscribe()

    budget = 1.0 / FPS
    last_kp = None
    if start is not None:
        last_kp = ramp_to_glove(start, session, skeleton_sub, send, budget)
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


def run_teleop(manager, hand_sn=None, glove_sn=None) -> int:
    """Connect a glove + hand and teleoperate until interrupted.

    ``hand_sn`` / ``glove_sn`` restrict the search to that serial number. Leave
    them unset to keep the default, which takes the last device of each kind the
    scan reports — an arbitrary one when several are reachable.
    """

    hand_dev = None
    glove_dev = None
    for d in manager.scan():
        print(f"  SN={d.sn}, Type={d.device_type}, Address={d.address}")
        if d.device_type in (DeviceType.WujiHand2, DeviceType.WujiHand):
            if hand_sn is None or d.sn == hand_sn:
                hand_dev = d
        elif d.device_type == DeviceType.WujiGlove:
            if glove_sn is None or d.sn == glove_sn:
                glove_dev = d

    if hand_dev is None or glove_dev is None:
        if hand_dev is None:
            print("No Wuji Hand / Wuji Hand 2 found"
                  + (f" with SN {hand_sn}" if hand_sn else ""))
        else:
            print("No Wuji Glove found" + (f" with SN {glove_sn}" if glove_sn else ""))
        return 1

    hand = manager.connect(sn=hand_dev.sn, device_name=hand_dev.sn)
    glove = manager.connect(sn=glove_dev.sn, device_name=glove_dev.sn)

    is_hand2 = isinstance(hand, WujiHand2)
    model = configure_wuji_hand_2(hand) if is_hand2 else configure_wuji_hand(hand)

    print("Teleoperating (Ctrl+C to stop)...")
    try:
        # Open the hand's command channel first — before teleop() builds the
        # retargeting session — so device communication remains active during
        # setup.
        if is_hand2:
            # Wuji Hand 2: send MIT position commands via the joint_command publisher.
            # send() takes one JointCommand per joint; position only (velocity/effort 0).
            publisher = hand.joint_command().publish()
            # Read where the hand actually is before commanding it anywhere, so
            # teleop can ease into the glove pose (see ramp_to_glove).
            wait_enabled(hand)
            start = read_current_pose(hand)
            teleop(
                glove,
                model,
                lambda q: publisher.send([JointCommand(p, 0.0, 0.0) for p in q]),
                start=start,
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Teleoperate a Wuji Hand from a Wuji Glove.")
    parser.add_argument("--hand-sn", default=None,
                        help="Serial number of the hand to drive. "
                             "Default: the last hand the scan reports.")
    parser.add_argument("--glove-sn", default=None,
                        help="Serial number of the glove to read. "
                             "Default: the last glove the scan reports.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    manager = SdkManager.instance()

    # Pick the SDK user to run under. This has to happen before connecting: the
    # glove reads the selected user's hand model at connect time.
    #
    # Integrating this example and don't need user selection? Drop this call and
    # the restore in the finally below — the SDK then keeps running as whatever
    # user is currently selected.
    try:
        previous_user = select_user(manager)
    except KeyboardInterrupt:
        print()
        return 130

    exit_code = 0
    try:
        exit_code = run_teleop(manager, args.hand_sn, args.glove_sn)
    finally:
        try:
            manager.switch_user(previous_user["user_id"])
        except Exception as exc:
            print(f"Failed to restore previous SDK user: {exc}")
            exit_code = 1

    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
