#!/usr/bin/env python3
"""
Purpose:
    Drive a Wuji Hand or Wuji Hand 2 from Wuji Glove keypoints.
Prerequisites:
    Install wuji-sdk and numpy. Use a hand and glove with matching handedness.
    Wuji Hand 2 needs firmware 3.0.0 or later.
    Keep the hand clear of people and objects.
Usage:
    python 1_teleop_real.py
    python 1_teleop_real.py --hand-sn HAND_SN --glove-sn GLOVE_SN
Expected Behavior:
    Select an SDK user before connecting. Enter keeps the current user.
    Without serial numbers, select the last hand and glove found by scanning.
    Use a 1.5 A effort limit and 5 Hz LowPass controller for Wuji Hand, or
    position mode with a
    40 Hz command cutoff for Wuji Hand 2. Retarget at up to 120 Hz.
    Wuji Hand 2 first ramps from its current pose over 1 second.
    Stop with an error if no new glove skeleton frame arrives for 5 seconds.
    On exit, close streams, disable, disconnect, and restore the SDK user.
    Exit 0 on completion or Ctrl-C, 1 on execution or cleanup failure,
    and 2 on invalid arguments. Errors go to stderr.
"""

import argparse
import contextlib
import math
import signal
import sys
import time

import numpy as np
from wuji_sdk import (ControlMode, DeviceType, HandModel, Handedness,
                      JointCommand, LowPass, RetargetSession, SdkManager,
                      WujiGlove, WujiHand, WujiHand2)

FPS = 120
RAMP_SEC = 1.0
TOTAL_JOINTS = 20
HAND_CUTOFF_HZ = 5.0
HAND_2_CUTOFF_HZ = 40.0


def select_user(manager, previous):
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
        if not 0 <= index < len(users):
            print(f"Invalid selection: {choice!r}")
            continue
        selected = users[index]
        manager.switch_user(selected["user_id"])
        break

    print(f"Running as SDK user: {selected['display_name']}")


def drain_latest(sub):
    latest = None
    while True:
        frame = sub.recv()
        if frame is None:
            return latest
        latest = frame


def read_keypoints(skeleton_sub):
    latest = drain_latest(skeleton_sub)
    if latest is None:
        return None
    keypoints = np.array([j.pose.position for j in latest.joints], dtype=np.float32)
    if keypoints.shape != (21, 3) or not np.isfinite(keypoints).all():
        raise RuntimeError("invalid hand_skeleton frame")
    return keypoints


def wait_enabled(hand):
    sub = hand.joint_diagnostics().subscribe()
    try:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            frame = None
            while (latest := sub.recv()) is not None:
                frame = latest
            if frame is not None and len(frame.joints) == TOTAL_JOINTS:
                if all(joint.status_word.ext_state == 2 for joint in frame.joints):
                    return
            time.sleep(0.01)
        raise TimeoutError("waiting for all joints to be enabled")
    finally:
        sub.close()


def read_positions(hand):
    sub = hand.joint_states().subscribe()
    try:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            frame = sub.recv()
            if frame is not None:
                positions = [None] * TOTAL_JOINTS
                for joint in frame.joints:
                    index = WujiHand2.nid_to_joint_index(joint.nid)
                    if positions[index] is not None:
                        raise RuntimeError("duplicate joint in joint_states")
                    if joint.position is None or not math.isfinite(joint.position):
                        raise RuntimeError("invalid joint position")
                    positions[index] = joint.position
                if all(position is not None for position in positions):
                    return positions
            time.sleep(0.01)
        raise TimeoutError("waiting for positions from all 20 joints")
    finally:
        sub.close()


def ramp_to_glove(start, session, skeleton_sub, send, budget, timeout=5.0):
    deadline = time.monotonic() + timeout
    kp = None
    while kp is None and time.monotonic() < deadline:
        kp = read_keypoints(skeleton_sub)
        if kp is None:
            time.sleep(budget)
    if kp is None:
        raise RuntimeError("Timed out waiting for the glove's first hand_skeleton frame")

    last_received = time.monotonic()
    steps = max(int(RAMP_SEC * FPS), 1)
    start = list(start)
    for step in range(1, steps + 1):
        frame_start = time.monotonic()
        fresh = read_keypoints(skeleton_sub)
        if fresh is not None:
            kp = fresh
            last_received = time.monotonic()
        if time.monotonic() - last_received >= timeout:
            raise TimeoutError("no new hand_skeleton frame for 5 seconds")
        goal = session.step(kp)
        alpha = step / steps
        send([a + alpha * (b - a) for a, b in zip(start, goal)])
        dt = time.monotonic() - frame_start
        if dt < budget:
            time.sleep(budget - dt)
    return kp, last_received


def teleop(session, skeleton_sub, send, start=None):
    budget = 1.0 / FPS
    last_kp = None
    last_received = time.monotonic()
    if start is not None:
        last_kp, last_received = ramp_to_glove(start, session, skeleton_sub, send, budget)
    while True:
        frame_start = time.monotonic()
        kp = read_keypoints(skeleton_sub)
        if kp is None:
            if time.monotonic() - last_received >= 5.0:
                raise TimeoutError("no new hand_skeleton frame for 5 seconds")
            kp = last_kp
            if kp is None:
                time.sleep(budget)
                continue
        else:
            last_kp = kp
            last_received = time.monotonic()
        qpos = session.step(kp)
        send(qpos.tolist())
        dt = time.monotonic() - frame_start
        if dt < budget:
            time.sleep(budget - dt)


def main():
    parser = argparse.ArgumentParser(allow_abbrev=False)
    parser.add_argument("--hand-sn")
    parser.add_argument("--glove-sn")
    args = parser.parse_args()
    if args.hand_sn == "" or args.glove_sn == "":
        parser.error("serial numbers must not be empty")

    manager = None
    hand = None
    publisher = None
    skeleton_sub = None
    previous_user = None
    controllers = contextlib.ExitStack()
    result = 0
    operation = "initialize SDK"
    try:
        manager = SdkManager.instance()
        operation = "read current SDK user"
        previous_user = manager.current_user()
        operation = "select SDK user"
        select_user(manager, previous_user)
        hand_info = None
        glove_info = None
        operation = "scan devices"
        for info in manager.scan():
            if info.device_type in (DeviceType.WujiHand, DeviceType.WujiHand2):
                if args.hand_sn is None or info.sn == args.hand_sn:
                    hand_info = info
            elif info.device_type == DeviceType.WujiGlove:
                if args.glove_sn is None or info.sn == args.glove_sn:
                    glove_info = info
        if hand_info is None or glove_info is None:
            raise RuntimeError("no matching hand and glove found")
        operation = "connect"
        device = manager.connect(sn=hand_info.sn, device_name="hand")
        operation = "check device type"
        if not isinstance(device, (WujiHand, WujiHand2)):
            raise RuntimeError(f"expected Wuji Hand or Wuji Hand 2, got {type(device).__name__}")
        hand = device
        operation = "connect"
        glove = manager.connect(sn=glove_info.sn, device_name="glove")
        operation = "check device type"
        if not isinstance(glove, WujiGlove):
            raise RuntimeError(f"expected Wuji Glove, got {type(glove).__name__}")
        operation = "read serial number"
        print(f"Connected hand: {hand.serial_number}, glove: {glove.serial_number}")
        is_hand2 = isinstance(hand, WujiHand2)
        operation = "read handedness"
        side = hand.handedness().get() if is_hand2 else hand.handedness_name().lower()
        operation = "read hand side"
        if side not in ("left", "right") or glove.hand_side().get() != side:
            raise RuntimeError("hand and glove must have matching handedness")
        handedness = Handedness.Left if side == "left" else Handedness.Right
        # Disable joints before switching the Wuji Hand 2 control mode.
        operation = "disable joints"
        hand.disable()
        if is_hand2:
            operation = "set control mode"
            hand.control_mode().set(ControlMode.Position)
            operation = "set position command low pass cutoff frequency"
            hand.position_command_low_pass_cutoff_frequency().set(HAND_2_CUTOFF_HZ)
            operation = "enable joints"
            hand.enable()
            operation = "wait for joints to be enabled"
            wait_enabled(hand)
            operation = "read joint positions"
            start = read_positions(hand)
            operation = "create publisher for joint command"
            publisher = hand.joint_command().publish()
            send = lambda q: publisher.send([JointCommand(p, 0.0, 0.0) for p in q])
            model = HandModel.WujiHand2
        else:
            operation = "set effort limits"
            hand.set_all_effort_limit(1.5)
            operation = "create realtime controller"
            controller = controllers.enter_context(hand.realtime_controller(LowPass(cutoff_hz=HAND_CUTOFF_HZ)))
            operation = "enable joints"
            hand.enable()
            start = None
            send = controller.set_target_position
            model = HandModel.WujiHand
        operation = "create retargeting session"
        session = RetargetSession.for_hand(model, side=handedness)
        operation = "subscribe hand skeleton"
        skeleton_sub = glove.hand_skeleton().subscribe()
        operation = "retarget and send joint commands"
        teleop(session, skeleton_sub, send, start)
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print(f"{operation} failed: {error}", file=sys.stderr)
        result = 1
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        for resource in (skeleton_sub, publisher):
            if resource is not None:
                try:
                    resource.close()
                except Exception as error:
                    print(f"close resource failed: {error}", file=sys.stderr)
                    result = 1
        try:
            controllers.close()
        except Exception as error:
            print(f"close controller failed: {error}", file=sys.stderr)
            result = 1
        if hand is not None:
            try:
                hand.disable()
            except Exception as error:
                print(f"disable failed: {error}", file=sys.stderr)
                result = 1
        if manager is not None:
            try:
                manager.disconnect_all()
            except Exception as error:
                print(f"disconnect failed: {error}", file=sys.stderr)
                result = 1
            if previous_user is not None:
                try:
                    manager.switch_user(previous_user["user_id"])
                except Exception as error:
                    print(f"restore user failed: {error}", file=sys.stderr)
                    result = 1
    return result


if __name__ == "__main__":
    sys.exit(main())
