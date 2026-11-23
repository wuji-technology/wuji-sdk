#!/usr/bin/env python3
"""
Purpose:
    Map 21 synthetic hand keypoints to 20 joint positions.
Prerequisites:
    Install wuji-sdk and numpy. No device is required.
Usage:
    python 0_retarget_session.py
Expected Behavior:
    Print the right-hand joint command in radians, grouped by finger.
    Exit 0 on completion or Ctrl-C, 1 on execution or cleanup failure,
    and 2 on invalid arguments. Errors go to stderr.
"""

import argparse
import sys

import numpy as np

from wuji_sdk import HandModel, Handedness, RetargetSession


def make_open_hand_keypoints() -> np.ndarray:
    """A synthetic open-right-hand pose, (21, 3) in meters, MediaPipe order.

    Replace this with your real keypoint source (camera / glove / replay).
    """
    kp = np.zeros((21, 3), dtype=np.float32)
    # finger base landmark index -> x offset (thumb, index, middle, ring, pinky)
    for base, x in [(1, -0.04), (5, -0.03), (9, -0.01), (13, 0.01), (17, 0.03)]:
        for k in range(4):
            kp[base + k] = [x, 0.03 * (k + 1), 0.0]
    kp[1] = [-0.03, 0.02, 0.01]  # thumb CMC nearer the palm
    return kp


def main():
    argparse.ArgumentParser(allow_abbrev=False).parse_args()
    operation = "create retargeting session"
    try:
        session = RetargetSession.for_hand(
            HandModel.WujiHand2, side=Handedness.Right
        )

        keypoints = make_open_hand_keypoints()
        operation = "retarget keypoints"
        qpos = session.step(keypoints)  # (20,) float32, firmware joint order

        print(f"qpos shape : {qpos.shape}, dtype {qpos.dtype}")
        print("joint command (radians, finger-major thumb/index/middle/ring/pinky):")
        for f, name in enumerate(["thumb", "index", "middle", "ring", "pinky"]):
            joints = qpos[f * 4 : (f + 1) * 4]
            print(f"  {name:6s}: {[f'{v:+.3f}' for v in joints]}")

    except KeyboardInterrupt:
        pass
    except Exception as error:
        print(f"{operation} failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
