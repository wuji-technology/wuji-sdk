#!/usr/bin/env python3
"""
Retargeting example - RetargetSession (no hardware required).

Map human hand keypoints (21 MediaPipe-format landmarks, in meters) to a Wuji
Hand joint command (20 values). This is the low-level building block: you supply
the keypoints from any source (camera, glove replay, a VR headset, ...), and get
back a joint command ready to send to the hand.

Install the retargeting runtime dependencies first (numpy for keypoint/qpos arrays):

    pip install wuji-sdk numpy

Usage: python 0.retarget_session.py
"""

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


def main() -> int:
    # Build a session for a right Wuji Hand 2. The hand model selects the builtin
    # tuning config internally, so you don't deal with config paths. The session
    # keeps warm-start and smoothing state across frames, so reuse it in your loop.
    # (Models: HandModel.WujiHand or HandModel.WujiHand2.)
    session = RetargetSession.for_hand(
        HandModel.WujiHand2, side=Handedness.Right
    )

    keypoints = make_open_hand_keypoints()
    qpos = session.step(keypoints)  # (20,) float32, firmware joint order

    print(f"qpos shape : {qpos.shape}, dtype {qpos.dtype}")
    print("joint command (radians, finger-major thumb/index/middle/ring/pinky):")
    for f, name in enumerate(["thumb", "index", "middle", "ring", "pinky"]):
        joints = qpos[f * 4 : (f + 1) * 4]
        print(f"  {name:6s}: {[f'{v:+.3f}' for v in joints]}")

    # session.step(...) every frame; session.reset() after a tracking gap.
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
