#!/usr/bin/env python3
"""
Offline pipeline example.

Run WujiGlove hand tracking and IMU fusion without connecting to a physical device.
This script hardcodes a few synthetic EMF pose frames and feeds them into
WujiGlove.offline_pipeline().

Usage: python 3.offline_pipeline.py
"""

from wuji_sdk import (
    EmfPose,
    EmfPoseArray,
    FrameHeader,
    ImuData,
    Pose,
    Quaternion,
    Vector3F64,
    WujiGlove,
)


def identity_orientation() -> Quaternion:
    return Quaternion(0.0, 0.0, 0.0, 1.0)


def make_emf_frame(seq: int, timestamp_us: int, curl: float) -> EmfPoseArray:
    """Create one synthetic 5-finger EMF pose frame in r_hand_emf_tx."""
    finger_positions = [
        [0.028 + curl * 0.002, -0.030, -0.010 - curl * 0.010],  # thumb
        [0.038, -0.015, -0.050 - curl * 0.018],  # index
        [0.044, 0.000, -0.058 - curl * 0.020],  # middle
        [0.040, 0.015, -0.052 - curl * 0.017],  # ring
        [0.032, 0.028, -0.043 - curl * 0.014],  # pinky
    ]

    return EmfPoseArray(
        header=FrameHeader(
            seq=seq,
            timestamp_us=timestamp_us,
            frame_id="r_hand_emf_tx",
        ),
        poses=[
            EmfPose(
                pose=Pose(position=position, orientation=identity_orientation()),
                confidence=0.95,
            )
            for position in finger_positions
        ],
    )


def make_raw_imu(seq: int, timestamp_us: int) -> ImuData:
    return ImuData.raw(
        header=FrameHeader(seq=seq, timestamp_us=timestamp_us, frame_id="imu_raw/palm"),
        angular_velocity=Vector3F64(0.0, 0.0, 0.0),
        linear_acceleration=Vector3F64(0.0, 0.0, 9.8),
    )


def main() -> None:
    # Use the SN of the glove that captured the data. It should match the
    # calibrated device so the SDK can load that device's URDF.
    #
    # Optional: pass urdf_path to force a specific calibrated hand model.
    # Example: urdf_path="~/.wuji/sdk/models/<capturing-glove-sn>_hand.urdf"
    # Hand IK outputs depend on the URDF; IMU fusion outputs do not.
    pipeline = WujiGlove.offline_pipeline(
        sn="WujiGlove-DEMO-OFFLINE",
        hand_side="right",
    )
    print(f"pipeline: sn={pipeline.sn}, hand_side={pipeline.hand_side}")
    print(f"urdf: source={pipeline.urdf_source}, path={pipeline.urdf_source_path}")

    frames = [
        make_emf_frame(seq=1, timestamp_us=0, curl=0.00),
        make_emf_frame(seq=2, timestamp_us=8_333, curl=0.35),
        make_emf_frame(seq=3, timestamp_us=16_666, curl=0.70),
    ]

    for frame in frames:
        angles = pipeline.hand_joint_angles().compute(frame)
        tips = pipeline.tip_poses().compute(frame)
        skeleton = pipeline.hand_skeleton().compute(frame)

        first_finger = angles.fingers[0]
        print(
            f"frame={frame.header.seq} "
            f"fingers={len(angles.fingers)} "
            f"tips={len(tips.poses)} "
            f"joints={len(skeleton.joints)} "
            f"thumb_conf={first_finger.confidence:.3f} "
            f"thumb_angles={[round(a, 3) for a in first_finger.angles]}"
        )

    fused = pipeline.imu_data_palm().compute(make_raw_imu(seq=4, timestamp_us=25_000))
    print(
        "imu_palm "
        f"seq={fused.header.seq} "
        f"orientation=({fused.orientation.x:.3f}, {fused.orientation.y:.3f}, "
        f"{fused.orientation.z:.3f}, {fused.orientation.w:.3f}) "
        f"cov0={fused.orientation_covariance[0]:.1f}"
    )


if __name__ == "__main__":
    main()
