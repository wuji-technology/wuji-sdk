#!/usr/bin/env python3
"""
EMF poses rate divider — purpose and affected streams.

WHY THIS EXISTS
    EMF poses are solved at the EMF sensor rate (~120 Hz). Every downstream
    hand-tracking stream is recomputed from each emf_poses frame, and the
    inverse-kinematics (IK) solve behind hand_joint_angles is the most
    expensive step. Most applications do not need 120 Hz hand tracking.

    `glove.emf_poses_rate_divider().set(N)` publishes emf_poses at
    input_rate / N (e.g. N=4 -> ~30 Hz). Because the downstream streams are
    data-driven, they all follow automatically, so the IK/FK solver runs N
    times less often — directly cutting CPU. The setting applies at runtime
    (within a few frames) and persists across reconnects. Default N=1 means
    no change.

STREAMS AFFECTED BY THE DIVIDER (all drop to ~input_rate / N):
    - emf_poses           the divider's direct target
    - hand_joint_angles   IK solve — this is where the CPU is saved
    - tip_poses           derived from hand_joint_angles
    - hand_skeleton       derived from hand_joint_angles
    - tactile_point_cloud derived from hand_skeleton

STREAMS NOT AFFECTED (rate unchanged):
    - imu_palm / imu_* (and tf): IMU-driven, so the wrist pose stays smooth
    - tactile / tactile_zones: device tactile stream
    - tactile_residual / tactile_binary: driven by the tactile stream, not
      emf_poses (their rate is unchanged; the hand pose they align against
      simply refreshes N times less often)

This example subscribes to the affected streams plus imu_palm (as an
unaffected reference) and prints each stream's measured rate at divider=1
and divider=4, so you can see the whole emf_poses chain drop together while
the IMU stream stays constant.

Usage: python 6.emf_poses_rate_divider.py
"""

import asyncio
from wuji_sdk import SdkManager, DeviceType, WujiGlove

# (label, subscribe callable, affected-by-divider?)
STREAMS = [
    ("emf_poses", lambda g: g.emf_poses().subscribe(), True),
    ("hand_joint_angles", lambda g: g.hand_joint_angles().subscribe(), True),
    ("tip_poses", lambda g: g.tip_poses().subscribe(), True),
    ("hand_skeleton", lambda g: g.hand_skeleton().subscribe(), True),
    ("tactile_point_cloud", lambda g: g.tactile_point_cloud().subscribe(), True),
    ("imu_palm (reference)", lambda g: g.imu_palm().subscribe(), False),
]

WINDOW_S = 3.0
SETTLE_S = 0.6  # let a divider change take effect (handler re-reads every ~10 frames)


async def measure_rates(glove, divider: int) -> dict:
    """Set the divider, let it settle, then measure each stream's rate (Hz).

    Subscriptions are created fresh for each call: a fresh subscription starts
    from "now" with no backlog, so counting frames over a fixed window gives the
    live rate directly — no draining needed.
    """
    glove.emf_poses_rate_divider().set(divider)
    await asyncio.sleep(SETTLE_S)

    subs = {label: subscribe(glove) for label, subscribe, _ in STREAMS}
    counts = {label: 0 for label in subs}

    async def count(label: str):
        sub = subs[label]
        while True:
            await sub.recv_async()
            counts[label] += 1

    tasks = [asyncio.create_task(count(label)) for label in subs]
    await asyncio.sleep(WINDOW_S)
    for t in tasks:
        t.cancel()
    return {label: counts[label] / WINDOW_S for label in counts}


async def main():
    manager = SdkManager.instance()
    devices = manager.scan()
    if not devices:
        print("No devices found")
        return
    for dev in devices:
        print(f"  SN={dev.sn}, Type={dev.device_type}, Address={dev.address}")

    gloves = [d for d in devices if d.device_type == DeviceType.WujiGlove]
    if not gloves:
        print("No Wuji Glove found among the scanned devices")
        return
    glove: WujiGlove = manager.connect(sn=gloves[0].sn, device_name="glove")
    print(f"Connected: {glove.serial_number}\n")

    hz_baseline = await measure_rates(glove, 1)
    hz_divided = await measure_rates(glove, 4)
    glove.emf_poses_rate_divider().set(1)  # restore default

    print(f"{'stream':<24}{'N=1 (Hz)':>10}{'N=4 (Hz)':>10}   effect")
    print("-" * 60)
    for label, _, affected in STREAMS:
        tag = "follows divider" if affected else "unaffected"
        print(f"{label:<24}{hz_baseline[label]:>10.0f}{hz_divided[label]:>10.0f}   {tag}")
    print("\nThe emf_poses chain drops ~4x (less IK/FK CPU); the IMU stream is unchanged.")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
