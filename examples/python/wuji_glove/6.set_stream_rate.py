#!/usr/bin/env python3
"""
Stream rate control example.

Subscription.set_rate(hz) lowers the device push rate of a subscribed stream
and returns the rate the device actually applied (it may be quantized to a
device-supported rate). Pass 0 to restore the device default.

Rate-adjustable streams on the Wuji Glove: emf_poses, tactile, tactile_zones,
imu_raw/{palm,thumb,index,middle,ring,pinky}. Streams derived from them
(hand_joint_angles, hand_skeleton, tip_poses, imu_data/*) follow their source
rate and raise WujiException on set_rate.

The rate belongs to the stream, not to your handle: it applies to every
subscriber of that stream, and resets to the device default when the stream is
released or the device reconnects.

Usage:
  python 6.set_stream_rate.py
"""

import asyncio
from wuji_sdk import DeviceType, SdkManager

REQUESTED_HZ = 30


async def main():
    manager = SdkManager.instance()
    gloves = [d for d in manager.scan() if d.device_type == DeviceType.WujiGlove]
    if not gloves:
        print("No Wuji Glove found")
        return
    glove = manager.connect(sn=gloves[0].sn, device_name="glove")
    print(f"Connected: {glove.serial_number}")

    # Keep the handle open for as long as you want its rate to stay in effect.
    sub = glove.emf_poses().subscribe()
    # other = glove.tactile().subscribe(); print(other.set_rate(REQUESTED_HZ))

    actual = sub.set_rate(REQUESTED_HZ)
    print(f"emf_poses: requested {REQUESTED_HZ} Hz, device applied {actual} Hz")
    for _ in range(5):
        frame = await sub.recv_async()
        print(f"emf_poses frame at {actual} Hz: {len(frame.poses)} poses")

    print(f"restored to the device default: {sub.set_rate(0)} Hz")
    sub.close()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
