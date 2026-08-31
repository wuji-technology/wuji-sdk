#!/usr/bin/env python3
"""
Stream rate control example.

Subscription.set_rate(hz) lowers the device push rate of a subscribed stream
and returns the rate the device actually applied (it may be quantized to a
device-supported rate). Pass 0 to restore the device default.

Rate-adjustable streams on the Wuji Hand 2: joint_states, joint_diagnostics,
imu, fingertip/{thumb,index,middle,ring,pinky}/data.
joint_states and joint_diagnostics share the same device stream, so changing
the rate through either subscription changes both.

The rate belongs to the stream, not to your handle: it applies to every
subscriber of that stream, and resets to the device default when the stream is
released or the device reconnects.

Usage:
  python 4.set_stream_rate.py
"""

import asyncio
from wuji_sdk import DeviceType, SdkManager

REQUESTED_HZ = 100


async def main():
    manager = SdkManager.instance()
    hands = [d for d in manager.scan() if d.device_type == DeviceType.WujiHand2]
    if not hands:
        print("No Wuji Hand 2 found")
        return
    sub = None
    try:
        hand = manager.connect(sn=hands[0].sn, device_name="hand")
        print(f"Connected: {hand.serial_number}")

        # Keep the handle open for as long as you want its rate to stay in effect.
        sub = hand.joint_states().subscribe()
        # Calling set_rate on joint_states or joint_diagnostics lowers both.
        # other = hand.imu().subscribe(); print(other.set_rate(REQUESTED_HZ))

        actual = sub.set_rate(REQUESTED_HZ)
        print(f"joint_states: requested {REQUESTED_HZ} Hz, device applied {actual} Hz")
        for _ in range(5):
            frame = await sub.recv_async()
            print(f"joint_states frame at {actual} Hz: {len(frame.joints)} joints")

        print(f"restored to the device default: {sub.set_rate(0)} Hz")
    finally:
        if sub is not None:
            sub.close()
        manager.disconnect_all()


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        pass
