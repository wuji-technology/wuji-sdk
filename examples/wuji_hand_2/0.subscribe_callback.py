#!/usr/bin/env python3
"""
Callback subscription example — Wuji Hand 2.

Auto-detect and connect to Wuji Hand 2 devices, enable motors, then subscribe to
the joint_state data stream using subscribe_with_callback().

Callbacks run in background threads, allowing non-blocking data processing.

Usage: python 0.subscribe_callback.py
"""

import time
from functools import partial

from wuji_sdk import SdkManager, ConnectOptions, WujiHand2, JointState


def on_joint_state(device_name: str, state: JointState):
    # Print the first 4 joints (thumb) as a compact sample
    thumb = state.joints[:4]
    positions = [f"{j.actual_pos:+.3f}" for j in thumb]
    print(f"[{device_name}][JointState] cycle={state.cycle_id} thumb_pos={positions}")


def main():
    manager = SdkManager.instance()
    devices = manager.scan()

    if not devices:
        print("No devices found")
        return

    print(f"Found {len(devices)} device(s)")
    for dev in devices:
        print(f"  SN={dev.sn}, Address={dev.address}")

    hands: list[WujiHand2] = []
    subscriptions = []
    for i, dev in enumerate(devices):
        # By default, the SDK allows multiple clients (this script, Wuji Studio,
        # a recorder, etc.) to connect to the same device concurrently.
        # Pass `options=ConnectOptions(enable_bridge=False)` below to opt out
        # for exclusive single-client use.
        # Docs: https://docs.wuji.tech/docs/en/wuji-sdk/latest/
        hand = manager.connect(sn=dev.sn, device_name=f"hand_{i}")
        print(f"Connected: {hand.serial_number} ({hand.online_joints_count().get()} joints online)")
        hand.enable()
        hands.append(hand)
        subscriptions.append(
            hand.joint_state().subscribe_with_callback(partial(on_joint_state, hand.device_name))
        )

    print(f"Subscribed to {len(subscriptions)} streams. Ctrl+C to stop.\n")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        for sub in subscriptions:
            sub.close()
        for hand in hands:
            hand.disable()


if __name__ == "__main__":
    main()
