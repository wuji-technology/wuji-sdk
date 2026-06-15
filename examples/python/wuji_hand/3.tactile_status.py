#!/usr/bin/env python3
"""
Tactile example - Wuji Hand.

Auto-detect and connect to a Wuji Hand, check tactile glove status, and print a
small pressure summary when frames are available.

Usage: python 3.tactile_status.py
"""

import math
import time

from wuji_sdk import SdkManager, TactileGloveFrame, TactileStatus, TransportType, WujiHand

REPORT_SECONDS = 0.5
POLL_SLEEP_SECONDS = 0.02


def pressure_summary(frame: TactileGloveFrame) -> str:
    pressure = [v for v in frame.pressure if math.isfinite(v)]
    if not pressure:
        return f"seq={frame.sequence} pressure=[]"

    sample = pressure[:8]
    mean_pressure = sum(pressure) / len(pressure)
    return (
        f"seq={frame.sequence} "
        f"max={max(pressure):.3f} "
        f"mean={mean_pressure:.3f} "
        f"first8={[f'{v:.3f}' for v in sample]}"
    )


def main():
    manager = SdkManager.instance()
    devices = [dev for dev in manager.scan() if dev.transport_type == TransportType.Usb]

    if not devices:
        print("No Wuji Hand devices found")
        return 1

    hand: WujiHand = manager.connect(sn=devices[0].sn, device_name="wuji_hand")
    status_sub = hand.tactile_status().subscribe()
    initial_status: TactileStatus | None = None
    attached = False
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        initial_status = status_sub.recv()
        if initial_status is not None:
            attached = initial_status.state == 1
            break
        time.sleep(POLL_SLEEP_SECONDS)

    print(
        f"Connected: {hand.serial_number} "
        f"({hand.handedness_name()}, tactile_attached={attached})"
    )

    frame_sub = None
    if attached:
        frame_sub = hand.tactile.subscribe_pressure_frame()
        print("Subscribed to tactile_status and tactile pressure frames. Ctrl+C to stop.\n")
    else:
        print("Subscribed to tactile_status. No tactile glove is attached.\n")

    latest: TactileStatus | None = initial_status
    latest_frame: TactileGloveFrame | None = None
    next_report = time.monotonic()

    try:
        while True:
            status: TactileStatus | None = status_sub.recv()
            if status is not None:
                latest = status
                # Late attach: subscribe to pressure frames as soon as the glove
                # reports attached, in case it was plugged in after start.
                if frame_sub is None and status.state == 1:
                    frame_sub = hand.tactile.subscribe_pressure_frame()
                    print("Tactile glove attached — subscribed to pressure frames.")

            frame = None
            if frame_sub is not None:
                frame = frame_sub.recv()
                if frame is not None:
                    latest_frame = frame

            now = time.monotonic()
            if latest is not None and now >= next_report:
                attached = latest.state == 1
                if latest_frame is not None:
                    print(
                        f"[{hand.device_name}][Tactile] "
                        f"attached={attached} {pressure_summary(latest_frame)}"
                    )
                else:
                    print(f"[{hand.device_name}][Tactile] attached={attached}")
                next_report = now + REPORT_SECONDS

            if status is None and frame is None:
                time.sleep(POLL_SLEEP_SECONDS)
    except KeyboardInterrupt:
        pass
    finally:
        status_sub.close()
        if frame_sub is not None:
            frame_sub.close()
        manager.disconnect_all()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
