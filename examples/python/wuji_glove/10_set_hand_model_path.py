#!/usr/bin/env python3
"""
Purpose:
    Set and read back a custom hand model path.
Prerequisites:
    Install wuji-sdk, connect a Wuji Glove, and select a named SDK user.
    Provide a readable hand URDF file; the default SDK user cannot set one.
Usage:
    python 10_set_hand_model_path.py --hand-model-path /absolute/path/to/hand.urdf
    python 10_set_hand_model_path.py --hand-model-path /absolute/path/to/hand.urdf --sn DEVICE_SN
Expected Behavior:
    Print the configured URDF path, then disconnect.
    Exit 0 on completion or Ctrl-C, 1 on execution or cleanup failure,
    and 2 on invalid arguments. Errors go to stderr.
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from wuji_sdk import SdkManager, WujiGlove


def connect_device(manager, sn):
    if sn is not None:
        return manager.connect(sn=sn, device_name="example")
    return manager.auto_connect(device_name="example")


def main():
    parser = argparse.ArgumentParser(allow_abbrev=False)
    parser.add_argument("--sn", metavar="DEVICE_SN")
    parser.add_argument("--hand-model-path", type=Path, required=True)
    args = parser.parse_args()
    if args.sn == "":
        parser.error("--sn must not be empty")
    path = args.hand_model_path.expanduser().resolve()
    if not path.is_file():
        parser.error(f"URDF file not found: {path}")

    manager = None
    result = 0
    operation = "initialize SDK"
    try:
        manager = SdkManager.instance()
        operation = "connect"
        glove = connect_device(manager, args.sn)
        operation = "check device type"
        if not isinstance(glove, WujiGlove):
            raise RuntimeError(f"expected Wuji Glove, got {type(glove).__name__}")
        operation = "read serial number"
        print(f"Connected: {glove.serial_number}")
        operation = "set hand model path"
        glove.hand_model_path().set(str(path))
        operation = "read hand model path"
        print(f"hand_model_path: {glove.hand_model_path().get()}")
    except KeyboardInterrupt:
        pass
    except Exception as error:
        print(f"{operation} failed: {error}", file=sys.stderr)
        result = 1
    finally:
        if manager is not None:
            try:
                manager.disconnect_all()
            except Exception as error:
                print(f"disconnect failed: {error}", file=sys.stderr)
                result = 1
    return result


if __name__ == "__main__":
    sys.exit(main())
