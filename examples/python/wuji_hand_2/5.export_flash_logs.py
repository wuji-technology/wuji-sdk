"""Export the Wuji Hand 2 on-flash log history to a JSONL file.

Pulls the whole flash log ring, decodes each frame, and writes one JSON
object per line (`timestamp_ms` / `level` / `target` / `message`). Every call
performs a full export and writes a new file — there is no separate
"how many frames are there" query.

`--out` defaults to `None`, which writes into the SDK's default directory
`~/.wuji/logs` — the same place the SDK keeps its own logs, with a `flash_`
prefix on the filename. Pass `--out` to keep the export somewhere else.

The first discovered Wuji Hand 2 is used.

Usage:
    python 5.export_flash_logs.py
"""

import argparse
import json

from wuji_sdk import DeviceType, SdkManager


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out", default=None, help="output dir (default ~/.wuji/logs)")
    args = parser.parse_args()

    manager = SdkManager.instance()
    hands = [d for d in manager.scan() if d.device_type == DeviceType.WujiHand2]
    if not hands:
        print("No Wuji Hand 2 found")
        return

    try:
        hand = manager.connect(sn=hands[0].sn, device_name="wuji_hand_2")
        print(f"Connected: {hand.serial_number}")

        result = hand.export_flash_logs(out_dir=args.out)
        print(f"Decoded {result['frames']} frames into {result['path']}")

        if result["frames"]:
            with open(result["path"], "r", encoding="utf-8") as f:
                first = json.loads(f.readline())
            print(f"First entry: [{first['level']}] {first['target']}: {first['message']}")
    finally:
        manager.disconnect_all()


if __name__ == "__main__":
    main()
