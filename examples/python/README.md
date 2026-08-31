# wuji-sdk (Python)

Python SDK for Wuji series devices. Provides automatic device discovery, connection management, and real-time data streaming for Wuji Glove, Wuji Hand 2, and other Wuji peripherals. Features a type-safe semantic API with native async/await and callback-based subscription support, multi-channel MCAP recording, and rich hand tracking data including joint angles, skeleton, and fingertip poses.

**For detailed documentation, see the [Wuji Docs Center](https://docs.wuji.tech/docs/en/wuji-glove/latest/).**

> Looking for the C SDK? See [the C SDK README](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/c/README.md).

## Installation

```bash
pip install wuji-sdk
```

## Quick Start

```python
import time
from wuji_sdk import SdkManager, DeviceType

manager = SdkManager.instance()
# scan() reports each device's type, so connect only the glove.
gloves = [d for d in manager.scan() if d.device_type == DeviceType.WujiGlove]
if not gloves:
    print("No Wuji Glove found")
    exit()
glove = manager.connect(sn=gloves[0].sn, device_name="glove")

sub = glove.tactile().subscribe_with_callback(
    callback=lambda frame: print(f"Max pressure: {max(frame.data):.2f}")
)

time.sleep(10)
sub.close()
```

More examples: [examples/python/](https://github.com/wuji-technology/wuji-sdk/tree/main/examples/python).

## Examples

- `wuji_glove/0.subscribe_callback.py`: subscribe to glove streams with callbacks. Pass `--hand-model-path` to set a custom online-IK URDF first.
- `wuji_glove/1.subscribe_async.py`: subscribe to glove streams with async/await. Pass `--hand-model-path` to set a custom online-IK URDF first.
- `wuji_glove/2.recording.py`: record glove data to MCAP.
- `wuji_glove/3.offline_pipeline.py`: run hand tracking from offline frames.
- `wuji_glove/4.user.py`: inspect, create, and switch local SDK users.
- `wuji_glove/5.calibration.py`: run hand model calibration with terminal guidance or API-style callbacks.
- `wuji_glove/6.set_stream_rate.py`: lower the glove's raw pose output rate at runtime with `subscription.set_rate()` on `emf_poses`; derived streams follow automatically.
- `wuji_glove/7.tactile_calibration.py`: run the recommended guided tactile-calibration flow.
- `wuji_glove/8.tactile_contact_view.py`: independently observe the live `tactile_binary` contact grid after calibration. It is not a calibration workflow.
- `wuji_glove/9.tactile_residual_view.py`: observe the continuous signed `tactile_residual` grid after calibration and apply an adjustable contact threshold.
- `wuji_hand_2/5.export_flash_logs.py`: export the device's on-flash log history to a JSONL file.
- `wuji_hand_2/6_opposition/opposition.py`: stream either side's recorded four-finger motion at 1 kHz.
- `wuji_hand_2/7.mit_sweep.py`: run the fixed joint-0 cosine sweep, or render the command sequence offline.
- `retargeting/0.retarget_session.py`: map hand keypoints (21×3) to a joint command with `RetargetSession` — no hardware needed.
- `retargeting/1.teleop_real.py`: live teleoperation — drive a Wuji Hand / Wuji Hand 2 from a Wuji Glove. This example uses `RetargetSession`, and you can replace the glove read with any `(21,3)` keypoint source. It lists the SDK users and asks which one to teleoperate under. Press Enter to keep the current user, pick the default user for the built-in hand URDF, or pick a named user to use that user's calibrated hand model. An uncalibrated named user falls back to the built-in URDF.



### Wuji Hand 2 Motion Examples

> [!WARNING]
> Examples 6 and 7 enable motor control and send position commands. Keep the
> hand clear of people, cables, and the bench. Run with direct supervision,
> and be ready to stop the device.

Example 7 is a standalone source file. Example 6 is a standalone bundle that
contains `opposition.py` and Right and Left replay data. Copy the complete
`6_opposition/` directory to use that example.

Both entries accept no command-line options. They auto-connect one device,
require all 20 joints online, enable the hand, and send a fixed command
sequence. Example 6 selects and streams one replay file from device
handedness.

#### Choose Example 6 or 7

| Example | Motion | Output |
| --- | --- | --- |
| `7.mit_sweep.py` | Moves joint 0 through a fixed 0.02 rad, 101-command cosine sweep while the other joints stay at zero | Publishes at 50 Hz, then disables and disconnects |
| `6_opposition/opposition.py` | Streams the connected side's recorded thumb opposition | Publishes 30,000 commands at 1 kHz, then disables and disconnects |

Example 7 is the smaller motion check. Example 6 replays a complete recorded
four-finger motion.

#### Run Example 7

Run the fixed supervised motion:

```bash
python examples/python/wuji_hand_2/7.mit_sweep.py
```

Example 7 sends a fixed 101-command cosine sweep. Joint 0 moves from zero to
0.02 rad and back to zero. The other 19 joints remain at zero.

#### Run Example 6

Auto-connect one Hand 2 and replay the recording selected by its handedness:

```bash
python examples/python/wuji_hand_2/6_opposition/opposition.py
```

Example 6 selects `data/right.replay` or `data/left.replay` from device
handedness. It validates the file header and length before enable, then streams
every recorded qpos once at a fixed 1 kHz command rate. It doesn't add
interpolation, resampling, smoothing, or recovery motion. The final recorded
frame is the return to Open.

#### Cleanup Behavior

Normal completion disables the hand after the final command. An error or
interrupt stops further commands and adds no recovery motion. Both examples
disable the connected hand, close their publishers, and disconnect the
device. Runtime and cleanup errors return 1. Ctrl-C returns 130.

## Retargeting

Map human hand keypoints to Wuji Hand joint commands. Retargeting works out of the box — numpy is the only extra dependency (keypoint/qpos arrays):

```bash
pip install wuji-sdk numpy
```

The SDK exposes the pure retarget interface — one frame at a time, supply keypoints from any source:

```python
import numpy as np
from wuji_sdk import Handedness, HandModel, RetargetSession

# The hand model selects the builtin tuning config internally — no config path to manage.
session = RetargetSession.for_hand(HandModel.WujiHand2, side=Handedness.Right)

# A synthetic open right hand, (21, 3) in meters, MediaPipe landmark order —
# replace with your real keypoint source (camera / glove / replay).
# (All-zero / degenerate frames are rejected with an exception.)
keypoints = np.zeros((21, 3), dtype=np.float32)
for finger, x in enumerate([-0.04, -0.03, -0.01, 0.01, 0.03]):  # thumb..pinky
    for k in range(4):
        keypoints[1 + finger * 4 + k] = [x, 0.03 * (k + 1), 0.0]
keypoints[1] = [-0.03, 0.02, 0.01]  # thumb CMC nearer the palm

qpos = session.step(keypoints)  # -> (20,) joint command (firmware order)
```

Driving a hand live (read → retarget → send) is plain application code built on
this interface — see `retargeting/1.teleop_real.py` for a complete glove → hand loop.

Retargeting is available on Linux x86_64 / aarch64.

## License

[MIT](https://github.com/wuji-technology/wuji-sdk/blob/main/LICENSE)
