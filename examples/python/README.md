# wuji-sdk (Python)

Python SDK for Wuji series devices. Provides automatic device discovery, connection management, and real-time data streaming for Wuji Glove, Wuji Hand 2, and other Wuji peripherals. Features a type-safe semantic API with native async/await and callback-based subscription support, multi-channel MCAP recording, and rich hand tracking data including joint angles, skeleton, and fingertip poses.

**For detailed documentation, see the [Wuji Docs Center](https://docs.wuji.tech/docs/en/wuji-glove/latest/).**

> Looking for the C SDK? See [the C SDK README](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/c/README.md).

## Installation

```bash
pip install wuji-sdk
```

When running examples from this repository against local source changes, rebuild
and install the Python extension into the active virtual environment first:

```bash
cd crates/sdk-python
maturin develop
```

## Quick Start

```python
import time
from wuji_sdk import SdkManager

manager = SdkManager.instance()
devices = manager.scan()
if not devices:
    print("No devices found")
    exit()
glove = manager.connect(sn=devices[0].sn, device_name="glove")

sub = glove.tactile().subscribe_with_callback(
    callback=lambda frame: print(f"Max pressure: {max(frame.data):.2f}")
)

time.sleep(10)
sub.close()
```

More examples: [examples/python/](https://github.com/wuji-technology/wuji-sdk/tree/main/examples/python).

## Examples

- `wuji_glove/0.subscribe_callback.py`: subscribe to glove streams with callbacks; pass `--hand-model-path` to set a custom online-IK URDF first.
- `wuji_glove/1.subscribe_async.py`: subscribe to glove streams with async/await; pass `--hand-model-path` to set a custom online-IK URDF first.
- `wuji_glove/2.recording.py`: record glove data to MCAP.
- `wuji_glove/3.offline_pipeline.py`: run hand tracking from offline frames.
- `wuji_glove/4.user.py`: inspect, create, and switch local SDK users.
- `wuji_glove/5.calibration.py`: run IK calibration with terminal guidance or API-style callbacks.
- `wuji_glove/6.emf_poses_rate_divider.py`: lower the EMF pose output rate and compare affected stream rates.
- `retargeting/0.retarget_session.py`: map hand keypoints (21×3) to a joint command with `RetargetSession` — no hardware needed.
- `retargeting/1.teleop_real.py`: live teleoperation — drive a Wuji Hand / Wuji Hand 2 from a Wuji Glove (an example built on `RetargetSession`; swap the glove read for any `(21,3)` keypoint source).

## Retargeting

Map human hand keypoints to Wuji Hand joint commands. Install the runtime dependencies first:

```bash
pip install "wuji-sdk[retarget]"
```

The SDK exposes the pure retarget interface — one frame at a time, supply keypoints from any source:

```python
import numpy as np
from wuji_sdk import Handedness, retargeting

# The hand model selects the builtin tuning config internally — no config path to manage.
session = retargeting.RetargetSession.for_hand(
    retargeting.HandModel.WujiHand2, side=Handedness.Right
)
qpos = session.step(np.zeros((21, 3), dtype=np.float32))  # -> (20,) joint command (firmware order)
```

Driving a hand live (read → retarget → send) is plain application code built on
this interface — see `retargeting/1.teleop_real.py` for a complete glove → hand loop.

Retargeting is available on Linux x86_64 / aarch64.

## License

[MIT](https://github.com/wuji-technology/wuji-sdk/blob/main/LICENSE)
