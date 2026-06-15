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

- `wuji_glove/0.subscribe_callback.py`: subscribe to glove streams with callbacks; also prints the realtime-IK hand model selection.
- `wuji_glove/1.subscribe_async.py`: subscribe to glove streams with async/await; also prints the realtime-IK hand model selection.
- `wuji_glove/2.recording.py`: record glove data to MCAP.
- `wuji_glove/3.offline_pipeline.py`: run hand tracking from offline frames.
- `wuji_glove/4.user.py`: inspect, create, and switch local SDK users.
- `wuji_glove/5.calibration.py`: run IK calibration with terminal guidance or API-style callbacks.

## License

[MIT](https://github.com/wuji-technology/wuji-sdk/blob/main/LICENSE)
