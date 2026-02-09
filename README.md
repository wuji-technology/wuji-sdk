# wuji-sdk

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE) [![Release](https://img.shields.io/github/v/release/wuji-technology/wuji-sdk)](https://github.com/wuji-technology/wuji-sdk/releases)

Python SDK for Wuji series devices. Provides automatic device discovery, connection management, and real-time data streaming for Wuji Glove and other Wuji peripherals. Features type-safe semantic API with native async/await and callback-based subscription support.

## Table of Contents

- [Repository Structure](#repository-structure)
- [Usage](#usage)
  - [Callback Subscription](#callback-subscription)
  - [Async Subscription](#async-subscription)
  - [Supported Devices](#supported-devices)
  - [Available Data Streams](#available-data-streams)
  - [Examples](#examples)
  - [Requirements](#requirements)
- [Contact](#contact)

## Repository Structure

```text
├── examples/
│   ├── 0.subscribe_callback.py
│   └── 1.subscribe_async.py
├── CHANGELOG.md
├── LICENSE
└── README.md
```

### Directory Description

| Directory | Description |
|-----------|-------------|
| `examples/` | Example scripts demonstrating SDK usage patterns |
| `CHANGELOG.md` | Version history and release notes |
| `LICENSE` | MIT license file |

## Usage

### Installation

```bash
pip install wuji-sdk
```

### Callback Subscription

```python
import time
from wuji_sdk import SdkManager

def on_tactile(frame):
    print(f"Max pressure: {max(frame.data):.2f}")

manager = SdkManager.instance()
glove = manager.auto_connect(device_name="glove")

# Subscribe with callback (auto background thread)
sub = glove.tactile().subscribe_with_callback(callback=on_tactile)

time.sleep(10)
sub.close()
```

### Async Subscription

```python
import asyncio
from wuji_sdk import SdkManager

async def main():
    manager = SdkManager.instance()

    # Auto discover and connect device
    glove = manager.auto_connect(device_name="glove")

    print(f"Device: {glove.device_name}")
    print(f"Serial: {glove.serial_number}")
    print(f"Hand:   {glove.hand_side().get()}")

    # Subscribe to data streams
    tactile_sub = glove.tactile().subscribe()
    emf_sub = glove.emf_poses().subscribe()

    # Receive data
    while True:
        tactile = await tactile_sub.recv_async()
        print(f"Tactile max: {max(tactile.data):.2f}")

asyncio.run(main())
```

### Supported Devices

| Device | Features |
|--------|----------|
| Wuji Glove | Tactile, EMF pose data |

### Available Data Streams

| Stream | Type | Description |
|--------|------|-------------|
| `tactile()` | TactileFrame | 24 * 32 calibrated f32 pressure values (0.0~1.0) |
| `tactile_zones()` | TactileZones | Pressure by finger zone (palm, thumb, index, etc.) |
| `emf_poses()` | EmfPoseArray | 5-finger position and orientation |

### Examples

See [examples/](examples/) for complete example code.

| Example | Description |
|---------|-------------|
| [0.subscribe_callback.py](examples/0.subscribe_callback.py) | Subscribe using background callbacks |
| [1.subscribe_async.py](examples/1.subscribe_async.py) | Subscribe using async/await with `recv_async()` |

### Requirements

- Python >= 3.10
- Linux x86_64 (more platforms planned)

## Contact

For any questions, please contact [support@wuji.tech](mailto:support@wuji.tech).
