# wuji-sdk

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE) [![Release](https://img.shields.io/github/v/release/wuji-technology/wuji-sdk)](https://github.com/wuji-technology/wuji-sdk/releases) ![Coverage](https://raw.githubusercontent.com/wuji-technology/wuji-sdk/badges/coverage.svg)

Python SDK for Wuji series devices. Provides automatic device discovery, connection management, and real-time data streaming for Wuji Glove and other Wuji peripherals. Features type-safe semantic API with native async/await and callback-based subscription support.

**Get started with [Quick Start](#quick-start). For detailed documentation, please refer to [Wuji Docs Center](https://docs.wuji.tech/docs/en/wuji-glove/latest/).**

## Repository Structure

```text
├── examples/                      # Example scripts demonstrating SDK usage patterns
│   ├── 0.subscribe_callback.py
│   └── 1.subscribe_async.py
├── CHANGELOG.md                   # Version history and release notes
├── LICENSE                        # MIT license file
└── README.md
```

## Quick Start

### Installation

```bash
pip install wuji-sdk
```

### Running

```python
import time
from wuji_sdk import SdkManager

manager = SdkManager.instance()
glove = manager.auto_connect(device_name="glove")

sub = glove.tactile().subscribe_with_callback(
    callback=lambda frame: print(f"Max pressure: {max(frame.data):.2f}")
)

time.sleep(10)
sub.close()
```

## Contact

For any questions, please contact [support@wuji.tech](mailto:support@wuji.tech).
