# wuji-sdk

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE) [![Release](https://img.shields.io/github/v/release/wuji-technology/wuji-sdk?cacheSeconds=3600)](https://github.com/wuji-technology/wuji-sdk/releases) ![Coverage](https://raw.githubusercontent.com/wuji-technology/wuji-sdk/badges/coverage.svg)

SDKs for Wuji series devices (Wuji Glove, WujiHand, Wuji Hand 2, and other peripherals): automatic device discovery, connection management, and real-time data streaming — rich hand-tracking data (joint angles, skeleton, fingertip poses), tactile/EMF sensing, multi-channel MCAP recording, and hand retargeting (map keypoints to hand joint commands).

## SDKs

| Language | Install | Examples |
|----------|---------|----------|
| **Python** | `pip install wuji-sdk` | [examples/python/](examples/python/) |
| **C** | Prebuilt tarball on each [Release](https://github.com/wuji-technology/wuji-sdk/releases) | [examples/c/](examples/c/) |

Each language folder under examples/ includes a README covering setup and API usage.

The Python SDK is the primary, full-featured interface. The C SDK exposes a C API (`libwuji_sdk_c.so` + `wuji_sdk.h`) for native/embedded integration.

## Repository Structure

```text
├── examples/
│   ├── python/              # Python SDK examples and docs (pip install wuji-sdk)
│   │   ├── README.md
│   │   ├── wuji_glove/
│   │   ├── wuji_hand/
│   │   ├── wuji_hand_2/
│   │   └── retargeting/     # map hand keypoints → joint commands
│   └── c/                   # C SDK examples and docs (prebuilt tarball from Releases)
│       ├── README.md
│       ├── wuji_glove/
│       ├── wuji_hand/
│       ├── wuji_hand_2/
│       └── retargeting/
├── CHANGELOG.md             # Version history (Python + C SDK)
├── LICENSE
└── README.md
```

## Documentation

For detailed documentation, see the [Wuji Docs Center](https://docs.wuji.tech/docs/en/wuji-glove/latest/).

## Changelog

See [CHANGELOG.md](CHANGELOG.md) for the version history of both SDKs.

## Contact

For any questions, please contact [support@wuji.tech](mailto:support@wuji.tech).

## License

[MIT](LICENSE)
