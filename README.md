# wuji-sdk

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE) [![Release](https://img.shields.io/github/v/release/wuji-technology/wuji-sdk?cacheSeconds=3600)](https://github.com/wuji-technology/wuji-sdk/releases)

SDKs for Wuji series devices (Wuji Glove, WujiHand, Wuji Hand 2, and other peripherals): automatic device discovery, connection management, and real-time data streaming — rich hand-tracking data (joint angles, skeleton, fingertip poses), tactile/EMF sensing, multi-channel MCAP recording, and hand retargeting (map keypoints to hand joint commands).

## SDKs

| Language | Install | Examples |
|----------|---------|----------|
| **Python** | `pip install wuji-sdk` | [examples/python/](examples/python/) |
| **C** | Prebuilt tarball on each [Release](https://github.com/wuji-technology/wuji-sdk/releases) | [examples/c/](examples/c/) |

Each language folder under examples/ includes a README covering setup and API usage.

The Python SDK is the primary, full-featured interface. The C SDK exposes a C API (a shared library — `libwuji_sdk_c.so` on Linux, `libwuji_sdk_c.dylib` on macOS — plus `wuji_sdk.h`) for native/embedded integration.

## Repository Structure

```text
├── docker/                  # Python SDK container example
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
├── LICENSE
└── README.md
```

## Documentation

For detailed documentation, see the [Wuji Docs Center](https://docs.wuji.tech/en/docs/wuji-sdk).

## Release notes

Release notes for both SDKs are published on the docs site: [Release Notes](https://docs.wuji.tech/en/release-notes).

## Contact

For any questions, please contact [support@wuji.tech](mailto:support@wuji.tech).

## License

[MIT](LICENSE)
