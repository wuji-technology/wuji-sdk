# wuji-sdk (C)

A prebuilt C SDK exposing the Wuji SDK over a C API: `libwuji_sdk_c.so` plus a generated `wuji_sdk.h`. It supports device discovery and connection, typed per-stream subscription callbacks (tactile, tactile zones, EMF poses, hand joint angles, hand skeleton, tactile point cloud), and global cross-device coordinate-transform streams (`tf`, `tf_static`). For the Wuji Hand 2 (dexterous hand) it adds typed get/set, control actions (enable/disable, clear faults, emergency stop, user origin — each with an optional per-joint mask), MIT impedance and effort-limit configuration with a per-joint online bitmap, `joint_states` / `joint_diagnostics` subscription streams, and a streaming joint-command publisher.

**For detailed documentation, see the [Wuji Docs Center](https://docs.wuji.tech/docs/en/wuji-glove/latest/).**

> Looking for the Python SDK? See [the Python SDK README](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/README.md).

## Download

The C SDK ships as a prebuilt tarball attached to each [public Release](https://github.com/wuji-technology/wuji-sdk/releases). Each tarball contains just `lib/`, `include/wuji_sdk.h`, and `LICENSE`. Three targets:

- `x86_64-linux-gnu` — desktop/server Linux (most dev machines).
- `aarch64-linux-gnu` — ARM64 Linux (SBCs, Jetson, etc.).
- `aarch64-android` — built for Android NDK integration; `.so` lives at `lib/arm64-v8a/` (jniLibs layout). Drop it into an Android Studio project; it links against Android's bionic, not a host glibc.

## Supported platforms

| Target | Runs on | Built with |
|--------|---------|------------|
| `x86_64-linux-gnu` | Linux x86_64, glibc ≥ 2.35 (Ubuntu 22.04+, Debian 12+, RHEL 9+) | ubuntu-22.04, glibc baseline ≤ 2.35 |
| `aarch64-linux-gnu` | Linux ARM64, glibc ≥ 2.35 | ubuntu-22.04-arm (native), glibc baseline ≤ 2.35 |
| `aarch64-android` | Android 10+ (API level ≥ 29), ABI `arm64-v8a`, 16 KB page support | Android NDK r27c, API 29 (pure bionic) |

The SDK version itself is in the tarball filename (`wuji-sdk-c-<VER>-<target>.tar.gz`) and the Release tag.

## Quick Start

Download and extract the tarball for your platform (you get `lib/`, `include/`, `LICENSE`):

```bash
# Replace <TAG> with the version you want, e.g. v2026.5.26
TAG=<TAG>
VER="${TAG#v}"                              # tarball filenames drop the leading `v`
SDK="wuji-sdk-c-${VER}-x86_64-linux-gnu"    # swap the triple for your platform

curl -L "https://github.com/wuji-technology/wuji-sdk/releases/download/${TAG}/${SDK}.tar.gz" | tar xz
```

Compile your program against the header and shared library:

```bash
cc my_app.c -I "${SDK}/include" -L "${SDK}/lib" -lwuji_sdk_c -o my_app
LD_LIBRARY_PATH="${SDK}/lib" ./my_app
```

Complete worked examples (with a CMake build) live in the repo under
[examples/c/](https://github.com/wuji-technology/wuji-sdk/tree/main/examples/c):

- [`wuji_glove/`](https://github.com/wuji-technology/wuji-sdk/tree/main/examples/c/wuji_glove) — typed per-stream subscription callbacks, EMF/IK rate-divider control, and online IK custom-URDF selection.
- [`wuji_hand_2/`](https://github.com/wuji-technology/wuji-sdk/tree/main/examples/c/wuji_hand_2) — Wuji Hand 2: `0_subscribe_joint_states` (joint-state stream), `1_control_motion` (drive the hand via the joint-command publisher), and `2_hand_info` (identity + per-joint diagnostics, showing the online bitmap).

Point an example's CMake at the extracted SDK:

```bash
cmake -S examples/c/wuji_glove -B build \
  -DWUJI_SDK_INCLUDE_DIR="${SDK}/include" \
  -DWUJI_SDK_LIB="${SDK}/lib/libwuji_sdk_c.so"
cmake --build build && ./build/0_subscribe_callback
```

To test online IK with a custom URDF from C:

```bash
cmake --build build
./build/2_set_hand_model_path /absolute/path/to/hand.urdf [serial_number]
```

The example calls `wuji_glove_set_hand_model_path()`, reads it back with
`wuji_glove_get_hand_model_path()`, then subscribes to `hand_joint_angles` and
prints the online IK frame confidences and angles.

## License

[MIT](https://github.com/wuji-technology/wuji-sdk/blob/main/LICENSE)
