# wuji-sdk (C)

A prebuilt C SDK exposing the Wuji SDK over a C API: `libwuji_sdk_c.so` plus a generated `wuji_sdk.h`. It supports device discovery and connection, typed per-stream subscription callbacks (tactile, tactile zones, EMF poses, hand joint angles, hand skeleton, tactile point cloud), and global cross-device coordinate-transform streams (`tf`, `tf_static`). For the Wuji Hand 2 (dexterous hand) it adds typed get/set, control actions (enable/disable, clear faults, emergency stop, user origin — each with an optional per-joint mask), MIT impedance and effort-limit configuration with a per-joint online bitmap, `joint_states` / `joint_diagnostics` subscription streams, and a streaming joint-command publisher.

**For detailed documentation, see the [Wuji Docs Center](https://docs.wuji.tech/docs/en/wuji-glove/latest/).**

> Looking for the Python SDK? See [the Python SDK README](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/README.md).

## Download

The C SDK ships as a prebuilt tarball attached to each [public Release](https://github.com/wuji-technology/wuji-sdk/releases). Each tarball contains just `lib/`, `include/wuji_sdk.h`, and `LICENSE`. Three targets:

- `x86_64-linux-gnu` — desktop/server Linux (most dev machines).
- `aarch64-linux-gnu` — ARM64 Linux (SBCs, Jetson, etc.).
- `aarch64-android` — built for Android NDK integration. The `.so` file lives at `lib/arm64-v8a/` in the jniLibs layout. Drop it into an Android Studio project. It links against Android's bionic, not a host glibc.

## Supported platforms

| Target | Runs on | Built with |
|--------|---------|------------|
| `x86_64-linux-gnu` | Linux x86_64, glibc ≥ 2.35 (Ubuntu 22.04+, Debian 12+, RHEL 9+) | ubuntu-22.04, glibc baseline ≤ 2.35 |
| `aarch64-linux-gnu` | Linux ARM64, glibc ≥ 2.35 | ubuntu-22.04-arm (native), glibc baseline ≤ 2.35 |
| `aarch64-android` | Android 10+ (API level ≥ 29), ABI `arm64-v8a`, 16 KB page support | Android NDK r27c, API 29 (pure bionic) |

The SDK version itself is in the tarball filename (`wuji-sdk-c-<VER>-<target>.tar.gz`) and the Release tag.

> **Wuji Hand (dexterous hand):** supported on **Linux x86_64 / aarch64
> (gnu)**. On the Android tarball, the functions are present but return
> `WUJI_STATUS_ERR_UNSUPPORTED`.

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

- [`wuji_glove/`](https://github.com/wuji-technology/wuji-sdk/tree/main/examples/c/wuji_glove) — typed per-stream subscription callbacks, runtime stream rate control, online IK custom-URDF selection, SDK user management (`3_user_management`), and hand model calibration (`4_user_calibration`). It also includes three separate tactile examples: `5_tactile_calibration` is the guided one-shot calibration flow; `6_tactile_contact_view` independently subscribes to `tactile_binary`, renders the live 24x31 contact grid, and tunes sensitivity with `+` / `-` after calibration; and `7_tactile_residual_view` subscribes to the continuous signed `tactile_residual` stream and applies an adjustable contact threshold. Training is available in the linux-gnu C SDK builds with model training support.
- [`wuji_hand/`](https://github.com/wuji-technology/wuji-sdk/tree/main/examples/c/wuji_hand) — Wuji Hand (first generation), mirroring the Python examples one-to-one: `0_subscribe` (joint_states stream), `1_publish` (hold zero pose via the joint-command publisher + low-pass realtime controller), `2_grasp_loop` (continuous open/close grasp with target-vs-actual readout), and `3_tactile_status` (tactile glove status + pressure summary). Supported in the Linux x86_64 / aarch64 (gnu) tarballs only.
- [`wuji_hand_2/`](https://github.com/wuji-technology/wuji-sdk/tree/main/examples/c/wuji_hand_2) — Wuji Hand 2: `0_subscribe_joint_states` (joint-state stream), `1_control_motion` (drive the hand via the joint-command publisher), `2_hand_info` (identity and per-joint diagnostics), `3_fingertip` (fingertip sensor metadata and force point stream), `4_set_stream_rate` (runtime stream rate control), `5_export_flash_logs` (on-flash log export), `6_opposition` (bilateral recorded four-finger replay), and `7_mit_sweep` (fixed single-joint sweep).
- [`retargeting/`](https://github.com/wuji-technology/wuji-sdk/tree/main/examples/c/retargeting) — hand retargeting, mirroring the Python examples: `0_retarget_session` maps 21 MediaPipe-format hand keypoints to a 20-joint command via `wuji_retarget_session_*` (no hardware required); `1_teleop_real` connects a Wuji Glove and a Wuji Hand or Wuji Hand 2, reads `hand_skeleton` frames from the glove at up to 120 Hz, retargets each frame, and drives the hand in real time — glove keypoints feed directly into `wuji_retarget_session_step`, and the result is sent via the joint-command publisher (Wuji Hand 2) or the low-pass realtime controller (Wuji Hand). It lists the SDK users and asks which one to teleoperate under: press Enter to keep the current user, pick the default user for the built-in hand URDF, or pick a named user to use that user's calibrated hand model — an uncalibrated named user falls back to the built-in URDF.

Point an example's CMake at the extracted SDK:

```bash
cmake -S examples/c/wuji_glove -B build \
  -DWUJI_SDK_INCLUDE_DIR="${SDK}/include" \
  -DWUJI_SDK_LIB="${SDK}/lib/libwuji_sdk_c.so"
cmake --build build
```

### Wuji Hand 2 Motion Examples

> [!WARNING]
> Examples 6 and 7 enable motor control and send position commands. Keep
> the hand clear of people, cables, and the bench. Run with direct supervision,
> and be ready to stop the device.

Example 7 is a standalone source file. Example 6 is a standalone bundle that
contains `opposition.c`, its own CMake project, and Right and Left replay data.
Copy the complete `6_opposition/` directory to use that example.

Each entry accepts no command-line options. Examples 6 and 7 connect the only
discovered device, require all 20 joints online, enable the hand, and send a
fixed command sequence. Example 6 selects and streams one replay file from
device handedness.

#### Build the C Examples

Configure the Wuji Hand 2 directory against the extracted SDK, then build the
two motion targets:

```bash
cmake -S examples/c/wuji_hand_2 -B build/hand2 \
  -DWUJI_SDK_INCLUDE_DIR="${SDK}/include" \
  -DWUJI_SDK_LIB="${SDK}/lib/libwuji_sdk_c.so"
cmake --build build/hand2 \
  --target 6_opposition 7_mit_sweep
```

The Hand 2 CMake project builds all targets from 0 through 7 when you omit
`--target`.

Build the Example 6 bundle independently after copying its complete directory:

```bash
cmake -S examples/c/wuji_hand_2/6_opposition \
  -B build/hand2-opposition \
  -DWUJI_SDK_INCLUDE_DIR="${SDK}/include" \
  -DWUJI_SDK_LIB="${SDK}/lib/libwuji_sdk_c.so"
cmake --build build/hand2-opposition
```

The build copies `data/right.replay` and `data/left.replay` beside the
executable. Keep the generated `data/` directory with the binary.

#### Choose Example 6 or 7

| Example | Motion | Output |
| --- | --- | --- |
| `7_mit_sweep` | Moves joint 0 through a fixed 0.02 rad, 101-command cosine sweep while the other joints stay at zero | Publishes at 50 Hz, then disables and disconnects |
| `6_opposition` | Streams the connected side's recorded thumb opposition | Publishes 30,000 commands at 1 kHz, then disables and disconnects |

Example 7 provides one conservative joint motion. Example 6 replays a complete
recorded four-finger motion.

#### Run Example 7

Run the fixed supervised motion:

```bash
./build/hand2/7_mit_sweep
```

Example 7 sends a fixed 101-command cosine sweep. Joint 0 moves from zero to
0.02 rad and back to zero. The other 19 joints remain at zero.

#### Run Example 6

Auto-connect one Hand 2 and replay the recording selected by its handedness:

```bash
./build/hand2/6_opposition/6_opposition
```

Example 6 selects `data/right.replay` or `data/left.replay` from device
handedness. It validates the file header and length before enable, then streams
every recorded qpos once at a fixed 1 kHz command rate. It doesn't add
interpolation, resampling, smoothing, or recovery motion. The final recorded
frame is the return to Open.

#### Cleanup Behavior

Normal completion disables the hand after the final command. An error or
interrupt stops further commands and adds no recovery motion. Each example
disables the connected hand, closes its publisher, disconnects the device,
and closes the SDK. Runtime and cleanup errors return 1. Ctrl-C returns 130.

## License

[MIT](https://github.com/wuji-technology/wuji-sdk/blob/main/LICENSE)
