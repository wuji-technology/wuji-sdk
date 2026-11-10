# wuji-sdk (C)

A prebuilt C SDK exposing the Wuji SDK over a C API: a shared library (`libwuji_sdk_c.so` on Linux, `libwuji_sdk_c.dylib` on macOS, `wuji_sdk_c.dll` on Windows) plus a generated `wuji_sdk.h`. It supports device discovery and connection, typed per-stream subscription callbacks (tactile, tactile zones, EMF poses, hand joint angles, hand skeleton, tactile point cloud), and global cross-device coordinate-transform streams (`tf`, `tf_static`). For the Wuji Hand 2 (dexterous hand) it adds typed get/set, control actions (enable/disable, clear faults, emergency stop, user origin — each with an optional per-joint mask), MIT impedance and effort-limit configuration with a per-joint online bitmap, `joint_states` / `joint_diagnostics` subscription streams, and a streaming joint-command publisher.

**For detailed documentation, see the [Wuji Docs Center](https://docs.wuji.tech/docs/en/wuji-sdk/latest).**

> Looking for the Python SDK? See [the Python SDK README](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/README.md).

## Download

The C SDK ships as prebuilt archives attached to each [Release](https://github.com/wuji-technology/wuji-sdk/releases). Every archive includes `include/wuji_sdk.h`. Linux, macOS, and Android use a `.tar.gz` with `lib/` and `LICENSE`. Windows x64 uses a `.zip` with `bin/`, `lib/`, `LICENSE`, and `licenses/`. Five targets:

- `x86_64-linux-gnu` — desktop/server Linux (most dev machines).
- `aarch64-linux-gnu` — ARM64 Linux (SBCs, Jetson, etc.).
- `aarch64-macos` — Apple Silicon Macs. The library is `lib/libwuji_sdk_c.dylib`.
- `aarch64-android` — built for Android NDK integration. The `.so` file lives at `lib/arm64-v8a/` in the jniLibs layout. Drop it into an Android Studio project. It links against Android's bionic, not a host glibc.
- `x86_64-windows-gnullvm` — Windows x64. Link against `lib/libwuji_sdk_c.dll.a` and ship `bin/wuji_sdk_c.dll` and `bin/libunwind.dll` next to your executable.

## Supported platforms

| Target | Runs on |
|--------|---------|
| `x86_64-linux-gnu` | Linux x86_64, glibc ≥ 2.31 (Ubuntu 20.04+, Debian 11+, RHEL 9+) |
| `aarch64-linux-gnu` | Linux ARM64, glibc ≥ 2.31 (Ubuntu 20.04+, Debian 11+, RHEL 9+) |
| `aarch64-macos` | macOS 11.0 or later on Apple Silicon |
| `aarch64-android` | Android 10+ (API level ≥ 29), ABI `arm64-v8a` |
| `x86_64-windows-gnullvm` | Windows 10 21H2 x64 or later, Windows Server 2019+ |

Android artifacts link against bionic and support 16 KB pages.

The SDK version itself is in the archive filename (`wuji-sdk-c-<VER>-<target>.tar.gz`, Windows: `.zip`) and the Release tag.

> **Wuji Hand (dexterous hand):** supported on **Linux x86_64 / aarch64 (gnu)**
> and **macOS arm64**. On the Android and Windows packages, the functions are
> present but return `WUJI_STATUS_ERR_UNSUPPORTED`.

## Windows Examples

Windows supports the following examples with LLVM-MinGW and MSVC. Each CMake project builds only the listed target on Windows. The other examples are available for Linux and macOS.

| Example | Device | Function |
|---------|--------|----------|
| [`wuji_glove/0_subscribe_callback.c`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/c/wuji_glove/0_subscribe_callback.c) | Wuji Glove | Scan, connect, and subscribe with callbacks |
| [`wuji_hand_2/0_subscribe_joint_states.c`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/c/wuji_hand_2/0_subscribe_joint_states.c) | Wuji Hand 2 | Scan, connect, and subscribe to joint states |
| [`recording/0_record_topics.c`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/c/recording/0_record_topics.c) | Wuji Glove and Wuji Hand 2 | Connect by serial number and record selected topics, with pause and resume |

The subscription examples handle Ctrl+C and Ctrl+Break on Windows and release subscriptions and device connections before exiting.

## Quick Start

The commands below compile your C application saved as `my_app.c`, which includes `wuji_sdk.h`. For the supplied example source files, see [Build with CMake](#build-with-cmake).

### Linux and macOS

Download and extract the archive for your platform:

```bash
# Replace <TAG> with the version you want, e.g. v2026.5.26
TAG=<TAG>
VER="${TAG#v}"                              # tarball filenames drop the leading `v`
SDK="wuji-sdk-c-${VER}-x86_64-linux-gnu"    # swap the triple for your platform

curl -L "https://github.com/wuji-technology/wuji-sdk/releases/download/${TAG}/${SDK}.tar.gz" | tar xz
cc my_app.c -I "${SDK}/include" -L "${SDK}/lib" -lwuji_sdk_c -o my_app
```

Run your application on Linux:

```bash
LD_LIBRARY_PATH="${SDK}/lib" ./my_app
```

Run your application on macOS:

```bash
DYLD_LIBRARY_PATH="${SDK}/lib" ./my_app
```

### Windows

Use PowerShell with the LLVM-MinGW `clang` compiler on `PATH`. Choose a tag from [Release](https://github.com/wuji-technology/wuji-sdk/releases) that includes the Windows x64 archive, and replace `<TAG>` below with that tag.

```powershell
$Tag = "<TAG>"
$Archive = "wuji-sdk-c-$($Tag.TrimStart('v'))-x86_64-windows-gnullvm"
Invoke-WebRequest "https://github.com/wuji-technology/wuji-sdk/releases/download/$Tag/$Archive.zip" -OutFile "$Archive.zip"
Expand-Archive "$Archive.zip" -DestinationPath .
$Sdk = (Resolve-Path "$Archive").Path
clang my_app.c -I "$Sdk/include" "$Sdk/lib/libwuji_sdk_c.dll.a" -o my_app.exe
Copy-Item "$Sdk/bin/*.dll" .
./my_app.exe
```

Keep the copied DLLs beside your executable when distributing the application. To build the repository examples, see [Build with CMake](#build-with-cmake).

## Examples

The example directories contain source files and CMake projects. See [Build with CMake](#build-with-cmake) for build commands.

- [`wuji_glove/`](https://github.com/wuji-technology/wuji-sdk/tree/main/examples/c/wuji_glove) — typed per-stream subscription callbacks, runtime stream rate control, online IK custom-URDF selection, SDK user management (`3_user_management`), and hand model calibration (`4_user_calibration`). It also includes three separate tactile examples: `5_tactile_calibration` is the guided one-shot calibration flow; `6_tactile_contact_view` independently subscribes to `tactile_binary`, renders the live 24x31 contact grid, and tunes sensitivity with `+` / `-` after calibration; and `7_tactile_residual_view` subscribes to the continuous signed `tactile_residual` stream and applies an adjustable contact threshold. Training is available in the linux-gnu C SDK builds with model training support. `8_offline_pipeline` re-solves recorded data without a connected device.
- [`wuji_hand/`](https://github.com/wuji-technology/wuji-sdk/tree/main/examples/c/wuji_hand) — Wuji Hand (first generation), mirroring the Python examples one-to-one: `0_subscribe` (joint_states stream), `1_publish` (hold zero pose via the joint-command publisher + low-pass realtime controller), `2_grasp_loop` (continuous open/close grasp with target-vs-actual readout), and `3_tactile_status` (tactile glove status + pressure summary). Supported in the Linux x86_64 / aarch64 (gnu) and macOS arm64 tarballs.
- [`wuji_hand_2/`](https://github.com/wuji-technology/wuji-sdk/tree/main/examples/c/wuji_hand_2) — Wuji Hand 2: `0_subscribe_joint_states` (joint-state stream), `1_control_motion` (drive the hand via the joint-command publisher), `2_hand_info` (identity and per-joint diagnostics), `3_fingertip` (fingertip sensor metadata and force point stream), `4_set_stream_rate` (runtime stream rate control), `5_export_flash_logs` (on-flash log export), `6_opposition` (bilateral recorded four-finger replay), and `7_mit_sweep` (fixed single-joint sweep). Read the [motion example safety instructions](#wuji-hand-2-motion-examples) before running examples 6 and 7.
- [`retargeting/`](https://github.com/wuji-technology/wuji-sdk/tree/main/examples/c/retargeting) — hand retargeting, mirroring the Python examples: `0_retarget_session` maps 21 MediaPipe-format hand keypoints to a 20-joint command via `wuji_retarget_session_*` (no hardware required); `1_teleop_real` connects a Wuji Glove and a Wuji Hand or Wuji Hand 2, reads `hand_skeleton` frames from the glove at up to 120 Hz, retargets each frame, and drives the hand in real time — glove keypoints feed directly into `wuji_retarget_session_step`, and the result is sent via the joint-command publisher (Wuji Hand 2) or the low-pass realtime controller (Wuji Hand). It lists the SDK users and asks which one to teleoperate under: press Enter to keep the current user, pick the default user for the built-in hand URDF, or pick a named user to use that user's calibrated hand model — an uncalibrated named user falls back to the built-in URDF.
- [`recording/`](https://github.com/wuji-technology/wuji-sdk/tree/main/examples/c/recording) includes `0_record_topics`, which records public topics to MCAP with LZ4, Zstd, or no compression. It records for 30 seconds, pauses for 10 seconds, and records for another 30 seconds before printing per-channel counts. See the command at the start of [0_record_topics.c](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/c/recording/0_record_topics.c).

## Build with CMake

Download and extract the SDK as described in [Quick Start](#quick-start). Run the following commands from the repository root, where `examples/c/` is available. Set `SDK` (Linux/macOS) or `Sdk` (PowerShell) to the absolute path of the extracted SDK directory.

### Linux and macOS

On Linux, use `libwuji_sdk_c.so`. On macOS, replace it with `libwuji_sdk_c.dylib`.

```bash
cmake -S examples/c/wuji_glove -B build \
  -DWUJI_SDK_INCLUDE_DIR="${SDK}/include" \
  -DWUJI_SDK_LIB="${SDK}/lib/libwuji_sdk_c.so"
cmake --build build
```

These CMake projects embed an RPATH, so their executables need no `LD_LIBRARY_PATH` or `DYLD_LIBRARY_PATH` setting.

### Windows with LLVM-MinGW

Put LLVM-MinGW, CMake, and Ninja on `PATH`. Link the import library and pass `bin/` as the runtime directory. The build copies the DLLs beside the executable.

```powershell
cmake -S examples/c/wuji_glove -B build -G Ninja `
  -DCMAKE_C_COMPILER=clang `
  -DWUJI_SDK_INCLUDE_DIR="$Sdk/include" `
  -DWUJI_SDK_LIB="$Sdk/lib/libwuji_sdk_c.dll.a" `
  -DWUJI_SDK_RUNTIME_DIR="$Sdk/bin"
cmake --build build
```

### Windows with MSVC

Use Developer PowerShell for Visual Studio 2022 17.5 or later, with CMake and Ninja on `PATH`. Use a separate build directory for each compiler. The build copies the DLLs beside the executable.

```powershell
cmake -S examples/c/wuji_glove -B build-msvc -G Ninja `
  -DCMAKE_C_COMPILER=cl -DCMAKE_BUILD_TYPE=Release `
  -DWUJI_SDK_INCLUDE_DIR="$Sdk/include" `
  -DWUJI_SDK_LIB="$Sdk/lib/libwuji_sdk_c.dll.a" `
  -DWUJI_SDK_RUNTIME_DIR="$Sdk/bin"
cmake --build build-msvc
```

## Wuji Hand 2 Motion Examples

> [!WARNING]
> Examples 6 and 7 enable motor control and send position commands. On start
> they read the current pose from `joint_states` and ease the hand to the
> trajectory start over about one second, so the hand begins moving toward a
> fully extended pose immediately after launch. Keep the hand clear of people,
> cables, and the bench, place it in a safe pose before running, run with
> direct supervision, and be ready to stop the device.

Example 7 is a standalone source file. Example 6 is a standalone bundle that
contains `opposition.c`, its own CMake project, and Right and Left replay data.
Copy the complete `6_opposition/` directory to use that example.

Each entry accepts no command-line options. Examples 6 and 7 scan for Wuji
Hand 2 devices, list the discovered serial numbers, and connect the hand you
pick by entering its candidate number. When several hands are on the network,
the prompt lists each numbered candidate (Enter - or a closed stdin - picks the
first). They require all 20 joints online, enable the hand, ramp from the
current pose to the trajectory start, and then send a fixed command sequence.
Example 6 selects and streams one replay file from device handedness.

### Build the C Examples

Configure the Wuji Hand 2 directory against the extracted SDK, then build the
two motion targets:

```bash
cmake -S examples/c/wuji_hand_2 -B build/hand2 \
  -DWUJI_SDK_INCLUDE_DIR="${SDK}/include" \
  -DWUJI_SDK_LIB="${SDK}/lib/libwuji_sdk_c.so"   # macOS: libwuji_sdk_c.dylib
cmake --build build/hand2 \
  --target 6_opposition 7_mit_sweep
```

On Linux and macOS, the Hand 2 CMake project builds all targets from 0 through 7 when you omit `--target`. On Windows, it builds only `0_subscribe_joint_states`.

Build the Example 6 bundle independently after copying its complete directory:

```bash
cmake -S examples/c/wuji_hand_2/6_opposition \
  -B build/hand2-opposition \
  -DWUJI_SDK_INCLUDE_DIR="${SDK}/include" \
  -DWUJI_SDK_LIB="${SDK}/lib/libwuji_sdk_c.so"   # macOS: libwuji_sdk_c.dylib
cmake --build build/hand2-opposition
```

The build copies `data/right.replay` and `data/left.replay` beside the
executable. Keep the generated `data/` directory with the binary.

### Choose Example 6 or 7

| Example | Motion | Output |
| --- | --- | --- |
| `7_mit_sweep` | Moves joint 0 through a fixed 0.02 rad, 101-command cosine sweep while the other joints stay at zero | Ramps to the start pose at 1 kHz, publishes the sweep at 50 Hz, then disables and disconnects |
| `6_opposition` | Streams the connected side's recorded thumb opposition | Ramps to the start pose at 1 kHz, publishes 30,000 recorded commands at 1 kHz, then disables and disconnects |

Example 7 provides one conservative joint motion. Example 6 replays a complete
recorded four-finger motion.

### Run Example 7

Run the fixed supervised motion:

```bash
./build/hand2/7_mit_sweep
```

Example 7 sends a fixed 101-command cosine sweep. Joint 0 moves from zero to
0.02 rad and back to zero. The other 19 joints remain at zero.

### Run Example 6

Connect one Wuji Hand 2 and replay the recording selected by its handedness:

```bash
./build/hand2/6_opposition/6_opposition
```

Example 6 selects `data/right.replay` or `data/left.replay` from device
handedness. It validates the file header and length before enable, then streams
every recorded qpos once at a fixed 1 kHz command rate. The recorded sequence
itself adds no interpolation, resampling, smoothing, or recovery motion; before
it starts, the example ramps from the captured current pose to the first
recorded frame over about one second. The final recorded frame is the return
to Open.

### Cleanup Behavior

Normal completion disables the hand after the final command. An error or
interrupt stops further commands and adds no recovery motion. Each example
disables the connected hand, closes its publisher, disconnects the device,
and closes the SDK. Runtime and cleanup errors return 1. Ctrl-C returns 130.

## License

[MIT](https://github.com/wuji-technology/wuji-sdk/blob/main/LICENSE)
