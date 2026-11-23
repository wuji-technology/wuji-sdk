# wuji-sdk (C)

C examples for Wuji Glove, Wuji Hand 2, Wuji Hand, and hand retargeting. Build them against the prebuilt SDK header and shared library.

See the [SDK documentation](https://docs.wuji.tech/docs/en/wuji-sdk/latest) for API details and the [Python examples](../python/README.md) for the Python SDK.

## Download

Download the C SDK archive for your platform from [Releases](https://github.com/wuji-technology/wuji-sdk/releases) and extract it. Download or clone the [repository](https://github.com/wuji-technology/wuji-sdk) for the example source files. Use the source tag that matches your SDK release.

Linux, macOS, and Android archives use the name `wuji-sdk-c-<VERSION>-<TARGET>.tar.gz`. Windows uses `wuji-sdk-c-<VERSION>-x86_64-windows-gnullvm.zip`. Each archive includes `include/wuji_sdk.h` and the library for its target. The Windows archive also contains runtime DLLs in `bin/` and dependency licenses in `licenses/`.

## Supported Platforms

| Target | Requirements | Library |
| --- | --- | --- |
| `x86_64-linux-gnu` | Linux x86_64, glibc 2.31 or later | `lib/libwuji_sdk_c.so` |
| `aarch64-linux-gnu` | Linux ARM64, glibc 2.31 or later | `lib/libwuji_sdk_c.so` |
| `aarch64-macos` | macOS 11.0 or later on Apple Silicon | `lib/libwuji_sdk_c.dylib` |
| `aarch64-android` | Android 10 or later, API level 29 or later, `arm64-v8a` | `lib/arm64-v8a/libwuji_sdk_c.so` |
| `x86_64-windows-gnullvm` | Windows 10 21H2 x64 or later, or Windows Server 2019 or later | `lib/libwuji_sdk_c.dll.a` and `bin/wuji_sdk_c.dll` |

The Android library uses bionic, supports 16 KB pages, and is intended for Android NDK integration. The build commands below target desktop platforms.

Wuji Hand requires the Linux or macOS SDK package. Its functions return `WUJI_STATUS_ERR_UNSUPPORTED` in the Android and Windows packages. See [Windows Examples](#windows-examples) for the Windows example list.

## Build with CMake

Install CMake 3.16 or later and a C11 compiler. Run the commands from the repository root, which contains `examples/`. Replace `path/to/extracted-sdk` with the SDK directory you extracted.

### Linux and macOS

Set the SDK path and library. On macOS, use `libwuji_sdk_c.dylib` in place of `libwuji_sdk_c.so`:

```bash
SDK="$(cd path/to/extracted-sdk && pwd)"
SDK_LIB="${SDK}/lib/libwuji_sdk_c.so"
cmake -S examples/c/wuji_glove -B build/glove \
  -DWUJI_SDK_INCLUDE_DIR="${SDK}/include" \
  -DWUJI_SDK_LIB="${SDK_LIB}"
cmake --build build/glove
```

CMake embeds the SDK library path in the executables, so no `LD_LIBRARY_PATH` or `DYLD_LIBRARY_PATH` setting is needed while the SDK remains at that path.

### Windows with LLVM-MinGW

Put LLVM-MinGW, CMake, and Ninja on `PATH`, then run in PowerShell:

```powershell
$Sdk = (Resolve-Path "path/to/extracted-sdk").Path
cmake -S examples/c/wuji_glove -B build/glove -G Ninja `
  -DCMAKE_C_COMPILER=clang `
  -DWUJI_SDK_INCLUDE_DIR="$Sdk/include" `
  -DWUJI_SDK_LIB="$Sdk/lib/libwuji_sdk_c.dll.a" `
  -DWUJI_SDK_RUNTIME_DIR="$Sdk/bin"
cmake --build build/glove
```

### Windows with MSVC

Use Developer PowerShell for Visual Studio 2022 17.5 or later, with CMake and Ninja on `PATH`. Use a separate build directory for each compiler:

```powershell
$Sdk = (Resolve-Path "path/to/extracted-sdk").Path
cmake -S examples/c/wuji_glove -B build/glove-msvc -G Ninja `
  -DCMAKE_C_COMPILER=cl -DCMAKE_BUILD_TYPE=Release `
  -DWUJI_SDK_INCLUDE_DIR="$Sdk/include" `
  -DWUJI_SDK_LIB="$Sdk/lib/libwuji_sdk_c.dll.a" `
  -DWUJI_SDK_RUNTIME_DIR="$Sdk/bin"
cmake --build build/glove-msvc
```

Both Windows builds copy the runtime DLLs beside the executables. Keep those DLLs with the executables when moving the build.

## Quick Start

After building the Wuji Glove examples, scan and list devices, then connect one Wuji Glove and print its identity:

```bash
./build/glove/0_connect
```

Automatic connection requires exactly one discovered device. If several devices are present, use the serial number from the scan:

```bash
./build/glove/0_connect --sn DEVICE_SN
```

On Windows, run `./build/glove/0_connect.exe` in PowerShell. For the MSVC build above, use `build/glove-msvc/` instead.

To use another category, replace `examples/c/wuji_glove` in the CMake command with `examples/c/wuji_hand_2`, `examples/c/wuji_hand`, or `examples/c/retargeting`, and choose a separate build directory. Each category has its own CMake project. For ordinary examples, run the executable named after the source file without the `.c` extension. For Wuji Hand 2, start with [`0_connect.c`](wuji_hand_2/0_connect.c).

## Examples

Each source file starts with its prerequisites, usage, and expected behavior. Ordinary examples are standalone files. Copy the complete directory for examples with bundled data. Corresponding Python and C examples use the same number and name. Python-only async subscription examples appear last.

The C SDK uses callback subscriptions. It has no async/await equivalent of the Python `subscribe_async` examples.

### Wuji Glove

| Example | Function | Preparation |
| --- | --- | --- |
| [`0_connect.c`](wuji_glove/0_connect.c) | Scan, connect, and list parameters and topics. Set connection options when using a serial number. | One Wuji Glove. |
| [`1_subscribe_callback.c`](wuji_glove/1_subscribe_callback.c) | Receive glove streams, including palm IMU and fingertip poses, through callbacks. | One Wuji Glove. |
| [`2_recording.c`](wuji_glove/2_recording.c) | Record tactile, EMF pose, and hand skeleton streams to MCAP, with pause, resume, live status, quality metrics, and alerts. | Supply `--output FILE`. |
| [`3_offline_pipeline.c`](wuji_glove/3_offline_pipeline.c) | Compute hand tracking and IMU fusion from synthetic frames. | No device required. |
| [`4_user.c`](wuji_glove/4_user.c) | Manage SDK users and export, validate, or import their data. | No device required. |
| [`5_calibration.c`](wuji_glove/5_calibration.c) | Run hand model calibration with terminal guidance or text feedback. | Select a named SDK user. |
| [`6_set_stream_rate.c`](wuji_glove/6_set_stream_rate.c) | Set the EMF pose stream rate and restore its default. | One Wuji Glove. |
| [`7_tactile_calibration.c`](wuji_glove/7_tactile_calibration.c) | Collect motions, review recordings, and train a tactile contact model. | SDK build with model training support. |
| [`8_tactile_contact_view.c`](wuji_glove/8_tactile_contact_view.c) | Display tactile contacts and adjust sensitivity. | Calibrated Wuji Glove. |
| [`9_tactile_residual_view.c`](wuji_glove/9_tactile_residual_view.c) | Display tactile residuals and adjust the display threshold. | Calibrated Wuji Glove. |
| [`10_set_hand_model_path.c`](wuji_glove/10_set_hand_model_path.c) | Set and read back a custom hand model path. | Named SDK user and `--hand-model-path FILE`. |
| [`11_time_sync.c`](wuji_glove/11_time_sync.c) | Synchronize the glove clock and print timing measurements. | One Wuji Glove. |
| [`12_network_config.c`](wuji_glove/12_network_config.c) | Read IP and port, and optionally write new settings. | Supply `--ip` or `--port` to change settings. |
| [`13_reboot.c`](wuji_glove/13_reboot.c) | Send a device reboot request. | Reboots the connected device. |

### Wuji Hand 2

| Example | Function | Preparation |
| --- | --- | --- |
| [`0_connect.c`](wuji_hand_2/0_connect.c) | Scan, connect, and list parameters and topics. Set connection options when using a serial number. | One Wuji Hand 2. |
| [`1_subscribe_callback.c`](wuji_hand_2/1_subscribe_callback.c) | Receive joint states and IMU data through callbacks. | One Wuji Hand 2. |
| [`2_publish.c`](wuji_hand_2/2_publish.c) | Return all joints to zero and hold through published commands. | Firmware 3.1.0 or later. All joints online. Moves the hand. |
| [`3_hand_info.c`](wuji_hand_2/3_hand_info.c) | Read identity, effort limits, and diagnostics with fault descriptions. | One Wuji Hand 2. |
| [`4_set_stream_rate.c`](wuji_hand_2/4_set_stream_rate.c) | Set the joint-state stream rate and restore its default. | One Wuji Hand 2. |
| [`5_export_flash_logs.c`](wuji_hand_2/5_export_flash_logs.c) | Export device log history to JSONL. | One Wuji Hand 2. |
| [`6_opposition/opposition.c`](wuji_hand_2/6_opposition/opposition.c) | Replay the recorded thumb-opposition trajectory in position mode. | Firmware 3.0.0 or later and bundled replay data. |
| [`7_mit_sweep.c`](wuji_hand_2/7_mit_sweep.c) | Move joint 0 through a cosine sweep in MIT_V2 mode. | Firmware 3.1.0 or later. All joints online. Moves the hand. |
| [`8_position_mode.c`](wuji_hand_2/8_position_mode.c) | Move all joints through a cosine sweep in position mode. | Firmware 3.0.0 or later. All joints online. |
| [`9_recording.c`](wuji_hand_2/9_recording.c) | Record joint states and diagnostics to MCAP, with pause, resume, live status, quality metrics, and alerts. | Supply `--output FILE`. |
| [`10_emergency_stop.c`](wuji_hand_2/10_emergency_stop.c) | Send a whole-hand emergency stop. | One Wuji Hand 2. |
| [`11_clear_fault.c`](wuji_hand_2/11_clear_fault.c) | Disable all joints and explicitly clear faults. | Resolve the fault cause first. |
| [`12_network_config.c`](wuji_hand_2/12_network_config.c) | Read IP and port, and optionally write new settings. | Supply `--ip` or `--port` to change settings. |
| [`13_reboot.c`](wuji_hand_2/13_reboot.c) | Send a device reboot request. | Reboots the connected device. |

> [!WARNING]
> Motion examples enable the hand and start moving it. Keep it clear of people, cables, and nearby objects. Run with direct supervision and be ready to stop the device. Check the source file's prerequisites and expected motion before running.

Copy the complete [`6_opposition/`](wuji_hand_2/6_opposition/) directory, including its replay data. When building the Wuji Hand 2 category, run `6_opposition/6_opposition` inside its build directory. CMake copies `data/` beside the executable. Keep them together when moving the build.

### Wuji Hand

These examples use the first-generation Wuji Hand. Keep the hand clear of people and objects when running `publish` or `grasp_loop`.

| Example | Function | Preparation |
| --- | --- | --- |
| [`0_subscribe.c`](wuji_hand/0_subscribe.c) | Receive joint states. | One Wuji Hand. |
| [`1_publish.c`](wuji_hand/1_publish.c) | Publish zero-position commands. | One Wuji Hand. |
| [`2_grasp_loop.c`](wuji_hand/2_grasp_loop.c) | Open and close the four non-thumb fingers. | One Wuji Hand. |
| [`3_tactile_status.c`](wuji_hand/3_tactile_status.c) | Read tactile glove attachment status and pressure. | Attach the tactile glove before connecting the hand. |

### Retargeting

For teleoperation with Wuji Hand 2, use firmware 3.0.0 or later. The example prompts you to select an SDK user and moves the connected hand. Keep the hand clear of people and objects.

| Example | Function | Preparation |
| --- | --- | --- |
| [`0_retarget_session.c`](retargeting/0_retarget_session.c) | Map synthetic hand keypoints to joint positions. | No device required. |
| [`1_teleop_real.c`](retargeting/1_teleop_real.c) | Drive a Wuji Hand or Wuji Hand 2 from Wuji Glove keypoints. | Hand and glove with matching handedness. |

## Windows Examples

The Wuji Glove and Wuji Hand 2 CMake projects build these targets on Windows with LLVM-MinGW or MSVC.

| Example | Device | Function |
| --- | --- | --- |
| [`wuji_glove/0_connect.c`](wuji_glove/0_connect.c) | Wuji Glove | Connect automatically or by serial number |
| [`wuji_glove/1_subscribe_callback.c`](wuji_glove/1_subscribe_callback.c) | Wuji Glove | Connect and subscribe with callbacks |
| [`wuji_glove/2_recording.c`](wuji_glove/2_recording.c) | Wuji Glove | Record tactile, EMF pose, and hand skeleton streams, with pause and resume |
| [`wuji_hand_2/0_connect.c`](wuji_hand_2/0_connect.c) | Wuji Hand 2 | Scan and list devices, then connect automatically or by serial number |
| [`wuji_hand_2/1_subscribe_callback.c`](wuji_hand_2/1_subscribe_callback.c) | Wuji Hand 2 | Connect and subscribe to joint states |
| [`wuji_hand_2/9_recording.c`](wuji_hand_2/9_recording.c) | Wuji Hand 2 | Record joint states and diagnostics to MCAP, with pause and resume |

The remaining C examples use the Linux or macOS SDK packages.

## Migrating Wuji Hand 2 to MIT_V2

Use `ControlMode.MitV2` / `"mit_v2"` in Python, or
`WUJI_CONTROL_MODE_MIT_V2` in C, for physical joint-torque control.
MIT (`WUJI_CONTROL_MODE_MIT`) retains its previous behavior and is deprecated.

| Quantity | `WUJI_CONTROL_MODE_MIT` | `WUJI_CONTROL_MODE_MIT_V2` |
| --- | --- | --- |
| Position / velocity | rad / rad/s | rad / rad/s |
| Kp / Kd | Firmware-internal units; not convertible. Retune in MIT_V2. | N·m/rad / N·m·s/rad |
| Command effort | Firmware-internal units; not convertible. Retune in MIT_V2. | N·m |
| Effort feedback | A | Joint torque estimate, N·m |
| effort_limit | A | N·m, shared with Position |

MIT_V2 computes `torque = Kp * (q_des - q) + Kd * (dq_des - dq) + feed-forward`,
then clamps the result to `effort_limit`. Its gains reset to 0 on reboot and
must be explicitly set before enabling. Explicitly setting both gains to 0
is allowed for feed-forward-only control. MIT_V2 and Position share the same
N·m effort limit; changing it in either mode also affects the other.

Before sending motion commands:

1. Require all 20 joints online and disable them before switching modes.
2. Select MIT_V2 and write explicit per-joint torque limits and physical Kp/Kd.
3. Read back all 20 torque limits and gain pairs; stop if any is missing or different.
4. Enable, wait until all 20 joints report Enabled, then publish commands.
5. Stop publishing and disable before disconnecting.

The examples provide fixed N·m limits and recommended tuning ranges.
`ControlModeVec` is kept for compatibility; use `wuji_hand_2_set_control_mode` / `wuji_hand_2_get_control_mode` instead.

MIT_V2 requires firmware 3.1.0 or later; with firmware 3.0.0, use MIT.
Position mode's N·m effort limits require firmware 3.0.0 or later.
Torque feedback is an estimate, not an external force-sensor measurement.

The shared Position/MIT_V2 limits written by `2_publish` and
`7_mit_sweep` remain after exit, disable or disconnect, including partial setup
failures.

Recorded joint-state `effort` is in A for MIT and in N·m for MIT_V2 and
Position. Recordings include each joint's control mode; see the recording API
reference before assigning units during replay.

A mode change is rejected while background gain writes are pending. Wait for
their completion before retrying; use synchronous gain writes when completion
must be known before switching modes.

Use `--sn <SN>` with `2_publish` and `7_mit_sweep` to select an exact
device. An unmatched serial number fails without falling back to another hand.

## License

[MIT](../../LICENSE)
