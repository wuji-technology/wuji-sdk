# wuji-sdk (Python)

Python examples for Wuji Glove, Wuji Hand 2, Wuji Hand, and hand retargeting. Install the SDK, run an example, and use its source as a starting point for your application.

See the [SDK documentation](https://docs.wuji.tech/docs/en/wuji-sdk/latest) for API details and the [C examples](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/c/README.md) for the C SDK.

## Installation

```bash
python -m pip install wuji-sdk
```

Wheels are available for CPython 3.8–3.14 on Linux x86_64 and aarch64 (glibc 2.17 or later), macOS 11.0 or later on Apple Silicon, and Windows 10 21H2 x64 or later. See [Windows Examples](#windows-examples) for the Windows example list.

## Quick Start

Download or clone the [repository](https://github.com/wuji-technology/wuji-sdk) to get the example source files. Run these commands from its root directory, which contains `examples/`.

Scan and list devices, then connect one Wuji Glove and print its identity:

```bash
python examples/python/wuji_glove/0_connect.py
```

Automatic connection requires exactly one discovered device. If several devices are present, use the serial number from the scan:

```bash
python examples/python/wuji_glove/0_connect.py --sn DEVICE_SN
```

For Wuji Hand 2, use [`wuji_hand_2/0_connect.py`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/wuji_hand_2/0_connect.py). Both connection examples disconnect before exiting.

## Examples

Each source file starts with its prerequisites, usage, and expected behavior. Ordinary examples are standalone files. Copy the complete directory for examples with bundled data. Corresponding Python and C examples use the same number and name. Python-only async subscription examples appear last.

### Wuji Glove

| Example | Function | Preparation |
| --- | --- | --- |
| [`0_connect.py`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/wuji_glove/0_connect.py) | Scan, connect, and list parameters and topics. Set connection options when using a serial number. | One Wuji Glove. |
| [`1_subscribe_callback.py`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/wuji_glove/1_subscribe_callback.py) | Receive glove streams, including palm IMU and fingertip poses, and global coordinate transforms through callbacks. | One Wuji Glove. |
| [`2_recording.py`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/wuji_glove/2_recording.py) | Record tactile, EMF pose, and hand skeleton streams to MCAP, with pause, resume, live status, quality metrics, and alerts. | Supply `--output FILE`. |
| [`3_offline_pipeline.py`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/wuji_glove/3_offline_pipeline.py) | Compute hand tracking and IMU fusion from synthetic frames. | No device required. |
| [`4_user.py`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/wuji_glove/4_user.py) | Manage SDK users and export, validate, or import their data. | No device required. |
| [`5_calibration.py`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/wuji_glove/5_calibration.py) | Run hand model calibration with terminal guidance or text feedback. | Select a named SDK user. |
| [`6_set_stream_rate.py`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/wuji_glove/6_set_stream_rate.py) | Set the EMF pose stream rate and restore its default. | One Wuji Glove. |
| [`7_tactile_calibration.py`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/wuji_glove/7_tactile_calibration.py) | Collect motions, review recordings, and train a tactile contact model. | SDK build with model training support. |
| [`8_tactile_contact_view.py`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/wuji_glove/8_tactile_contact_view.py) | Display tactile contacts and adjust sensitivity. | Calibrated Wuji Glove. |
| [`9_tactile_residual_view.py`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/wuji_glove/9_tactile_residual_view.py) | Display tactile residuals and adjust the display threshold. | Calibrated Wuji Glove. |
| [`10_set_hand_model_path.py`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/wuji_glove/10_set_hand_model_path.py) | Set and read back a custom hand model path. | Named SDK user and `--hand-model-path FILE`. |
| [`11_time_sync.py`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/wuji_glove/11_time_sync.py) | Synchronize the glove clock and print timing measurements. | One Wuji Glove. |
| [`12_network_config.py`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/wuji_glove/12_network_config.py) | Read IP and port, and optionally write new settings. | Supply `--ip` or `--port` to change settings. |
| [`13_reboot.py`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/wuji_glove/13_reboot.py) | Send a device reboot request. | Reboots the connected device. |
| [`14_subscribe_async.py`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/wuji_glove/14_subscribe_async.py) | Receive glove streams, including palm IMU and fingertip poses, and global coordinate transforms with async/await. | One Wuji Glove. |

### Wuji Hand 2

| Example | Function | Preparation |
| --- | --- | --- |
| [`0_connect.py`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/wuji_hand_2/0_connect.py) | Scan, connect, and list parameters and topics. Set connection options when using a serial number. | One Wuji Hand 2. |
| [`1_subscribe_callback.py`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/wuji_hand_2/1_subscribe_callback.py) | Receive joint states and IMU data through callbacks. | One Wuji Hand 2. |
| [`2_publish.py`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/wuji_hand_2/2_publish.py) | Return all joints to zero and hold through published commands. | Firmware 3.1.0 or later. All joints online. Moves the hand. |
| [`3_hand_info.py`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/wuji_hand_2/3_hand_info.py) | Read identity, effort limits, and diagnostics with fault descriptions. | One Wuji Hand 2. |
| [`4_set_stream_rate.py`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/wuji_hand_2/4_set_stream_rate.py) | Set the joint-state stream rate and restore its default. | One Wuji Hand 2. |
| [`5_export_flash_logs.py`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/wuji_hand_2/5_export_flash_logs.py) | Export device log history to JSONL. | One Wuji Hand 2. |
| [`6_opposition/opposition.py`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/wuji_hand_2/6_opposition/opposition.py) | Replay the recorded thumb-opposition trajectory in position mode. | Firmware 3.0.0 or later and bundled replay data. |
| [`7_mit_sweep.py`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/wuji_hand_2/7_mit_sweep.py) | Move joint 0 through a cosine sweep in MIT_V2 mode. | Firmware 3.1.0 or later. All joints online. Moves the hand. |
| [`8_position_mode.py`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/wuji_hand_2/8_position_mode.py) | Move all joints through a cosine sweep in position mode. | Firmware 3.0.0 or later. All joints online. |
| [`9_recording.py`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/wuji_hand_2/9_recording.py) | Record joint states and diagnostics to MCAP, with pause, resume, live status, quality metrics, and alerts. | Supply `--output FILE`. |
| [`10_emergency_stop.py`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/wuji_hand_2/10_emergency_stop.py) | Send a whole-hand emergency stop. | One Wuji Hand 2. |
| [`11_clear_fault.py`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/wuji_hand_2/11_clear_fault.py) | Disable all joints and explicitly clear faults. | Resolve the fault cause first. |
| [`12_network_config.py`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/wuji_hand_2/12_network_config.py) | Read IP and port, and optionally write new settings. | Supply `--ip` or `--port` to change settings. |
| [`13_reboot.py`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/wuji_hand_2/13_reboot.py) | Send a device reboot request. | Reboots the connected device. |
| [`14_subscribe_async.py`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/wuji_hand_2/14_subscribe_async.py) | Receive joint states and IMU data with async/await. | One Wuji Hand 2. |

> [!WARNING]
> Motion examples enable the hand and start moving it. Keep it clear of people, cables, and nearby objects. Run with direct supervision and be ready to stop the device. Check the source file's prerequisites and expected motion before running.

Copy the complete [`6_opposition/`](https://github.com/wuji-technology/wuji-sdk/tree/main/examples/python/wuji_hand_2/6_opposition/) directory, including its replay data. Keep `data/` beside `opposition.py`.

### Wuji Hand

These examples use the first-generation Wuji Hand. Keep the hand clear of people and objects when running `publish` or `grasp_loop`.

| Example | Function | Preparation |
| --- | --- | --- |
| [`0_subscribe.py`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/wuji_hand/0_subscribe.py) | Receive joint states. | One Wuji Hand. |
| [`1_publish.py`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/wuji_hand/1_publish.py) | Publish zero-position commands. | One Wuji Hand. |
| [`2_grasp_loop.py`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/wuji_hand/2_grasp_loop.py) | Open and close the four non-thumb fingers. | One Wuji Hand. |
| [`3_tactile_status.py`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/wuji_hand/3_tactile_status.py) | Read tactile glove attachment status and pressure. | Attach the tactile glove before connecting the hand. |

### Retargeting

Install NumPy before running these examples:

```bash
python -m pip install numpy
```

Retargeting is available on Linux x86_64 and aarch64.

For teleoperation with Wuji Hand 2, use firmware 3.0.0 or later. The example prompts you to select an SDK user and moves the connected hand. Keep the hand clear of people and objects.

| Example | Function | Preparation |
| --- | --- | --- |
| [`0_retarget_session.py`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/retargeting/0_retarget_session.py) | Map synthetic hand keypoints to joint positions. | No device required. |
| [`1_teleop_real.py`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/retargeting/1_teleop_real.py) | Drive a Wuji Hand or Wuji Hand 2 from Wuji Glove keypoints. | Hand and glove with matching handedness. |

## Windows Examples

These examples are supported on Windows.

| Example | Device | Function |
| --- | --- | --- |
| [`wuji_glove/0_connect.py`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/wuji_glove/0_connect.py) | Wuji Glove | Connect automatically or by serial number |
| [`wuji_glove/1_subscribe_callback.py`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/wuji_glove/1_subscribe_callback.py) | Wuji Glove | Connect and subscribe with callbacks |
| [`wuji_glove/2_recording.py`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/wuji_glove/2_recording.py) | Wuji Glove | Record tactile, EMF pose, and hand skeleton streams to MCAP |
| [`wuji_glove/14_subscribe_async.py`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/wuji_glove/14_subscribe_async.py) | Wuji Glove | Connect and subscribe asynchronously |
| [`wuji_hand_2/0_connect.py`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/wuji_hand_2/0_connect.py) | Wuji Hand 2 | Scan and list devices, then connect automatically or by serial number |
| [`wuji_hand_2/1_subscribe_callback.py`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/wuji_hand_2/1_subscribe_callback.py) | Wuji Hand 2 | Connect and subscribe with callbacks |
| [`wuji_hand_2/9_recording.py`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/wuji_hand_2/9_recording.py) | Wuji Hand 2 | Record joint states and diagnostics to MCAP, with pause and resume |
| [`wuji_hand_2/14_subscribe_async.py`](https://github.com/wuji-technology/wuji-sdk/blob/main/examples/python/wuji_hand_2/14_subscribe_async.py) | Wuji Hand 2 | Connect and subscribe asynchronously |

For other platforms, see the prerequisites above and in each source file.

## Migrating Wuji Hand 2 to MIT_V2

Use `ControlMode.MitV2` / `"mit_v2"` in Python, or
`WUJI_CONTROL_MODE_MIT_V2` in C, for physical joint-torque control.
MIT (`ControlMode.Mit` / `"mit"`) retains its previous behavior and is deprecated.

| Quantity | `ControlMode.Mit` | `ControlMode.MitV2` |
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
`ControlModeVec` is kept for compatibility; use `control_mode()` instead.

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

Use `--sn <SN>` with `2_publish` and `7_mit_sweep` to select an exact device. An unmatched
serial number fails without falling back to another hand.

## License

[MIT](https://github.com/wuji-technology/wuji-sdk/blob/main/LICENSE)
