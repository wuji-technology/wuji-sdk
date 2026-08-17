# Changelog

All notable changes to wuji-sdk will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project uses calendar versioning (YYYY.M.D).

## [Unreleased]

## [2026.8.17]

### Added

- Added `Subscription.set_rate(frequency_hz)` to lower a subscribed stream's device push rate at runtime, in samples per second. Upgrade the device to firmware that supports stream rate control before using this method (Wuji Glove v0.11.4 or later, Wuji Hand 2 v2.3.0 or later). Pass 0 to restore the device default. The call returns the rate the device applies, which may be quantized. Supported on `emf_poses`, `tactile`, `tactile_zones`, and `imu_raw/*` (Wuji Glove) and on `joint_states`, `joint_diagnostics`, `imu`, and `fingertip/*/data` (Wuji Hand 2). Derived streams—including `hand_joint_angles`, `hand_skeleton`, `tip_poses`, and `imu_data/*`—follow their source rate and reject `set_rate`. The setting affects every subscriber of the stream and resets to the device default when the last subscription to the stream is closed or the device reconnects. Examples: `examples/python/wuji_glove/6.set_stream_rate.py` and `examples/c/wuji_glove/1_set_stream_rate.c` for the Wuji Glove, and `examples/python/wuji_hand_2/4.set_stream_rate.py` and `examples/c/wuji_hand_2/4_set_stream_rate.c` for the Wuji Hand 2.
- **C SDK**: Added `wuji_sub_set_rate(sub, frequency_hz, out_actual_hz)` on any subscription handle, matching the Python `Subscription.set_rate` semantics. Streams that do not support rate control return `WUJI_STATUS_ERR_UNSUPPORTED (-8)` without sending a rate-control request to the device.

### Changed

- **Wuji Hand 2**: Printing a `JointDiagnosticsEntry` now shows `error_code_current` in hexadecimal (`0x2103`) instead of decimal (`8451`), matching how error codes are written in the documentation. `error_code().get()` still returns an integer — format it as `f"0x{code:04X}"` if you print it directly.
- **Wuji Hand 2**: On device firmware v2.4.0 and later, fingertip per-point force arrives normalized to the sensor's full scale — normal axis 0 to 1, tangential axes -1 to 1, clamped beyond that. Firmware before v2.4.0 keeps reporting each point in newtons. The aggregate force and the temperature reading are in newtons and Celsius on both. Read `scale` and `unit` from `info.format` rather than assuming either convention, and re-fetch the info after a firmware upgrade so your decoder picks up the new scale.
- `device.get()` / `manager.get()` now raise `WujiException` for a nonexistent parameter path. Previously they silently returned `None`.
- **Retargeting**: The live teleoperation examples (`examples/python/retargeting/1.teleop_real.py` and `examples/c/retargeting/1_teleop_real.c`) now list the SDK users at startup and let you pick which one to teleoperate under, instead of always switching to the default user. Press Enter to keep the user you are already on, pick the default user to run on the built-in hand URDF, or pick a named user to teleoperate on that user's calibrated hand model — a named user that has not been calibrated yet also runs on the built-in URDF. The previously selected user is still restored on exit.

### Removed

- **Wuji Glove — BREAKING**: Removed the EMF pose rate divider—Python `glove.emf_poses_rate_divider()`, C `wuji_glove_get_emf_poses_rate_divider` / `wuji_glove_set_emf_poses_rate_divider`, and the matching examples. Upgrade the device firmware to v0.11.4 or later, then use `set_rate` on an `emf_poses` subscription. The setting resets when the last subscription closes or the device reconnects. Lowering `emf_poses` also lowers the EMF input rate used for hand tracking.

### Fixed

- **C SDK**: Calling an operation that a resource does not permit, such as reading a write-only resource or subscribing to a publish-only topic, now returns `WUJI_STATUS_ERR_UNSUPPORTED` instead of `WUJI_STATUS_ERR_INTERNAL`.
- **Python and C SDKs**: After a device disconnects or is taken over by another host, Python `recv()` and `recv_async()` now raise `WujiException` immediately, and C subscription callbacks terminate without delivering buffered frames. Previously, Python could return buffered frames or `None`, and C callbacks could deliver buffered frames before terminating.
- **Wuji Hand**: Direct hand methods (`read_joint_state()`, `enable()`/`disable()`, `clear_all_faults()`, effort-limit and diagnostics accessors, opening a realtime controller) now fail with a disconnect error after `disconnect()` instead of continuing to operate the device. Python raises `WujiException`. C returns `WUJI_STATUS_ERR_DISCONNECTED`.
- **Wuji Hand**: Disconnecting now releases the USB device even when a joint-state subscription is still open. The same process can reconnect immediately without first closing the subscription or deleting the hand object.

## [2026.8.3]

### Added

- **Wuji Hand 2**: The `joint_diagnostics` stream now carries built-in communication health — each joint reports its bus response rate and timeout count, and every frame includes a summary with end-to-end (Ethernet) stream loss, request retry/timeout counters, and the fingertip tactile online bitmap. The SDK maintains all of it automatically in the background, so no polling is required.
- **Wuji Glove**: Added guided tactile calibration — record prompted hand motions to train a per-glove contact model for real-time contact detection (`tactile_binary`). Python: `glove.calibrate_tactile()` / `glove.calibrate_tactile_blocking()`. C: blocking guided flow with pose-prompt callbacks.

  Models load automatically. Training requires a `model-export` build — see the motion guide at https://docs.wuji.tech/docs/en/wuji-studio/latest/tactile-calibration/.
- **C SDK**: Added `wuji_glove_set_tactile_binary_sensitivity` for standalone `tactile_binary` observation tools. It changes the SDK-local contact sensitivity for the connected glove and rejects non-finite or non-positive values.
- **C SDK**: Added the runnable user-management example `examples/c/wuji_glove/3_user_management.c` for inspecting the current SDK user, creating or updating named users, and switching users before calibration.
- **Wuji Glove**: Added `tactile_residual` observation examples — the continuous signed signal behind `tactile_binary`, where you set your own contact threshold instead of using a built-in one. Python: `glove.tactile_residual()`, C: `wuji_glove_subscribe_tactile_residual` typed callback. Calibrate the glove first.

### Changed

- **Wuji Glove**: The C IK calibration example is now `examples/c/wuji_glove/4_user_calibration.c` (renamed from `3_user_calibration.c`) and calibrates the SDK user you are already on instead of creating or switching one — use the new `3_user_management.c` example first if you need to create or switch users. It also gained a `--blocking` mode, which runs calibration to completion without the cooperative Ctrl+C cancellation the default asynchronous path offers.
- **Wuji Hand 2 — BREAKING (C)**: The joint diagnostics structs gained new fields, so their C ABI changed. Applications that use `WujiJointDiagnosticsEntry` or `WujiJointDiagnosticsFrame` must be recompiled against the new header before linking the new library — linking new against old will read and write past the end of the old structs. Existing fields keep their meaning and order, and no source changes are needed; the added fields carry the per-joint bus communication quality and the frame-level communication summary described above.
- **Wuji Hand**: `RealtimeController` can now be shared across Python threads — for example a writer thread streaming targets while a reader thread polls actual positions — without any locking. Previously, touching the controller from a second thread raised an error.
- **C SDK**: The Linux x86_64 and aarch64 (gnu) tarballs now ship one library file, `libwuji_sdk_c.so`. Wuji Hand support no longer needs the `libwujihandcpp.so` that earlier tarballs bundled beside it, and `libstdc++.so.6` and `libusb-1.0.so.0` are no longer runtime dependencies. The C API and link line are unchanged, so existing applications keep working without a rebuild. If you deployed `libwujihandcpp.so` from an earlier tarball, you can delete it — the SDK doesn't load it anymore.

### Removed

- **Wuji Hand — BREAKING (Python)**: Removed the one-shot `joint_target_position()` resource (`hand.joint_target_position().set(...)`). Send targets through the joint-command publisher instead: `hand.joint_command().publish().send([JointCommand(position, 0.0, 0.0), ...])` with exactly 20 entries — behavior is unchanged.

### Fixed

- **Wuji Hand 2**: The SDK no longer binds a fixed local network port when connecting a hand. The local port is now assigned automatically by the operating system, so the SDK will not conflict with ports used by your own applications.
- Device scanning no longer hangs indefinitely in the rare case where a network peer disappears during discovery. The network discovery step is now time-bounded, so scanning always returns within a few seconds with the devices found so far. Covers both the Python SDK and the C SDK.
- Network device discovery now works on Linux hosts with multiple network interfaces or IP addresses when the host's primary IP address is not on the device's subnet — previously scanning on such hosts could permanently find no devices. Covers both the Python SDK and the C SDK.
- **C SDK**: Wuji Hand tactile-glove calls no longer crash the process when the glove is missing or faulty — they return an error status instead (details via `wuji_last_error()`).
- **Wuji Hand**: Connecting to a hand that isn't ready no longer crashes your application — it now fails with a catchable error that names any joint that didn't respond. Covers both the Python SDK and the C SDK.
- **Wuji Hand 2**: Setting `effort_limit` to a value the device rejects (e.g. above the current ceiling) now raises a `ValueError` instead of a generic error; the device keeps its previous limit.
- **Wuji Hand**: Physically unplugging the tactile glove at runtime is now detected. `is_tactile_attached()` returns `false`, the tactile status stream reports `NotPresent` (state 0) typically within 200 ms, and other tactile reads return a clear "tactile not present" error — previously they all kept reporting a healthy, attached glove indefinitely and only the frame age hinted at the loss. The hand itself is unaffected and keeps working; re-attaching the glove still requires reconnecting the hand. Applies to both the Python SDK and the C SDK (`wuji_hand_is_tactile_attached`).
- **Wuji Glove**: EMF finger poses now use the selected hand URDF to reject unreachable mirrored positions, reducing incorrect position reversals caused by transient interference.

## [2026.7.21]

### Added

- **Wuji Hand 2**: First official release of fingertip tactile sensing — subscribe to self-describing per-finger sensor streams (`FingertipSensorInfo` metadata describes how to decode each `FingertipSensorData` frame — see the `3.fingertip_typed.py` example), recalibrate all five fingertips to a fresh zero baseline with `hand.tactile_calibrate()` (Python) / `wuji_hand_2_tactile_calibrate(dev)` (C) while keeping the sensor surfaces unloaded, and query each sensor's model and calibration state with `hand.tactile_status(finger)` to confirm a recalibration has finished.
- **C SDK**: Added the hand retargeting API (`wuji_retarget_session_create` / `_step` / `_reset` / `_free`) — maps 21 hand keypoints (MediaPipe order) to 20 joint angles in firmware order, matching the Python `RetargetSession`. Degenerate or invalid keypoint frames return the new `WUJI_STATUS_ERR_ALGORITHM` status (details via `wuji_last_error()`).

### Changed

- Hand retargeting now runs on a native implementation: the `[retarget]` extra (`scipy`, `nlopt`, `pin`, `pyyaml`) is no longer required — `RetargetSession` works out of the box with lower per-step latency. `numpy` is still used for keypoint/qpos arrays.
- Retargeting output has been refined — notably improved thumb articulation. Joint angles for identical inputs may differ from previous releases.
- **BREAKING (Python)**: `RetargetSession` and `HandModel` now live at the package top level, and the `wuji_sdk.retargeting` submodule is removed — `from wuji_sdk.retargeting import RetargetSession, HandModel` no longer imports. Update imports to `from wuji_sdk import RetargetSession, HandModel`.

### Fixed

- **Wuji Hand 2**: Firmware upgrade failures now report the detailed error (specific reason, error code, and any failed nodes) instead of a generic "Unknown error".
- **Wuji Hand**: Connecting to a hand that's on the USB bus but not responding now returns a clear initialization error with the underlying cause, instead of misreporting the device as not found. This covers both the Python SDK (connect by serial number) and the C SDK (`wuji_hand_connect_sn`).

## [2026.7.14]

### Added

- **Python SDK**: Added export, preview, and import of your Wuji user data as a portable `.zip` bundle. `SdkManager.export_user_data(user_id, out_path)` writes everything a user owns — calibration hand models plus the tactile model and its latest complete calibration run — into one file. `preview_user_data(zip_path)` validates a bundle and reports its contents without importing it. `import_user_data(zip_path)` restores a bundle to the user who created it (creating that user if it doesn't exist) and overwrites that user's existing data, without changing the current user.
- **C SDK**: Added user data import/export, mirroring the Python API. `wuji_user_data_export` / `wuji_user_data_preview` / `wuji_user_data_import` move a user's data as a portable `.zip` bundle and return a `WujiStatus` — call `wuji_last_error()` for the message on failure. They report the specific failure: `WUJI_STATUS_ERR_NOT_FOUND` when the bundle file doesn't exist, and `WUJI_STATUS_ERR_INVALID_DATA` when the bundle is present but fails validation (corrupt zip, bad manifest, checksum mismatch, or unsafe path).
- **Python SDK**: `SdkManager.scan()` now reports each discovered device's type. `DiscoveredDevice.device_type` returns a `DeviceType` enum (`WujiGlove`, `WujiHand2`, `WujiHand`, or `Unknown`), so you can tell what a device is before connecting. `Unknown` means the type was not available at scan time.
- **C SDK**: Added device type at scan time. `WujiDiscovered.device_id` is now a `WujiDeviceType` (`WUJI_DEVICE_TYPE_WUJI_GLOVE` / `_WUJI_HAND_2` / `_WUJI_HAND` / `_UNKNOWN`) and `WujiDiscovered.model` carries the readable type string (e.g. `"WujiGlove"`), so you can tell what a device is before connecting.
- **C SDK**: Added SDK user management and Wuji Glove IK calibration APIs matching the Python SDK behavior. C applications can create and switch SDK users, run calibration with structured feedback, cancel asynchronous calibration sessions, and receive the per-user calibrated URDF path. The calibration example provides the same in-place terminal dashboard and throttled API log mode as the Python example.
- **Wuji Hand**: The real-time controller now exposes `get_actual_effort()` (per-joint effort in amps, read from the same non-blocking upstream cache as `get_actual_position()`) in the Python SDK, and the C SDK exposes the matching `wuji_hand_realtime_controller_get_actual_effort`.
- **Wuji Hand**: Added Wuji Hand (first-generation dexterous hand) support to the C SDK. Connect by USB serial number (`wuji_hand_connect_sn`), then read joint positions, per-joint diagnostics (bus voltage / temperature / error code), firmware soft limits, handedness, and whether a tactile glove is attached. Control the hand with `enable` / `disable` / `clear_all_faults`, and set or read the effort limit. Stream commands through a joint-command publisher — a list of 20 `JointCommand` structs, one `position` / `velocity` / `effort` per joint, the same shape as Wuji Hand 2, so command code is portable between the two. Run a low-pass-filtered real-time position controller (`set_target_position` / `get_actual_position` / `get_actual_effort`). Subscription streams for joint state and tactile pressure are also available. Supported on **Linux x86_64 / aarch64 (gnu) only** — those tarballs ship `libwujihandcpp.so` alongside `libwuji_sdk_c.so`. Keep the two files in the same directory and it is found automatically (no `LD_LIBRARY_PATH` needed). `libstdc++.so.6` and `libusb-1.0.so.0` are the only runtime system dependencies. On the Android tarball these functions are present but return `WUJI_STATUS_ERR_UNSUPPORTED`. Worked examples are under `examples/c/wuji_hand/`.

### Changed

- **Wuji Hand — BREAKING (Python)**: The joint-command publisher now takes an array of structs. `JointCommandPublisher.send(...)` changed from three parallel lists `send(positions, velocities, efforts)` to a single list of per-joint commands `send(joints)`, where each entry is a `JointCommand(position, velocity, effort)` — pass exactly 20. The old `publish(...)` alias was removed. This matches Wuji Hand 2 exactly, so joint-command code is now portable between the two hands. Update any `send(positions, velocities, efforts)` / `publish(...)` call to `send([JointCommand(p, v, e), ...])`.
- **Wuji Hand — BREAKING (Python)**: The joint-command publisher is now opened via `hand.joint_command().publish()`. The resource accessor was renamed from `.publisher()` to `.publish()`, and the shortcut method `hand.joint_command_publisher()` was removed — both to match Wuji Hand 2 exactly. The returned `JointCommandPublisher` (with `send(...)` / `close()`) is unchanged. Update `hand.joint_command().publisher()` or `hand.joint_command_publisher()` to `hand.joint_command().publish()`.
- **Wuji Hand — BREAKING (Python)**: Tactile-glove classes now carry a `TactileGlove` prefix. The paired tactile glove's classes were renamed so they cannot be confused with the Wuji Glove's generic tactile streams: `TactileStatus` → `TactileGloveStatus`, `TactileDiagnostics` → `TactileGloveDiagnostics`, `TactileDeviceInfo` → `TactileGloveDeviceInfo`, and the sub-module handle `WujiTactile` → `WujiTactileGlove`. Accessors (`hand.tactile()`, `hand.tactile_status()`, …) and wire formats are unchanged — update type references and `isinstance` checks to the new class names.
- **Wuji Hand — BREAKING (Python)**: The joint-state stream is now `hand.joint_states()`. The subscription accessor was renamed from `hand.joint_state()` to `hand.joint_states()`, and the frame class from `HandJointState` to `HandJointStates` — matching Wuji Hand 2 and the ROS `/joint_states` convention. Update `hand.joint_state().subscribe()` to `hand.joint_states().subscribe()` and any `HandJointState` type references to `HandJointStates`.
- **Wuji Glove — BREAKING**: Hand calibration now belongs to the SDK user and hand side instead of an individual glove. Swapping to another glove of the same side no longer requires recalibration. Calibration recorded by older SDK versions is not loaded anymore — recalibrate under your user profile.
- **Wuji Glove — BREAKING**: The `hand_profile` calibration option is deprecated and ignored (passing it emits a `DeprecationWarning`). The calibration result was simplified to `handedness` plus a single `calibrated_urdf` path. The `calibration.hand_profile` and `calibration.hand_model_paths.*` parameters were removed: setting them now fails, and values left by older versions are ignored.
- **Wuji Glove — BREAKING**: The tactile contact model is now located automatically from the current SDK user and device serial number — the same convention as the calibrated hand model — and stored under the SDK's own tactile directory. The `algorithms.tactile_binary.model_dir` parameter was removed: setting it now fails, and values left by older versions are ignored. Integrations that set this parameter to point tactile contact detection at a model directory should stop setting it. The model is resolved automatically once you select the SDK user.
- **Wuji Glove**: Calibration bundles use a new format with user-level hand models and per-glove tactile data. Importing bundles from older SDK versions keeps the tactile data but skips the old hand calibration (it is no longer loaded — recalibrate under your user profile). Bundles exported by this version require an up-to-date SDK to import.
- **C SDK — BREAKING**: `WujiConnectOptions` changed layout to match Python `ConnectOptions` for bridge and background time-sync settings. Rebuild C applications against the updated `wuji_sdk.h`. Call `wuji_connect_options_default()` before overriding fields when passing non-NULL options to `wuji_connect`, or pass `opts_or_null = NULL` to keep default connection behavior.
- **Retargeting**: Optimized wheel packaging — download and installed size are significantly smaller. No functional change.
- **Wuji Hand 2**: `disconnect()` now fully releases the hand — after disconnecting, the hand is immediately available to any other application or machine, exactly as if your application had exited. Previously the hand could remain held by the application until it exited.

### Removed

- **C SDK — BREAKING**: Removed unused API `wuji_glove_subscribe_tactile_zones_cache()`.

### Fixed

- **Wuji Glove**: Setting a custom hand URDF under the default SDK user (semantic setters and generic SDK-param write paths) now fails with a clear error instead of silently having no effect — switch to a named SDK user first. The semantic setter also rejects unreadable paths immediately.
- Fixed an issue where connecting multiple devices in the same process (for example a Wuji Glove and a Wuji Hand 2) could fail depending on the connection order.
- **Wuji Hand 2**: Fixed a bug where applications that repeatedly connect and disconnect could see each new connection take longer than the last. Connections now stay consistently fast, no matter how long the application has been running.
- **Python SDK**: Fixed a crash on exit. If a device disconnected while a callback subscription was still running and the process then exited, the app could crash with a fatal Python error or segmentation fault. Background subscription threads now stop cleanly before the interpreter shuts down.

## [2026.7.2]

### Fixed

- **Retargeting**: Fixed the live teleoperation example (`examples/python/retargeting/1.teleop_real.py`) — it now connects with `ConnectOptions(enable_bridge=False)` so the glove → retarget → hand loop runs correctly.
- **Wuji Hand**: Fixed reconnecting within the same process — after `disconnect()`, a new `connect()` to the same hand previously failed until the process exited. After `disconnect()` the handle is no longer usable; create a new connection to reconnect.

## [2026.7.1]

### Added

- **Retargeting**: Added hand retargeting — map human hand keypoints (21 MediaPipe-format landmarks) to Wuji Hand joint angles. Build a session with `wuji_sdk.retargeting.RetargetSession.for_hand(HandModel.WujiHand2, side=Handedness.Right)`, then call `session.step(keypoints)` to get a 20-value joint command ready to send to the device. A live teleoperation example (Wuji Glove keypoints → retarget → Wuji Hand / Wuji Hand 2) is included under `examples/python/retargeting/1.teleop_real.py`. Install the runtime dependencies with `pip install wuji-sdk[retarget]`. Linux x86_64 / aarch64 only.
- **C SDK**: Added Wuji Hand 2 (dexterous hand) support, mirroring the redesigned resource-style Python interface — `joint_states` / `joint_diagnostics` subscription streams, `mit_params` and `effort_limit` configuration, control actions (enable / disable / clear faults / emergency stop / user origin), and a joint-command publisher. Whole-hand reads cover all 20 joints with a per-joint online bitmap so offline joints are distinguishable. String getters use a two-call size query: call once with a `NULL` buffer to get the required length, then again to fill it.
- **C SDK**: Added `wuji_glove_get_emf_poses_rate_divider` / `wuji_glove_set_emf_poses_rate_divider` to lower the WujiGlove EMF pose output rate (divider N ≥ 1; output rate = input rate / N, e.g. N=4 → ~30 Hz from the 120 Hz source), matching the Python `glove.emf_poses_rate_divider()`. Every stream derived from EMF poses (`hand_joint_angles`, `tip_poses`, `hand_skeleton`, `tactile_point_cloud`) drops to the same rate automatically, reducing inverse-kinematics CPU. Applies at runtime and persists across reconnects.
- **C SDK**: Added `wuji_glove_sync_time` (filling a `WujiTimeSyncResult` with the clock offset, round-trip time, and measurement timestamp in microseconds) to trigger a single time-sync round-trip with a Wuji Glove, matching Python `glove.sync_time()`.
- **Wuji Glove / C SDK**: Added Python `glove.hand_model_path().get()/set(path)` and matching C `wuji_glove_get_hand_model_path` / `wuji_glove_set_hand_model_path` to select a custom WujiGlove hand URDF for online IK. Existing Python subscription examples now expose `--hand-model-path`, and the new C example sets and reads back the URDF before subscribing to `hand_joint_angles` to verify the live IK output.

### Changed

- **Wuji Glove**: Updated `tactile_point_cloud` to the 526-taxel active point-cloud contract. The SDK taxel mapping now matches the firmware's 24×31 tactile output and removes the eight additional hardware-masked taxels beyond the dropped dead column. Use this SDK with the matching firmware so point-cloud frames report 526 points consistently.
- **Wuji Glove / SDK users**: Changed the default SDK user to always use the built-in default hand URDF for IK-derived streams (`hand_skeleton`, `hand_joint_angles`, `tip_poses`) and ignore stored calibration URDF paths. Running `calibrate` as the default user now fails fast with error code `0x4108`; create or switch to a named SDK user before calibration so generated URDFs are scoped to that user.
- **Wuji Hand 2 — BREAKING**: Redesigned the interface into a unified resource-style API. Scripts written against the previous version must be updated, and the device firmware and SDK must be upgraded together. See the migration guide for details. The new model:
  - **One access model.** Every feature is a resource accessed via `.get()` / `.set(...)` / `.subscribe()` / `.publish()`, plus action methods for control. Whole-hand and single-joint use the same methods: `hand.<resource>()` and `hand.joints()[k].<resource>()`.
  - **Polymorphic `set`.** Pass a single value to apply it to all joints, or a 20-element array to set per joint.
  - **Control actions.** `enable()`, `disable()`, `clear_fault()`, and user-zero `set_origin()` / `clear_origin()` — each accepts an optional length-20 0/1 mask to act on a subset of joints. `emergency_stop()` always acts on the whole hand.
  - **Feedback as subscription streams.** `hand.joint_states()` (position / velocity / effort) and `hand.joint_diagnostics()` (status word / current / bus voltage / temperature / error code). Frames are variable-length and contain only online joints — identify each entry by its node id.
  - **Real-time commands via a publisher.** `hand.joint_command().publish().send(joints)` — a list of 20 `JointCommand`, each carrying `position` / `velocity` / `effort`.
  - **Configuration resources** `mit_params` (`(kp, kd)`) and `effort_limit` (amperes). Writes reject NaN / infinity / negative values; reads return 20 entries (offline joints are `None`).
  - **Control mode** is no longer set from the Python API; the device operates in MIT control by default.
  - **Typed reads auto-decode** — no manual byte parsing.
  - **Error lookup.** `describe_error(code)` turns a status / error code into a human-readable description.
- **Wuji Hand 2 — BREAKING**: Changed the joint-command publisher to take an array of structs. `hand.joint_command().publish().send(...)` changed from three parallel lists `send(positions, velocities, efforts)` to a single list of per-joint commands `send(joints)`, where each entry is a `JointCommand(position, velocity, effort)`. Pass exactly 20 `JointCommand`. Update any `send(positions, velocities, efforts)` call accordingly.
- **Wuji Hand 2 — `joint_states` / `joint_diagnostics` feedback frames now carry a `FrameHeader`** (`seq` + `timestamp_us` + `frame_id`), consistent with the IMU feedback frames. `timestamp_us` is the firmware send time and `frame_id` is `l_wrist` / `r_wrist` (filled by the firmware from its own handedness) for 3D hand-pose visualization. The previous top-level `seq` field moved into `header.seq`. Requires the matching firmware build.
- **Wuji Glove**: Local params files are now saved in a cleaner canonical form. The SDK no longer persists generated defaults, empty values, deprecated fields, or runtime-only cache data; existing legacy hand profile fields are migrated to the current `wujihand` / `wujihand2` names, and params files with no persistent settings are removed on save. This only changes the on-disk config file shape; runtime hand-tracking behavior remains unchanged.
- **Wuji Glove**: Updated the built-in default hand model — the fallback hand geometry used for hand-tracking output (`hand_skeleton`, `hand_joint_angles`, `tip_poses`) when no per-device hand profile is loaded. Finger-segment lengths and joint origins are refreshed to the latest calibration for more accurate default hand tracking; the joint/link layout is unchanged. Only affects sessions relying on the default (uncalibrated) hand model.

## [2026.6.18]

### Added

- **Wuji Hand 2**: Added a `ControlMode` enum (`ControlMode.Off`, `.Mit`, `.Position`, `.Velocity`, `.Current`) for `hand.set_all_control_mode(...)` and `hand.control_mode().set(...)`. The previous lowercase string form (`"off"`, `"mit"`, `"position"`, `"velocity"`, `"current"`) still works.
- **Wuji Hand 2**: Added a query for the connected hand's side, reporting `"left"` or `"right"`. Raises if the device reports an unrecognized value.

### Changed

- **Wuji Hand 2**: Replaced `hand.clear_fault().set(1)` with a direct action `hand.clear_fault()` (no argument) that clears all joint faults, matching `hand.reboot()`. Update any `hand.clear_fault().set(1)` call to `hand.clear_fault()`.
- **Wuji Hand 2**: Changed the whole-hand read methods (`diagnostics`, `calib_result`, `mit_params`, `speed_pi`, `current_pi`, `aux_function`, `get_diagnostic_status`, `online_joints_count`) to raise on disconnect instead of silently returning empty data. A reachable hand with offline joints still returns `None` entries, so disconnect and offline are now distinguishable. Wrap reads in `try/except` if the hand may be disconnected.
- **Wuji Glove**: Updated tactile data layout to 744 values arranged 24×31 (was 768, 24×32). A dead sensor column is dropped at the device, so `tactile`, `tactile_residual`, and `tactile_binary` frames — and the per-zone tactile layout — are correspondingly smaller. Invalid taxels are still reported as `-1.0`.

## [2026.6.16]

### Added

- **Wuji Glove**: Added `glove.emf_poses_rate_divider()` to lower the EMF pose output rate. Set an integer divider N (default 1) with `glove.emf_poses_rate_divider().set(N)` to publish poses at input_rate / N (e.g. N=4 → ~30 Hz from the 120 Hz source); read it back with `.get()`. Every stream derived from EMF poses drops to the same rate automatically — `emf_poses`, `hand_joint_angles`, `tip_poses`, `hand_skeleton`, and `tactile_point_cloud` — which reduces the CPU spent on inverse-kinematics solving. IMU-driven streams (`imu_*`, `tf`) and the raw tactile streams (`tactile`, `tactile_zones`, `tactile_residual`, `tactile_binary`) keep their original rate. The setting applies at runtime and persists across reconnects.

### Fixed

- Fixed known issues for the public SDK release.

## [2026.6.15]

### Added

- **Wuji Glove / Python SDK**: Added local SDK user profiles — each profile keeps its own calibration data per device, and switching profiles reloads calibration on connected devices immediately.
- **Wuji Glove / Python SDK**: Added Python-side IK calibration with per-pose progress callbacks and an option to generate URDFs for one or both hand profiles from a single capture, selectable later for live hand tracking.
- **Wuji Glove**: Added `glove.tactile_residual()` — publishes a continuous baseline-removed contact signal at the same rate as `tactile`, with the same frame shape so existing tactile visualizers work unchanged. An untouched hand rests near 0. Apply your own thresholding instead of the built-in `tactile_binary` output.
- **C SDK**: Added a prebuilt C SDK with device discovery, typed per-stream subscription callbacks (tactile, EMF poses, hand joint angles, hand skeleton), and global cross-device coordinate transforms (`tf`, `tf_static`). Distributed as a tarball attached to each Release for `x86_64-linux-gnu`, `aarch64-linux-gnu`, and `aarch64-android`.
- **Wuji Hand**: Added initial public version — USB auto-discovery and connection, motor enable/disable, per-joint effort limits, real-time joint state subscription (20 joints, position), joint command publishing with real-time command smoothing, and an optional plug-in tactile glove stream.

### Changed

- **Wuji Glove**: Parallelized hand pose solving across CPU cores, reducing the compute time behind `hand_joint_angles` and the hand-tracking data derived from it. Output values and schemas are unchanged.
- **Wuji Hand**: Mirrored the Wuji Hand 2 SDK API surface (scan, connect, joint state, joint command, realtime controller), so existing Wuji Hand 2 example code transfers to Wuji Hand with minimal changes.

## [2026.6.2] - 2026-06-02

### Added

- **Wuji Glove**: Added `glove.tactile_binary()` — publishes a 768-element binary contact frame (`1.0` contact, `0.0` no contact, `-1.0` invalid) at the same rate as `tactile`, sharing `TactileFrame`'s shape so existing tactile-grid visualizers work unchanged.
- `SdkManager.connect(handedness=Handedness.Left | Handedness.Right, device_name=...)` — connect to a bimanual device by side.

## [2026.5.26]

### Added

- **Wuji Glove**: `glove.sync_time()` — manually trigger a time sync and inspect the result (`offset_us`, `round_trip_us`, `synced_at_us`). The SDK also runs a 30 s background time sync after connect, configurable via `ConnectOptions.auto_time_sync_interval_ms` (pass `None` to disable, or any value `>= 100` ms to override the default). Requires firmware > 0.10.1.
- **Wuji Hand 2**: Initial public release — auto-discovery and connection, motor enable/disable, MIT impedance control parameters, real-time joint state subscription (20 joints, position/velocity/torque), joint fault clearing, and per-joint diagnostics (bus voltage, temperature, fault codes)

## [0.10.0] - 2026-05-15

### Added

- Added `device.export_flash_logs(out_dir)` to dump historical device logs to JSONL files (`<out_dir>/<sn>_<unix_ts>.jsonl`), safe against log buffer wrap during export
- Added per-channel recording health via `QualityMetrics.channels: List[ChannelHealth]` — frame rate, drop rate, jitter, `is_online`, and `last_downtime_ms` for each recorded topic

### Changed

- **Wuji Glove**: Updated `hand_joint_angles` and `tip_poses` to derive from the URDF kinematic chain; numerical values may differ from previous SDK versions, output schemas unchanged

### Removed

- **Wuji Glove**: Removed `glove.tactile_raw()` — subscribe to `glove.tactile()` (calibrated tactile frames) instead
- **Wuji Glove**: Removed `glove.factory_reset()`

### Fixed

- Fixed a harmless warning on Python script exit, and ensured devices publish their offline status reliably when `disconnect_all()` is called from non-async contexts

## [0.9.0] - 2026-04-07

### Fixed

- Fixed a crash when pausing and resuming a recording session
- Fixed out-of-order timestamps in MCAP files after pausing and resuming

## [0.8.0] - 2026-03-23

### Added

- Added multi-client support so the Wuji SDK can share a device with Wuji Studio
- Late subscribers now receive a short replay of recent decoded device logs before switching to the live stream

### Changed

- MCAP recording stop no longer hangs when channels are idle
- Encoder shutdown failures are now surfaced instead of being reported as success
- Reconnecting to a device no longer replays logs from the previous connection session
- Device log subscription now uses a dedicated receiver type that handles replay and live delivery transparently

## [0.7.0] - 2026-03-09

### Added
- Cross-device merged topics — `manager.subscribe("tf_static")` aggregates data from all connected devices into a single stream
- MCAP recording engine — record multi-channel sensor data to MCAP files with LZ4/Zstd compression
  - `TopicRecorder` for configuring and starting recording sessions
  - `RecordingHandle` for pause/resume/stop control
  - Real-time quality monitoring: frame drop rate, jitter, cross-channel sync offset
  - Episode switching: reuse session config, switch output file for new episodes

### Improved
- **Wuji Glove**: Improved tactile point cloud accuracy with mesh-based skinning deformation, replacing the previous geometric approximation

### Fixed
- Fix a crash (segfault) when an unhandled exception occurs while callback subscriptions are active
- Fix an issue where subscribing to multiple resources simultaneously could result in missing data
- **Wuji Glove**: Fix an issue that prevents certain topics (e.g. `imu_data/palm`) from being subscribed

## [0.6.0] - 2026-02-14

### Added
- `get()` / `set()` now support reading and writing device configuration parameters
- **Wuji Glove**: `glove.save_params()` to persist parameter changes to device flash
- SDK/Device logging support, use `set_log_level(level)` to control log level
- **Wuji Glove**: Device logging — `glove.get_device_log_dir()`, `glove.set_device_log_elf()` for firmware log decoding
- **Wuji Glove**: IMU data for 6 sensors — `glove.imu_palm()`, `glove.imu_thumb()`, `glove.imu_index()`, `glove.imu_middle()`, `glove.imu_ring()`, `glove.imu_pinky()` for accelerometer, gyroscope, and fused orientation data
- **Wuji Glove**: Static coordinate transforms — `glove.tf_static().subscribe()` for fixed frame relationships (wrist → emf_tx, wrist → palm_imu_link) at 1 Hz
- **Wuji Glove**: Dynamic coordinate transforms — `glove.tf().subscribe()` for IMU-driven wrist orientation (waist → wrist) in real-time, with adjustable waist→wrist offset
- **Wuji Glove**: Hand tracking data — `glove.tip_poses()` for fingertip poses, `glove.hand_joint_angles()` for IK-solved joint angles (21 DoF), `glove.hand_skeleton()` for 21 MediaPipe hand landmarks
- **Wuji Glove**: Tactile point cloud — `glove.tactile_point_cloud()` for 3D tactile visualization

### Changed
- **Wuji Glove**: Positioning quality metric `EmfPose.confidence` — indicates EMF pose estimation reliability

## [0.5.0] - 2026-02-09

### Added
- **Wuji Glove**: Semantic API support
- Callback subscription support with `subscribe_with_callback()`
- Async/await support with `recv_async()`
- Complete type stubs for IDE support
- **Wuji Glove**: Data streams: tactile, tactile_zones, emf_poses

### Supported Devices
- Wuji Glove - Glove with tactile and EMF sensors

[Unreleased]: https://github.com/wuji-technology/wuji-sdk/compare/v2026.8.17...HEAD
[2026.8.17]: https://github.com/wuji-technology/wuji-sdk/compare/v2026.8.3...v2026.8.17
[2026.8.3]: https://github.com/wuji-technology/wuji-sdk/compare/v2026.7.21...v2026.8.3
[2026.7.21]: https://github.com/wuji-technology/wuji-sdk/compare/v2026.7.14...v2026.7.21
[2026.7.14]: https://github.com/wuji-technology/wuji-sdk/compare/v2026.7.2...v2026.7.14
[2026.7.2]: https://github.com/wuji-technology/wuji-sdk/compare/v2026.7.1...v2026.7.2
[2026.7.1]: https://github.com/wuji-technology/wuji-sdk/compare/v2026.6.18...v2026.7.1
[2026.6.18]: https://github.com/wuji-technology/wuji-sdk/compare/v2026.6.16...v2026.6.18
[2026.6.16]: https://github.com/wuji-technology/wuji-sdk/compare/v2026.6.15...v2026.6.16
[2026.6.15]: https://github.com/wuji-technology/wuji-sdk/compare/v2026.6.2...v2026.6.15
[2026.6.2]: https://github.com/wuji-technology/wuji-sdk/compare/v2026.5.26...v2026.6.2
[2026.5.26]: https://github.com/wuji-technology/wuji-sdk/compare/v0.10.0...v2026.5.26
[0.10.0]: https://github.com/wuji-technology/wuji-sdk/compare/v0.9.0...v0.10.0
[0.9.0]: https://github.com/wuji-technology/wuji-sdk/compare/v0.8.0...v0.9.0
[0.8.0]: https://github.com/wuji-technology/wuji-sdk/compare/v0.7.0...v0.8.0
[0.7.0]: https://github.com/wuji-technology/wuji-sdk/compare/v0.6.0...v0.7.0
[0.6.0]: https://github.com/wuji-technology/wuji-sdk/compare/v0.5.0...v0.6.0
[0.5.0]: https://github.com/wuji-technology/wuji-sdk/releases/tag/v0.5.0
