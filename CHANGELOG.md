# Changelog

All notable changes to wuji-sdk will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

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
- **Wuji Hand 2 — `joint_states` / `joint_diagnostics` feedback frames now carry a `FrameHeader`** (`seq` + `timestamp_us` + `frame_id`), consistent with the IMU / tactile feedback frames. `timestamp_us` is the firmware send time and `frame_id` is `l_wrist` / `r_wrist` (filled by the firmware from its own handedness) for 3D hand-pose visualization. The previous top-level `seq` field moved into `header.seq`. Requires the matching firmware build.
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
- **Wuji Hand 2**: Initial public release — auto-discovery and connection, motor enable/disable, MIT impedance control parameters, real-time joint state subscription (20 joints, position/velocity/torque), multi-finger tactile sensor stream, joint fault clearing, and per-joint diagnostics (bus voltage, temperature, fault codes)

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

[Unreleased]: https://github.com/wuji-technology/wuji-sdk/compare/v2026.7.1...HEAD
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
