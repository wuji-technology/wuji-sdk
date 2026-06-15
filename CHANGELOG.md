# Changelog

All notable changes to wuji-sdk will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

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

[Unreleased]: https://github.com/wuji-technology/wuji-sdk/compare/v2026.6.15...HEAD
[2026.6.15]: https://github.com/wuji-technology/wuji-sdk/compare/v2026.6.2...v2026.6.15
[2026.6.2]: https://github.com/wuji-technology/wuji-sdk/compare/v2026.5.26...v2026.6.2
[2026.5.26]: https://github.com/wuji-technology/wuji-sdk/compare/v0.10.0...v2026.5.26
[0.10.0]: https://github.com/wuji-technology/wuji-sdk/compare/v0.9.0...v0.10.0
[0.9.0]: https://github.com/wuji-technology/wuji-sdk/compare/v0.8.0...v0.9.0
[0.8.0]: https://github.com/wuji-technology/wuji-sdk/compare/v0.7.0...v0.8.0
[0.7.0]: https://github.com/wuji-technology/wuji-sdk/compare/v0.6.0...v0.7.0
[0.6.0]: https://github.com/wuji-technology/wuji-sdk/compare/v0.5.0...v0.6.0
[0.5.0]: https://github.com/wuji-technology/wuji-sdk/releases/tag/v0.5.0
