# Changelog

All notable changes to wuji-sdk will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

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

[Unreleased]: https://github.com/wuji-technology/wuji-sdk/compare/v0.8.0...HEAD
[0.8.0]: https://github.com/wuji-technology/wuji-sdk/compare/v0.7.0...v0.8.0
[0.7.0]: https://github.com/wuji-technology/wuji-sdk/compare/v0.6.0...v0.7.0
[0.6.0]: https://github.com/wuji-technology/wuji-sdk/compare/v0.5.0...v0.6.0
[0.5.0]: https://github.com/wuji-technology/wuji-sdk/releases/tag/v0.5.0
