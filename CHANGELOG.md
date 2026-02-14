# Changelog

All notable changes to wuji-sdk will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

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

[Unreleased]: https://github.com/wuji-technology/wuji-sdk/compare/v0.6.0...HEAD
[0.6.0]: https://github.com/wuji-technology/wuji-sdk/compare/v0.5.0...v0.6.0
[0.5.0]: https://github.com/wuji-technology/wuji-sdk/releases/tag/v0.5.0
