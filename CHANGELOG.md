# Changelog

> Category hints: `Added`, `Changed`, `Deprecated`, `Removed`, `Fixed`

All notable changes to this project will be documented in this file.

## [v0.0.2] - 2026.5.9
### Added
- Added compatibility for Gazebo-converted `PointXYZIRT` style input topics in the E1R simulation workflow (`/E1R/lidar/points_xyzirt`, `/E1R2/lidar/points_xyzirt`), enabling direct use of converted point clouds in Super-LIO.

### Fixed
- Fixed HESAI16 point-cloud parsing robustness by rejecting invalid/NaN timestamps and intensities, skipping negative offset times, and using the maximum valid point offset to compute frame end time.
- Fixed potential runtime crash (`exit code -8` / SIGFPE) in aggressive-motion scenarios by adding time-validity guards in `sync_measure` (non-finite timestamps and reversed lidar time ranges are now dropped safely).

---

## [v0.0.1] - 2026.5.9
### Added
- Added multi-lidar support mode where primary lidar remains the only source for LIO estimation, while the auxiliary lidar is transformed by configurable extrinsics and fused only for point-cloud output.
- Added auxiliary lidar buffering and nearest-timestamp pairing for fusion publishing, with configurable time-difference gating and buffer-size limits.
- Added a dedicated fused point-cloud output configuration under `lio.multi` (`fused_topic`, `fused_frame_id`), defaulting to Super-LIO frame convention (`imu`).
- Added grouped YAML configuration keys for the second lidar under `lio.multi.lidar2.*` to improve readability and maintainability.

### Changed
- Changed multi-lidar YAML style to recommend `lio.multi.lidar2.topic`, `lio.multi.lidar2.type`, and `lio.multi.lidar2.extrinsic_lidar2_lidar1`, while keeping backward compatibility with legacy keys.
- Changed fused cloud frame-id defaults from FAST-LIO-style `_body` to Super-LIO-style `imu`.
- Changed `/lio/robo/odom` publishing semantics by setting `child_frame_id` to `robo`.

### Fixed
- Fixed `/lio/robo/odom` missing twist output by publishing robot-frame linear and angular velocity instead of all-zero values.
- Fixed robot-odometry twist computation by deriving robot velocity from IMU state and odom-to-robot extrinsic when `state_robot.v/w/a` is not provided by the current ESKF path.

---

