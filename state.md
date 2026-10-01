# EKF Sensor-Fusion Handoff

This document records the current EKF/sensor-fusion design, recent changes,
and the validation work still required before relying on new measurements.

## EKF configuration

The active EKF profiles are:

- `onboard/src/sensor_fusion/config/oogway.yaml`
- `onboard/src/sensor_fusion/config/crush.yaml`
- `onboard/src/sensor_fusion/config/oogway_shell.yaml`

The filter state order in `robot_localization` is:

`[x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]`

The EKF publishes local odometry in the `odom` frame with `base_link` as the
vehicle body frame. There is no global position source, so horizontal position
is dead-reckoned from velocity and can drift.

## Fused measurements

| Source | Measurement currently fused | Notes |
| --- | --- | --- |
| DVL | `vx`, `vy`, `vz` | Bottom-track velocity. Quality-aware covariance and validity gating are implemented. |
| Pressure sensor | Relative `z` position | `pose0_differential: false` and `pose0_relative: true` retain the startup depth as zero while directly bounding vertical drift. |
| VectorNav VN-100 | Angular velocity X/Y | `crush` has pre-existing additional orientation/acceleration selections; see the configuration before modifying it. |
| Separate gyro | Angular velocity Z / yaw rate | Fixed zero-bias subtraction and conservative fixed covariance. |

The DVL can publish attitude, but DVL roll/pitch EKF fusion is intentionally
disabled by default pending pool validation of axes, signs, and mounting frame.
DVL yaw remains disabled.

## DVL changes

Vehicle hardware mapping:

- **Oogway** uses the Teledyne Pathfinder DVL.
- **Crush** uses the Teledyne Wayfinder DVL.

The Pathfinder driver exposes the full per-beam diagnostics used by the
quality-aware fusion path: range, correlation, intensity, percent-good, RSSI,
and error velocity. The Wayfinder path currently exposes valid bottom-track
velocity, beam/mean range, and error velocity, but does not expose equivalent
per-beam correlation/percent-good diagnostics. Its DVL covariance therefore
uses the baseline plus error-velocity term rather than the Pathfinder's full
quality multiplier.

Relevant files:

- `core/src/custom_msgs/msg/DVLRaw.msg`
- `onboard/src/dvl_pathfinder/src/dvl_pathfinder.cpp`
- `onboard/src/sensor_fusion/sensor_fusion/dvl_to_odom.py`

`DVLRaw` now carries:

- bottom-track validity (`bs_status`);
- DVL error velocity (`bi_error`);
- attitude and `sa_valid` when available;
- per-beam range, correlation, intensity, percent-good, and RSSI, with
  `bt_quality_valid` indicating that those diagnostics are present.

The Pathfinder driver marks a fix invalid when it lacks usable velocity, three
or more valid bottom ranges, or finite XYZ velocity. It publishes velocity and
error velocity in the common raw-message convention of mm/s. `dvl_to_odom`
converts those values to m/s.

`dvl_to_odom` rejects invalid bottom-track fixes and fewer than three usable
beam ranges. Low percent-good/correlation, a reduced valid-beam count, and
higher error velocity increase covariance instead of creating new untested
hard rejection thresholds.

The baseline DVL velocity variance is `0.0225 m^2/s^2` (standard deviation
about `0.15 m/s`) before quality/error multipliers.

## VectorNav VN-100

The VN-100 is an IMU/AHRS, not a GNSS/INS. It can provide:

- quaternion and yaw/pitch/roll attitude;
- compensated and uncompensated gyro/accelerometer vectors;
- magnetometer, temperature, and barometric pressure;
- delta angle/delta velocity and AHRS heave-related outputs when configured.

It does **not** provide trustworthy absolute underwater position, altitude, or
depth. Its barometer measures enclosure/air pressure, not external water depth
on a sealed vehicle.

The project adapter currently publishes `/vectornav/imu`,
`/vectornav/imu_uncompensated`, `/vectornav/magnetic`,
`/vectornav/temperature`, and `/vectornav/pressure`. The published covariance
arrays are set by configuration; the VN-100 does not supply a per-message ROS
covariance matrix.

Current conservative VectorNav covariance settings in
`onboard/src/vectornav/vectornav/config/vectornav.yaml`:

- roll/pitch orientation: `0.04 rad^2`;
- magnetic yaw orientation: `0.25 rad^2`;
- angular rate XYZ: `0.01`, `0.01`, `0.02 rad^2/s^2`;
- linear acceleration XYZ: `0.25 (m/s^2)^2`.

Do not enable new acceleration fusion until vehicle-level data has been
recorded. When acceleration is fused, set
`imu0_remove_gravitational_acceleration: true`; this is now present in all EKF
profiles and only changes behavior where acceleration is selected.

VectorNav yaw is a magnetic absolute heading while the separate gyro measures
yaw rate. They are complementary, not directly comparable. Keep VectorNav yaw
disabled until hard/soft-iron calibration and thruster/motor interference tests
show that it is stable. If validated, use it only as a high-variance, gated,
slow correction of gyro-integrated yaw.

## Other covariance and innovation settings

- Pressure depth variance: `0.04 m^2` (standard deviation about `0.2 m`).
- Separate gyro yaw-rate variance: `0.03 rad^2/s^2`.
- DVL attitude variance when valid: `0.25 rad^2`; invalid attitude is assigned
  `1e6 rad^2` so it has negligible effect if later enabled.
- DVL, IMU, depth, and gyro measurements have Mahalanobis rejection thresholds
  of `3.0` in the EKF profiles.

These are deliberately conservative starting values, not calibrated truth.
The measurement covariance should eventually be set from recorded vehicle data:

`measurement variance = observed stationary variance × safety factor`

Start with a safety factor of roughly 4–10, then use sensor health/quality
signals to increase variance during poor conditions.

## Pool-test checklist

1. Rebuild `custom_msgs` and all dependent packages. `DVLRaw.msg` changed.
2. Record a 2–5 minute stationary bag with thrusters off, then another with
   thrusters on.
3. At rest, check DVL velocity mean, DVL bottom-lock status/beam ranges,
   pressure-depth stability and sign, gyro yaw-rate mean, and VectorNav output.
4. Verify VectorNav raw acceleration magnitude is near `9.81 m/s^2` at rest
   before gravity removal.
5. Run slow, known surge/sway/vertical motions and compare DVL velocity and
   integrated displacement to observed motion.
6. Test known roll/pitch motions and compare DVL attitude to VectorNav before
   enabling DVL roll/pitch fusion.
7. Test heading while motors/thrusters switch. Do not fuse VectorNav yaw if it
   moves with electrical load or does not return consistently after rotations.
8. Monitor `/diagnostics` for frequent EKF measurement rejections. Repeated
   rejections usually indicate an axis, sign, frame, timestamp, or covariance
   error.

## Build note

Python syntax, YAML parsing, and diff whitespace checks passed in this
environment. ROS compilation was not run because `colcon` and `cmake` are not
installed here.
