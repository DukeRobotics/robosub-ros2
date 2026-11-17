# Sensor Fusion

This package uses `robot_localization` to combine DVL velocity, pressure depth,
IMU roll/pitch rates, and the separate gyro's yaw rate. The existing DVL driver,
`dvl_to_odom`, and EKF handle the pipeline; no additional nodes are needed.
Robot profiles are in `config/{oogway,oogway_shell,crush}.yaml`.

## Pathfinder formats and PD0 recording

The Python reader accepts **both PD6 and PD13**; it does not configure the
instrument. `pathfinder_format` defaults to `pd6` and selects the reader, not
the hardware's persistent output setting.

| Format | Use |
| --- | --- |
| PD6 | Text bottom/water velocity, status and error velocity; sufficient for normal fusion. |
| PD13 | Similar text output, adding pressure and individual beam ranges; useful for bottom-lock checks. |
| PD0 | Binary diagnostic ensembles with selectable quality, configuration and timing blocks; useful for calibration bags. |

Neither PD6 nor PD13 exposes per-beam correlation magnitudes. PD0 can provide
correlation, intensity, percent-good, ranges and RSSI, plus optional high-resolution
velocity (`5803h`) and navigation uncertainty/timing (`2013h`) when the device's
features support them. It exposes more evidence for covariance tuning; changing
the encoding does not itself improve acoustic accuracy. Enabled blocks affect
packet size and available serial bandwidth. See the
[Pathfinder guide](https://www.teledynemarine.com/en-us/resources/Documents/Brand%20Support/RD%20INSTRUMENTS/Technical%20Resources/Manuals%20and%20Guides/Pathfinder/PathFinder%20DVL%20Guide_Apr22.pdf),
“Choosing an Output Data Format.”

For PD0 tests, reuse the existing C++ `dvl_pathfinder` driver in place of the
Python reader. No extra driver runs concurrently. Configure the device for PD0
using the vendor setup tool, with fixed leader `0000h`, bottom track `0600h`,
and any supported diagnostic blocks needed for the experiment. Use ship
coordinates (`EX10xxx`, preserving the intended tilt/three-beam flags) for
decoded velocity; other coordinate systems are recorded but not fused.

After sourcing ROS and the core workspace, rebuild from `onboard/`:

```bash
colcon build --packages-select dvl_pathfinder offboard_comms execute
source install/setup.bash
export ROBOT_NAME=oogway
ros2 launch execute robot.launch.py pathfinder_format:=pd0
```

In another sourced terminal, record:

```bash
ros2 bag record -o pd0_covariance_run /sensors/dvl/pd0 /sensors/dvl/raw \
  /sensors/dvl/odom /sensors/depth /sensors/gyro/angular_velocity/twist \
  /vectornav/imu /state /diagnostics /tf /tf_static
```

`/sensors/dvl/pd0` is `std_msgs/msg/UInt8MultiArray`: each message contains one
complete checksum-verified ensemble, including its header/checksum and optional
blocks. Bag timestamps are ROS receipt times, not hardware ping times; native
timing fields remain in the bytes for offline analysis. `/sensors/dvl/raw`
contains decoded bottom velocity/quality for convenient plotting. Record the
binary topic too so optional blocks are preserved even if not decoded yet.

This PD0 test reader currently maps **bottom track only** into fusion; water-track
blocks remain in the binary recording. Use PD6/PD13 for the existing online
water-track fallback. To return, stop the launch, restore the device's PD6
output setting, and launch with `pathfinder_format:=pd6` (or omit the argument).
Do not independently start both serial drivers. The standalone existing
`ros2 launch dvl_pathfinder dvl_pathfinder.xml` is also available when the Python
reader is stopped.

## DVL bottom lock and water-track fallback

- Prefer bottom-track velocity with status `A`, finite XYZ, and at least three
  positive finite beam ranges when ranges are supplied. Missing/sentinel data
  and unknown status do not constitute bottom lock.
- Weak correlation/percent-good on an otherwise valid bottom fix increases its
  covariance. Correlation 64 is an initial normalization value, not a calibrated
  rejection threshold. The Python PD13 stream exposes ranges/status/error
  velocity, but not correlation; do not interpret absent diagnostics as zeros.
- On loss of bottom lock, the same converter selects a valid, fresh Pathfinder
  water-track fix (`WS` status `A`, finite XYZ, and `WD` age at most one second).
  Bottom track takes priority again as soon as valid lock returns. No old fix
  is republished with a fresh timestamp. With neither source valid, there is
  no DVL correction.
- IMU acceleration fusion is disabled in all profiles, including during long
  dropouts. Pressure still anchors relative depth; angular-rate fusion remains.

Configure the Pathfinder to output PD6/PD13 with water-mass tracking enabled.
`BK2` requests a water ping after an unsuccessful bottom ping; `BK1` supplies
both on each cycle and is useful for measuring their difference. Set the
reference layer with `BL` for the deployment and verify actual `WI`, `WS`, and
`WD` sentences. The driver does not change persistent hardware settings.
In water-only `BK3` mode there is no `BD` delimiter, so the driver finishes the
previous ensemble at the next `SA`/`TS` sentence (one ensemble of delivery lag).
See the [Pathfinder guide](https://www.teledynemarine.com/en-us/resources/Documents/Brand%20Support/RD%20INSTRUMENTS/Technical%20Resources/Manuals%20and%20Guides/Pathfinder/PathFinder%20DVL%20Guide_Apr22.pdf),
sections on BK/BL and PD6/PD13.

Wayfinder's documented interface exposes bottom-track velocity only: it has
neither a water-track output nor per-beam correlation magnitudes. It uses
correlation internally, while this driver checks finite XYZ, instrument-frame
coordinates and three usable bottom ranges. Do not assume Pathfinder fallback
exists on Crush. See the [Wayfinder output specification](https://www.teledynemarine.com/en-us/support/Pages/Wayfinder-Binary-Interface-Packet-Protocol.aspx).

Water-track velocity is relative to moving water, not the bottom. It can bridge
loss of ground reference but will drift with current. Its initial variance is
larger than bottom track's (`0.25` versus `0.0225 (m/s)^2`, before error terms).
These are starting values in `dvl_to_odom.py`, not measured accuracy claims.

## Plan for measuring sensor covariance

1. **Verify frames and units first.** Check known surge/sway/heave and positive
   roll/pitch/yaw motions against independent observations. Confirm DVL mm/s
   becomes m/s, depth sign, mounting transforms, and angular rates in rad/s.
   Resolve rotated axes, scale errors and timestamp delays before tuning noise.
2. **Collect repeatable data.** Record 3–5 minutes stationary with warm sensors,
   then repeat under several thruster loads and temperatures. Add known-distance
   translations and known-angle turns in both directions. Save robot YAML,
   hardware settings, environmental conditions and independent ground truth.

   ```bash
   ros2 bag record -o covariance_run /sensors/dvl/raw /sensors/dvl/odom \
     /sensors/depth /sensors/gyro/angular_velocity/twist \
     /vectornav/imu /vectornav/magnetic /state /diagnostics /tf /tf_static
   ```

3. **Separate bias from random noise.** Time-align each sensor to an independent
   reference in the same frame. For residuals `r = measurement - reference`,
   report mean bias, RMSE, and `R_ij = cov(r_i, r_j)`. Covariance uses squared
   units: m², (m/s)², rad², or (rad/s)². Correct persistent bias separately;
   increasing variance does not calibrate it. Stationary noise alone misses
   dynamic scale/alignment errors. An EKF already using the sensor is not
   independent ground truth.
4. **Characterize bottom-lock quality.** Compare clean fixes, three-beam fixes,
   partial obstruction, different bottom materials/altitudes, and motor loads.
   Bin residual error by beam count, error velocity, and correlation/percent-good
   only where the output actually provides them. PD13 cannot supply an acoustic
   correlation curve; that experiment requires a diagnostic output such as PD0.
   Choose baseline variance and conservative quality inflation from held-out
   runs; do not copy a universal correlation threshold. Add cross-axis covariance
   only if residuals support a symmetric positive-semidefinite matrix.
5. **Characterize water tracking separately.** With both modes available, measure
   water/bottom velocity differences to expose current, then deliberately lose
   bottom lock. Verify fresh WS data is selected, stale/invalid WS data is not,
   and bottom recovery restores the ground-referenced source. Test 1, 5, 15 and
   30-second gaps against independent position/velocity measurements. Record
   drift, uncertainty and recovery error, including in moving water. Do not tune
   water-track covariance from still-water tests alone.
6. **Compare yaw sources on identical runs.** Replay three configurations:
   (A) separate gyro yaw rate, (B) VectorNav Z rate with the separate gyro source
   disabled, and (C) separate gyro plus VectorNav absolute yaw. IMU mask index 11
   selects Z rate; index 5 selects yaw orientation. Keep all other settings fixed.
   Transform to common axes and reconcile initial heading before comparison;
   unwrap yaw. Measure stationary drift (degrees/minute), known-turn RMSE,
   return-to-heading error, latency, and motor-induced heading steps across
   temperatures and pool locations. Test magnetic calibration/interference before
   choosing C. Keep VectorNav absolute yaw disabled if it follows electrical load.
7. **Apply and validate.** Put measured diagonal variances in the existing
   publishers/VectorNav YAML; retain separate bottom/water variances. Check
   pressure `pose.covariance[14]`, gyro `twist.covariance[35]`, and DVL
   `twist.covariance[0,7,14]`. Tune EKF process noise only after measurement
   covariance. Validate on complete runs excluded from tuning, and document the
   accepted current conditions and maximum tested dropout duration.

`DVLRaw` includes water-track fields, so rebuild `custom_msgs` and downstream
onboard packages and source the intended overlay before testing. Check live
parameters to confirm acceleration entries 12–14 in `imu0_config` are false,
`pose0_differential` is false, and `pose0_relative` is true. ROS compilation and
vehicle tests require the Jazzy environment.

## Dummy State Publisher

`dummy_state_publisher.py` publishes dummy odometry and the `odom` → `base_link`
transform for local testing when real sensor fusion cannot run.
