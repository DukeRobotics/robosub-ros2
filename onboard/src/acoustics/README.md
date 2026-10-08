# Acoustics

One-shot Saleae Logic8 capture and hydrophone processing through
`/acoustics/request`. Analog 0–3 maps to A0–A3. Capture, native loading,
DC removal, band-pass/envelope TOA analysis, validity checks, and the nearby
classifier are copied from
[DukeRobotics/acoustics-v3](https://github.com/DukeRobotics/acoustics-v3/tree/8af76833520f304c9900e8775da839b7b320b584).
[Copy provenance](acoustics/upstream/README.md) documents the source and model.

## Run the service

Start Logic2 with its automation server enabled. The Python runtime needs
`logic2-automation` (installed by the repository Dockerfile).
The copied wrapper connects to the running instance, selects the first physical
device, enables analog 0–3, waits for one timed capture, exports binaries, and
closes the capture/connection. Connect only the intended Logic8.

Rebuild `custom_msgs`, source its install, then build and source `acoustics`.
The extended service schema requires rebuilding clients.

```bash
# In core/
colcon build --packages-select custom_msgs
# In onboard/, after sourcing the core install
colcon build --packages-select acoustics

ros2 launch acoustics acoustics.xml
ros2 service call /acoustics/request custom_msgs/srv/AcousticsRequest '{}'
```

The robot launch already includes `acoustics.xml`. Each request performs one
capture; there are no recurring captures, sample subscriptions, result topics,
or multiepoch voting. The callback blocks during capture/export/analysis.

Response fields:

- `closest`: earliest calibrated TOA, indexed 0–3, or -1 when all-four-channel
  TOA analysis is invalid. This is not a direction estimate.
- `nearby`: any channel classified nearby, gated by TOA validity.
  `nearby_available` distinguishes a disabled classifier from a negative result.
  The upstream learned 10 ft category is not a metric range estimate.
- `analysis_valid`, raw/calibrated channel TOAs, validity reasons, classifier
  confidences, capture directory, timestamp reference, and overall status/reason.
- Common-window detections and six-pair delay/bearing results.

Robust TDOA remains unfinished: a valid detection produces
`status=not_implemented` with no fabricated direction/position, even when
`analysis_valid=true`. Capture/processing errors return `invalid` and
`closest=-1`.

## Configuration and timing

[config/dsp.yaml](config/dsp.yaml) contains capture, filter/detection, calibration,
nearby-model, geometry, sound-speed, and estimator settings.
Launch with `config_file:=/absolute/path/dsp.yaml` to use another file.
Scalar settings are ROS parameters; geometry overrides are `channel_ids`,
`coordinates_m` (12 row-major floats), `frame_id`, and `timing_offsets_s`
(four floats). Settings are loaded at startup.

Defaults match upstream: 2 s capture, 781250 samples/s, order-6 30–34 kHz
band-pass, envelope threshold sigma 5, raw signal threshold 0.5, and 0.1 s
arrival margins. Geometry is an **illustrative 5 cm tetrahedron**; replace it
with measured coordinates before using localization.

Positive offset means the channel records late:

```text
calibrated_toa_i = raw_toa_i - offset_i
tau_ij = t_i - t_j
calibrated_tau_ij = raw_tau_ij - (offset_i - offset_j)
```

Calibration never shifts waveform samples or independently crops channels.
All channels share one filtered window and retain original sample indices.
Waveform delay candidates are calibrated once, before physical-bound checks.
Measure electronic offsets separately from acoustic propagation delays.

Clipping rejection is configurable but disabled until ADC/amplifier limits are
measured; recordings have about 5 V DC bias. Upstream weak-signal and
arrival-margin validation remain active. Upstream detects one arrival per
channel, with its envelope-peak fallback when no threshold crossing occurs.
This is not robust multiburst or multipath detection.

## Replay native recordings

Replay runs the same processing without ROS or hardware. Use a capture directory
with four Saleae analog v0 binary exports (`analog_0.bin` … `analog_3.bin`,
or upstream-compatible channel suffixes), or one four-channel CSV export.
CSV takes precedence, as upstream. Missing/duplicate channels, unequal lengths,
and mismatched original timelines/rates are rejected. No implicit resampling.

```bash
python3 -m pip install numpy scipy PyYAML pandas matplotlib joblib 'scikit-learn==1.9.0'
PYTHONPATH=onboard/src/acoustics python3 -m acoustics.replay \
  --config onboard/src/acoustics/config/dsp.yaml /absolute/path/to/capture_epoch
```

List multiple directories to replay independent captures. Output is one JSON
result per directory, without large waveform arrays. Invalid input is reported
and replay continues. The bundled model was saved with scikit-learn 1.9.0;
that version is pinned in `setup.py`. Only load trusted joblib/pickle artifacts.

Recordings without a sidecar use recording-relative timestamps. Live capture
writes `capture.json` with a host marker immediately before the SDK capture call.
`host_before_capture_call` is **not a synchronized hardware timestamp**; capture
latency must be characterized before combining bearings with robot poses.
Replay preserves that clock reference but uses calibration from the selected
config, not automatically from the historical sidecar.

## Remaining DSP skeleton

`capture.py` owns the one-shot SDK workflow; `upstream_adapter.py` adds calibration,
synchronization checks, and common-window extraction around copied processing.
`acoustics.py` is the single ROS node. The ROS-free `dsp/` modules retain the
contracts and extension points from the localization task:

- Explicit samples/geometry/detections, six-pair candidate delays, and localization
  results with timestamps, frames, statuses, residuals, ambiguity, and optional
  uncertainty/position.
- Coarse envelope/onset and fine waveform/phase estimator hooks. Tone timing can
  have plausible alternatives spaced by `1/f0`; do not choose only the largest
  correlation peak.
- Physical bounds `abs(tau_ij) <= norm(h_i-h_j)/c` and redundant-pair consistency.
  Four channels provide three independent delays; the six pairs must satisfy
  `tau_ij + tau_jk = tau_ik`.
- A reference far-field fit and separate near-field/moving-robot tracker interfaces.
  Tracking and near-field algorithms are not implemented.

Far-field directions point from array toward source:
`c*tau_ij = -(h_i-h_j) dot u`. Azimuth is `atan2(u_y,u_x)`; elevation is
`atan2(u_z,hypot(u_x,u_y))`, in radians in the configured array frame.
Noncoplanar geometry supports 3D. Coplanar arrays retain reflected candidates
unless an explicit normal-sign constraint is configured; the normal's largest
absolute component is positive for reproducibility. Collinear arrays are rejected.
The unweighted reference fit needs all six valid pairs and preserves alternatives;
candidate-limit exhaustion reports ambiguity without truncating them.
Four hydrophones do not establish reliable single-ping range.

Still needed: robust coarse/fine TDOA and multipath rejection, measured clipping
limits, quality weighting/uncertainty, tracking, and real-device/ROS verification.

## Checks

With pytest and Ruff installed:

```bash
python3 -m pip install logic2-automation
PYTHONPATH=onboard/src/acoustics pytest -q onboard/src/acoustics/test
ruff check onboard/src/acoustics/acoustics onboard/src/acoustics/test onboard/src/acoustics/setup.py
git diff --check
```

Tests cover native replay/CLI, copied TOA/classifier equivalence, one-shot SDK
configuration and cleanup with a fake manager, calibration sign and timing
preservation, bad recordings, physical bounds, pair consistency, and ambiguity.
ROS message generation and real Logic8 capture require the robot environment.
