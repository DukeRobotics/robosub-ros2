# Copied acoustics-v3 implementation

Source: [DukeRobotics/acoustics-v3](https://github.com/DukeRobotics/acoustics-v3/tree/8af76833520f304c9900e8775da839b7b320b584),
main commit `8af76833520f304c9900e8775da839b7b320b584`.
The main, freq_indep, and logic2_refactoring capture/loading flows were inspected;
main's already-running-Logic2 automation flow is the one integrated here.

Copied from upstream `scripts/`:

- `logic/logic2.py`: Manager.connect, first physical device, analog 0–3,
  timed capture, wait, raw export, and close.
- `hydrophones/hydrophone.py` and `hydrophone_array.py`: channel container,
  native binary/CSV loading, timestamps, and channel DC removal.
- `analyzers/base_analyzer.py`, `garbage_detector.py`,
  `toa_envelope_analyzer.py`, and `nearby_analyzer.py`: shared band-pass,
  envelope/TOA, validity checks, feature extraction, and nearby classification.
- `artifacts/proximity_classifier_10ft_threshold_2026-04-12--23-04-00.pkl`:
  the model selected by upstream main. SHA-256:
  `d57213878423c4ea6519595b4b90e14050446d7007d683afe92379031f16209f`.
  Saved with scikit-learn 1.9.0; the package pins that runtime dependency.

Algorithm bodies and the SDK flow are unchanged. Changes to copied text are
package initialization files, one absolute hydrophone import converted to a
relative import, LF line endings, and provenance/lint-exemption comments.
Lint exemptions deliberately keep the copied algorithm source intact.
Upstream's plotting support remains present, but is disabled by the adapter.

The ROS-free adapter in `../upstream_adapter.py` subclasses HydrophoneArray
for configurable timing offsets, retaining original sample values/indices.
It validates four-channel completeness and matching acquisition timelines,
analyzes all four channels (upstream main currently selects H0), and extracts
one common filtered window. `../capture.py` adds a host-clock sidecar and uses
one binary-export capture per service invocation.

The continuous recorder/controller and multiepoch voting are intentionally not
the ROS execution path: capture is driven by the existing request service.
The new TDOA/localization/tracking contracts are separate from copied TOA and
nearby processing. Classifier output is a learned nearby/far category, not a
metric position or reliable single-ping range.

Only use trusted model files: joblib/pickle deserialization can execute code.
