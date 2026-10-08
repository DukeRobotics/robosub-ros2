"""ROS service adapter for one-shot Saleae Logic8 capture and upstream hydrophone processing."""
import json
from dataclasses import fields
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from custom_msgs.msg import AcousticsBearing, AcousticsDetection, AcousticsSampleBlock, AcousticsTDOA
from custom_msgs.srv import AcousticsRequest
from geometry_msgs.msg import Point, Vector3
from rclpy.node import Node
from std_msgs.msg import Header

from acoustics.capture import CaptureProcessor
from acoustics.dsp.config import DSPConfig, load_config
from acoustics.dsp.contracts import (
    CHANNEL_COUNT,
    ArrayGeometry,
    LocalizationResult,
    PingDetection,
    SampleBlock,
    Status,
    TDOAResult,
)


def make_header(timestamp_s: float, frame_id: str) -> Header:
    """Convert acquisition seconds to a normalized ROS header; preserve the clock reference."""
    nanoseconds = round(timestamp_s * 1e9)
    seconds, nanos = divmod(nanoseconds, 1_000_000_000)
    header = Header(frame_id=frame_id)
    header.stamp.sec, header.stamp.nanosec = seconds, nanos
    return header


def block_message(block: SampleBlock, frame_id: str) -> AcousticsSampleBlock:
    """Serialize a common-grid window without shifting individual channels."""
    return AcousticsSampleBlock(
        header=make_header(block.timestamp_s, frame_id), channel_ids=list(block.channel_ids),
        sample_rate_hz=block.sample_rate_hz, units=block.units, first_sample_index=block.first_sample_index,
        timestamp_reference=block.timestamp_reference,
        sample_count=len(block.samples), samples=block.samples.ravel().tolist(),
    )


def detection_message(detection: PingDetection, frame_id: str) -> AcousticsDetection:
    """Return original indices, the common waveform window, and explicit detection quality."""
    timestamp = detection.window.timestamp_s + (
        detection.onset_sample_index - detection.start_sample_index
    ) / detection.window.sample_rate_hz
    return AcousticsDetection(
        header=make_header(timestamp, frame_id), start_sample_index=detection.start_sample_index,
        end_sample_index=detection.end_sample_index, onset_sample_index=detection.onset_sample_index,
        has_frequency=detection.frequency_hz is not None, frequency_hz=detection.frequency_hz or 0.0,
        window=block_message(detection.window, frame_id), quality_names=list(detection.quality),
        quality_values=list(detection.quality.values()), status=detection.status.value, reason=detection.reason,
    )


def bearing_message(result: LocalizationResult, pairs: tuple[TDOAResult, ...]) -> AcousticsBearing:
    """Return every direction and tone-cycle alternative; unfinished estimates remain explicit."""
    message = AcousticsBearing(
        header=make_header(result.timestamp_s, result.frame_id), status=result.status.value,
        reason=result.reason, ambiguity=result.ambiguity,
        directions=[Vector3(x=c.unit_vector[0], y=c.unit_vector[1], z=c.unit_vector[2])
                    for c in result.direction_candidates],
        azimuth_rad=[c.azimuth_rad for c in result.direction_candidates],
        elevation_rad=[c.elevation_rad for c in result.direction_candidates],
        residual_rms_m=[c.residual_rms_m for c in result.direction_candidates],
        pairs=[AcousticsTDOA(
            channel_i=pair.channel_pair[0], channel_j=pair.channel_pair[1], physical_bound_s=pair.physical_bound_s,
            candidate_delays_s=[c.delay_s for c in pair.candidates],
            candidate_quality_json=[json.dumps(c.quality, allow_nan=False) for c in pair.candidates],
            status=pair.status.value, reason=pair.reason,
        ) for pair in pairs],
        has_position=result.position_m is not None,
        uncertainty=result.uncertainty.ravel().tolist() if result.uncertainty is not None else [],
    )
    if result.position_m is not None:
        message.position = Point(x=result.position_m[0], y=result.position_m[1], z=result.position_m[2])
    return message


class Acoustics(Node):
    """Capture only in response to /acoustics/request; return analysis in the service response."""

    def __init__(self) -> None:
        super().__init__('acoustics')
        default_path = str(Path(get_package_share_directory('acoustics')) / 'config' / 'dsp.yaml')
        base = load_config(self.declare_parameter('config_file', default_path).value)
        values = {field.name: self.declare_parameter(field.name, getattr(base, field.name)).value
                  for field in fields(base) if field.name != 'geometry'}
        geometry = base.geometry
        coordinates = self.declare_parameter('coordinates_m', geometry.coordinates_m.ravel().tolist()).value
        configured_geometry = ArrayGeometry(
            tuple(self.declare_parameter('channel_ids', list(geometry.channel_ids)).value),
            np.asarray(coordinates).reshape(4, 3),
            self.declare_parameter('frame_id', geometry.frame_id).value,
            np.asarray(self.declare_parameter('timing_offsets_s', geometry.timing_offsets_s.tolist()).value),
        )
        if configured_geometry.channel_ids != ('A0', 'A1', 'A2', 'A3'):
            msg = 'Logic8 analog channels 0-3 must map to A0-A3 in that order'
            raise ValueError(msg)
        self.processor = CaptureProcessor(
            DSPConfig(geometry=configured_geometry, **values), clock=lambda: self.get_clock().now().nanoseconds / 1e9,
        )
        self.service = self.create_service(AcousticsRequest, '/acoustics/request', self.perform_acoustics_request)
        self.get_logger().info('Acoustics service ready; each request captures analog 0-3 through running Logic2')

    def perform_acoustics_request(
        self, request: AcousticsRequest.Request, response: AcousticsRequest.Response,  # noqa: ARG002 - empty request
    ) -> AcousticsRequest.Response:
        """Capture once, process all four hydrophones, and return diagnostics/results without publishing."""
        response.closest = -1
        try:
            result, path = self.processor.run()
            response.capture_directory = str(path)
            response.timestamp_reference = 'host_before_capture_call'
            response.status = result.status.value
            response.reason = result.reason or '; '.join(item.reason for item in result.localizations if item.reason)
            analysis = result.channel_analysis
            response.channel_ids = [item.channel_id for item in analysis]
            response.raw_toa_s = [item.raw_toa_s for item in analysis]
            response.calibrated_toa_s = [item.calibrated_toa_s for item in analysis]
            response.channel_valid = [item.valid for item in analysis]
            response.validation_reasons = [item.validation_reason for item in analysis]
            response.analysis_valid = len(analysis) == CHANNEL_COUNT and all(item.valid for item in analysis)
            response.nearby_available = bool(analysis) and all(item.nearby is not None for item in analysis)
            response.nearby_confidence = [item.nearby_confidence for item in analysis
                                          if item.nearby_confidence is not None]
            if response.analysis_valid:
                response.closest = min(range(len(analysis)), key=lambda index: analysis[index].calibrated_toa_s)
                response.nearby = any(item.nearby for item in analysis) if response.nearby_available else False
            frame_id = self.processor.config.geometry.frame_id
            response.detections = [detection_message(item, frame_id) for item in result.detections]
            response.bearings = [bearing_message(item, pairs)
                                for item, pairs in zip(result.localizations, result.delays, strict=True)]
        except Exception as error:  # noqa: BLE001 - RPC boundary returns explicit SDK/acquisition/processing failures
            response.status = Status.INVALID.value
            response.reason = str(error)
            response.analysis_valid = False
            response.closest = -1
            response.nearby = False
            response.nearby_available = False
            self.get_logger().error(f'Acoustics request failed: {error}')
        return response


def main(args: list[str] | None = None) -> None:
    """Run a serialized one-shot capture service using the repository's ROS node lifecycle."""
    rclpy.init(args=args)
    node = None
    try:
        node = Acoustics()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
