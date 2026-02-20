"""Acoustic data service client example.

This client calls the `GetAcousticData` service and saves any returned
`AcousticSample` payloads to files. It's defensive about missing fields so it
can be used before message definitions are finalized.
"""

import argparse
import os
import wave
import traceback
from typing import Any

import rclpy
from rclpy.node import Node

# Import the service type from this package. This will be available once
# the package is built with rosidl support.
try:
    from acoustics.srv import GetAcousticData
except Exception:  # pragma: no cover - allow editing before build
    GetAcousticData = None  # type: ignore


class AcousticClient(Node):
    def __init__(self, service_name: str = 'get_acoustic_data'):
        super().__init__('acoustic_client')
        if GetAcousticData is None:
            self.get_logger().warn('GetAcousticData service type not available yet')
        self.cli = self.create_client(GetAcousticData, service_name) if GetAcousticData else None

    def wait_for_service_ready(self, timeout_sec: float = 1.0) -> bool:
        if not self.cli:
            return False
        ready = False
        while not ready:
            ready = self.cli.wait_for_service(timeout_sec=timeout_sec)
            if not ready:
                self.get_logger().info('service not available, waiting...')
        return True

    def build_request(self, start_time: float | None, end_time: float | None, duration: float | None) -> Any:
        if GetAcousticData is None:
            return None
        req = GetAcousticData.Request()
        # Flexible API: prefer explicit start/end, else use duration from now.
        if start_time is not None:
            try:
                req.start_time = float(start_time)
            except Exception:
                pass
        if end_time is not None:
            try:
                req.end_time = float(end_time)
            except Exception:
                pass
        if duration is not None:
            try:
                req.duration = float(duration)
            except Exception:
                pass
        return req

    def call_service(self, req: Any):
        if not self.cli:
            raise RuntimeError('service type or client not available')
        return self.cli.call_async(req)


def _save_sample(sample: Any, out_dir: str, index: int) -> str:
    os.makedirs(out_dir, exist_ok=True)
    # Defensive attribute access
    encoding = getattr(sample, 'encoding', None)
    sample_rate = getattr(sample, 'sample_rate', None)
    channels = getattr(sample, 'channels', 1)
    data_field = getattr(sample, 'data', None)

    # Try to coerce data to bytes
    data_bytes = b''
    if data_field is None:
        data_bytes = b''
    elif isinstance(data_field, (bytes, bytearray)):
        data_bytes = bytes(data_field)
    else:
        try:
            data_bytes = bytes(data_field)
        except Exception:
            data_bytes = b''

    # Determine filename and write appropriately
    if encoding and 'wav' in str(encoding).lower():
        filename = os.path.join(out_dir, f'acoustic_{index}.wav')
        with open(filename, 'wb') as f:
            f.write(data_bytes)
        return filename

    if encoding and any(x in str(encoding).lower() for x in ('pcm16', 'pcm_s16', 's16')) and sample_rate:
        filename = os.path.join(out_dir, f'acoustic_{index}.wav')
        try:
            with wave.open(filename, 'wb') as wf:
                wf.setnchannels(int(channels or 1))
                wf.setsampwidth(2)
                wf.setframerate(int(sample_rate))
                wf.writeframes(data_bytes)
            return filename
        except Exception:
            # fallback to raw
            pass

    # generic binary dump
    filename = os.path.join(out_dir, f'acoustic_{index}.bin')
    with open(filename, 'wb') as f:
        f.write(data_bytes)
    return filename


def main():
    parser = argparse.ArgumentParser(description='Acoustic service client')
    parser.add_argument('--service', default='get_acoustic_data')
    parser.add_argument('--start', type=float, default=None)
    parser.add_argument('--end', type=float, default=None)
    parser.add_argument('--duration', type=float, default=None)
    parser.add_argument('--out', default='acoustic_output')
    args = parser.parse_args()

    rclpy.init()
    node = AcousticClient(service_name=args.service)
    try:
        if not node.wait_for_service_ready():
            node.get_logger().error('Service not available; exiting')
            return

        req = node.build_request(args.start, args.end, args.duration)
        if req is None:
            node.get_logger().error('Request type not available (srv not generated yet)')
            return

        future = node.call_service(req)
        rclpy.spin_until_future_complete(node, future)
        try:
            resp = future.result()
        except Exception:
            node.get_logger().error('Service call failed: %s' % traceback.format_exc())
            return

        # Prefer common fields if present
        success = getattr(resp, 'success', True)
        message = getattr(resp, 'message', '')
        samples = getattr(resp, 'samples', [])

        if not success:
            node.get_logger().error('Service returned error: %s' % message)
            return

        if not samples:
            node.get_logger().info('Service returned no samples')
            return

        saved = []
        for i, s in enumerate(samples):
            try:
                path = _save_sample(s, args.out, i)
                saved.append(path)
            except Exception:
                node.get_logger().error('Failed saving sample %d: %s' % (i, traceback.format_exc()))

        if saved:
            node.get_logger().info('Saved %d files to %s' % (len(saved), os.path.abspath(args.out)))
        else:
            node.get_logger().info('No files saved')

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()