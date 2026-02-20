import rclpy
from rclpy.node import Node
from acoustics.srv import GetAcousticData
from std_msgs.msg import Header
import traceback
import math


class AcousticService(Node):
    def __init__(self):
        super().__init__('acoustic_service')
        self.declare_parameter('max_chunk_bytes', 1024 * 1024)
        self.max_chunk = int(self.get_parameter('max_chunk_bytes').value)
        self.declare_parameter('default_sample_rate', 48000)
        self.default_sample_rate = int(self.get_parameter('default_sample_rate').value)
        self.declare_parameter('default_channels', 1)
        self.default_channels = int(self.get_parameter('default_channels').value)
        self.srv = self.create_service(GetAcousticData, 'get_acoustic_data', self.handle_get_acoustic)

    def validate_request(self, request):
        # Basic validation: ensure times/duration are non-negative when provided
        try:
            if request.duration < 0.0:
                return False, 'duration must be >= 0'
            if request.start_time < 0.0 and request.start_time != 0.0:
                # allow zero as sentinel but disallow negative Epoch times accidentally
                return False, 'start_time must be >= 0'
            if request.end_time < 0.0 and request.end_time != 0.0:
                return False, 'end_time must be >= 0'
        except Exception:
            pass
        return True, None

    def handle_get_acoustic(self, request, response):
        try:
            ok, msg = self.validate_request(request)
            if not ok:
                response.success = False
                response.message = msg or 'invalid request'
                return response

            # Determine duration to produce (seconds)
            duration = 1.0
            if hasattr(request, 'duration') and request.duration and request.duration > 0.0:
                duration = float(request.duration)

            # Build a synthetic sample for now (silence) — replace with hardware or storage access
            sample = self._build_sample(duration_seconds=duration)

            response.samples = [sample]
            response.success = True
            response.message = 'ok'
            self.get_logger().info('Served GetAcousticData request (duration=%fs)' % (duration,))
            return response
        except Exception:
            self.get_logger().error('Error handling request: %s' % (traceback.format_exc(),))
            response.success = False
            response.message = 'internal error'
            return response

    def _build_sample(self, duration_seconds: float = 1.0):
        # Create an AcousticSample with silence PCM16 data for the requested duration.
        from acoustics.msg import AcousticSample

        s = AcousticSample()
        # header
        h = Header()
        h.stamp = self.get_clock().now().to_msg()
        h.frame_id = 'acoustic'
        s.header = h

        sample_rate = self.default_sample_rate
        channels = self.default_channels
        s.sample_rate = sample_rate
        s.channels = channels
        s.encoding = 'pcm16'

        # clamp duration to avoid very large messages
        max_duration = max(0.0, float(self.max_chunk) / (sample_rate * channels * 2))
        if duration_seconds > max_duration:
            duration_seconds = max_duration

        total_samples = int(math.floor(sample_rate * duration_seconds))

        # PCM16 silence: zeros
        data_bytes = bytearray(total_samples * channels * 2)
        # store as uint8[]
        s.data = data_bytes

        # simple metadata
        s.metadata = 'generated:duration=%f' % (duration_seconds,)
        return s


def main(args=None):
    rclpy.init(args=args)
    node = AcousticService()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()