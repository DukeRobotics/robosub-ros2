import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# Try to import jtop for Jetson GPU/RAM/temp monitoring
try:
    from jtop import jtop
    JETSON_AVAILABLE = True
except ImportError:
    JETSON_AVAILABLE = False
class SystemMonitor(Node):
    """Monitor and publish Jetson GPU, RAM, and temperature metrics"""
    def __init__(self) -> None:
        super().__init__('system_monitor')
        # Publisher for system metrics
        self.publisher = self.create_publisher(String, '/system/metrics', 10)
        # Timer to publish metrics at 1Hz
        self.timer = self.create_timer(1.0, self.publish_metrics)
        # Initialize Jetson monitoring
        self.jetson = None
        if JETSON_AVAILABLE:
            try:
                self.jetson = jtop()
                self.jetson.start()
                self.get_logger().info(':white_check_mark: Jetson monitoring enabled (GPU/RAM/Temp)')
            except Exception as e:
                self.get_logger().error(f':x: Failed to start Jetson monitoring: {e}')
        else:
            self.get_logger().warn(':warning:  jetson-stats not available - no GPU monitoring')
    def get_jetson_metrics(self):
        """Get GPU, RAM, and temperatures from Jetson"""
        if not self.jetson or not self.jetson.ok():
            return {
                'status': 'unavailable',
                'gpu_percent': 0,
                'memory_percent': 0,
                'temp_cpu': 0,
                'temp_gpu': 0
            }
        try:
            return {
                'status': 'active',
                'gpu_percent': self.jetson.gpu.get('val', 0),
                'memory_used_mb': self.jetson.memory['RAM']['used'],
                'memory_total_mb': self.jetson.memory['RAM']['tot'],
                'memory_percent': (self.jetson.memory['RAM']['used'] /
                                  self.jetson.memory['RAM']['tot']) * 100,
                'temp_cpu': self.jetson.temperature.get('CPU', 0),
                'temp_gpu': self.jetson.temperature.get('GPU', 0),
                'temp_aux': self.jetson.temperature.get('AUX', 0),
                'power_total': self.jetson.power.get('tot', {}).get('power', 0),
                'gpu_freq': self.jetson.gpu.get('frq', 0)
            }
        except Exception as e:
            self.get_logger().warn(f'Error reading Jetson stats: {e}')
            return {'status': 'error'}
    def publish_metrics(self):
        """Collect and publish Jetson metrics"""
        metrics = self.get_jetson_metrics()
        # Log occasionally for debugging
        if hasattr(self, '_counter'):
            self._counter += 1
        else:
            self._counter = 0
        if self._counter % 10 == 0:  # Log every 10 seconds
            self.get_logger().info(
                f"GPU: {metrics.get('gpu_percent', 0):.1f}% | "
                f"RAM: {metrics.get('memory_percent', 0):.1f}% | "
                f"Temp CPU: {metrics.get('temp_cpu', 0):.1f}°C | "
                f"Temp GPU: {metrics.get('temp_gpu', 0):.1f}°C"
            )
        # Publish as JSON
        msg = String()
        msg.data = json.dumps(metrics)
        self.publisher.publish(msg)
    def __del__(self):
        """Clean up Jetson monitoring on shutdown"""
        if self.jetson:
            try:
                self.jetson.close()
                self.get_logger().info('Jetson monitoring stopped')
            except:
                pass
def main(args=None):
    rclpy.init(args=args)
    node = SystemMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
if __name__ == '__main__':
    main()