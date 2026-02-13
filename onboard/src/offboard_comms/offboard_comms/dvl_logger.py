import rclpy
from rclpy.node import Node
from custom_msgs.msg import DVLRaw
from pathlib import Path


class DVLLogger(Node):
    """Logs timestamps for DVL reading analysis. Writes files in the dvl_logs subdirectory."""
    def __init__(self) -> None:

        super().__init__('dvl_logger')

        self.get_logger().info('DVL Logger node started.')

        filename = Path('~/robosub-ros2/dvl_timestamps.txt').expanduser()
        if filename.exists:
            self.log_file = filename.open('a')
        else:
            self.log_file = filename.open('w')

        # Subscribe to image topic to get images
        self.image_sub = self.create_subscription(DVLRaw, '/sensors/dvl/raw', self.dvl_callback,
                                                   10)

    def dvl_callback(self, data: DVLRaw) -> None:
        """Write timestamps to file."""
        curtime = self.get_clock().now().seconds_nanoseconds()
        self.log_file.write(f'{curtime[0]}.{curtime[1]:09d}\n')
        self.log_file.flush()

def main(args: None = None) -> None:
    """Run the node."""
    rclpy.init(args=args)
    dvl_logger = DVLLogger()
    try:
        rclpy.spin(dvl_logger)
    except KeyboardInterrupt:
        pass
    finally:
        dvl_logger.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
