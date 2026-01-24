from onboard.src.cv.cv import bin_detector
import rclpy
from custom_msgs.msg import DVLRaw


class DVLLogger(Node):
    """Logs timestamps for DVL reading analysis. Writes files in the dvl_logs subdirectory."""
    def __init__(self) -> None:

        super().__init__('dvl_logger')

        self.get_logger().info('DVL Logger node started.')
        self.log_file = open('dvl_logs/dvl_timestamps.txt', 'a')
        # Subscribe to image topic to get images
        self.image_sub = self.create_subscription(DVLRaw, '/sensors/dvl/raw', self.dvl_callback,
                                                   10)

    def dvl_callback(self, data: DVLRaw) -> None:
        """Write timestamps to file."""
        timestamp = self.get_clock().now().seconds_nanoseconds()[1]
        self.log_file.write(f'{timestamp}\n')
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
