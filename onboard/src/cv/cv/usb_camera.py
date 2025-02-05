import contextlib

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

from cv.image_tools import ImageTools


class USBCamera(Node):
    """
    Object to stream any camera at /dev/video* and publishes the image feed at the device framerate.

    Currently used for the deepwater exploration usb mono cameras.

    Launch using: roslaunch cv usb_camera.launch.
    :param topic: rostopic to publish the image feed to; default is set to camera/usb_camera/compressed
    :param device_path: path to device to read the stream from (e.g., /dev/video0); can be a symlinked path
    :param framerate: custom framerate to stream the camera at; default is set to device default
    """

    def __init__(self, topic: str | None = None, device_path: str | None = None, framerate: int | None = None) -> None:
        # Instantiate new USB camera node
        super().__init__(f'usb_camera_{topic}')

        # Read custom camera configs from launch command
        self.topic = topic if topic else self.declare_parameter('topic', '/camera/usb_camera/compressed').value

        self.device_path = device_path if device_path else self.declare_parameter('device_path',
                                                                                  '/dev/video_front').value

        # If no custom framerate is passed in, set self.framerate to None to trigger default framerate
        self.framerate = framerate if framerate else self.declare_parameter('framerate', -1).value

        if self.framerate == -1:
            self.framerate = None

        # Create image publisher at given topic
        self.publisher = self.create_publisher(CompressedImage, self.topic, 10)

        self.image_tools = ImageTools()
        with contextlib.suppress(Exception):
            self.run()

    def run(self) -> None:
        """
        Connect to camera found at self.device_path using cv2.VideoCaptures.

        Stream every image as it comes in at the device framerate.
        """
        total_tries = 5
        success = False

        for _ in range(total_tries):
            if not rclpy.ok():
                break

            # Try connecting to the camera unless a connection is refused
            try:
                # Connect to camera at device_path
                self.get_logger().info('connecting to camera at '+ str(self.device_path))
                cap = cv2.VideoCapture(self.device_path, apiPreference=cv2.CAP_V4L2)
                self.get_logger().info('autodetected ' + str(cap.getBackendName()))

                # Read first frame
                success, img = cap.read()
                self.get_logger().info('First frame read ' + str(success))

                # Set publisher rate (framerate) to custom framerate if specified, otherwise, set to default
                if self.framerate is None:
                    self.framerate = cap.get(cv2.CAP_PROP_FPS)

                # If the execution reaches the following statement, and the first frame was successfully read,
                # then a successful camera connection was made, and we enter the main while loop
                if success:
                    break
            except Exception:  # noqa: BLE001
                self.get_looged().info('Failed to connect to USB camera, trying again...')

            if not rclpy.ok():
                break

            # Wait two seconds before trying again
            # This ensures the script does not terminate if the camera is just temporarily unavailable
            rclpy.spin_once(self, timeout_sec=2)

        if success:
            # Including 'not rospy.is_shutdown()' in the loop condition here to ensure if this script is exited
            # while this loop is running, the script quits without escalating to SIGTERM or SIGKILL
            while rclpy.ok():
                if success:
                    # Convert image read from cv2.videoCapture to image message to be published
                    image_msg = self.image_tools.convert_to_ros_compressed_msg(img)  # Compress image
                    # Publish the image
                    self.publisher.publish(image_msg)

                # Read next image
                success, img = cap.read()
                # Sleep loop to maintain frame rate
                rclpy.spin_once(self, timeout_sec=self.framerate)
        else:
            self.get_logger().error(f'{total_tries} attempts were made to connect to the USB camera. '
                         f'The camera was not found at device_path {self.device_path}. All attempts failed.')
            error_msg = (f'{total_tries} attempts were made to connect to the USB camera.\n'
                         f'The camera was not found at device_path {self.device_path}. All attempts failed.')
            raise RuntimeError(error_msg)

def main(args: list[str] | None = None) -> None:
    """Define main function."""
    rclpy.init(args=args)
    usb_camera = USBCamera()
    try:
        rclpy.spin(usb_camera)
    except KeyboardInterrupt:
        pass
    finally:
        usb_camera.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
