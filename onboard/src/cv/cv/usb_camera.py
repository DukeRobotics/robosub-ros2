import os
from pathlib import Path

import cv2
import rclpy
import resource_retriever as rr
import yaml
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

from cv.image_tools import ImageTools

CV_CONFIG_PATH = f'package://cv/config/{os.getenv('ROBOT_NAME')}.yaml'


class USBCamera(Node):
    """Publish image feed from a USB camera."""

    def __init__(self) -> None:
        """Initialize USB camera node."""
        super().__init__('usb_camera')

        self.camera = self.declare_parameter('camera', 'front').value

        # If framerate is -1, use camera's default framerate
        self.framerate = self.declare_parameter('framerate', -1).get_parameter_value().integer_value

        with Path(rr.get_filename(CV_CONFIG_PATH, use_protocol=False)).open() as f:
            cv_config = yaml.safe_load(f)
            usb_cameras = cv_config['usb_cameras']

        if self.camera not in usb_cameras:
            error_msg = f'Camera {self.camera} not found in CV config file.'
            raise ValueError(error_msg)

        camera_config = usb_cameras[self.camera]
        self.device_path = camera_config['device_path']
        self.topic = camera_config['topic']

        self.connect()

        self.image_tools = ImageTools()
        self.publisher = self.create_publisher(CompressedImage, self.topic, 10)
        self.publish_timer = self.create_timer(1.0 / self.framerate, self.publish_frame)

    def connect(self) -> None:
        """
        Connect to camera using cv2.VideoCapture.

        This function will return if the camera is connected successfully, otherwise it will raise an error. If
        self.framerate is -1, it will set self.framerate to the camera's default framerate.
        """
        total_tries = 5
        success = False

        for i in range(total_tries):
            if not rclpy.ok():
                return

            # Try connecting to the camera unless a connection is refused
            try:
                # Connect to camera at device_path
                self.get_logger().info(f'Attempt {i + 1} to connect to USB camera {self.camera}.')
                self.cap = cv2.VideoCapture(self.device_path, apiPreference=cv2.CAP_V4L2)

                self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

                # Read first frame
                success, _ = self.cap.read()
                if success:
                    self.get_logger().info(f'Received frame from USB camera {self.camera}.')

                    # Set publisher rate (framerate) to custom framerate if specified, otherwise, set to default
                    if self.framerate == -1:
                        self.framerate = self.cap.get(cv2.CAP_PROP_FPS)

                    # Successfully connected to camera
                    return

            except:  # noqa: E722
                self.get_logger().info(f'Failed to connect to USB camera {self.camera}, trying again...')

            if not rclpy.ok():
                return

            # Wait two seconds before trying again
            # This ensures the script does not terminate if the camera is just temporarily unavailable
            rclpy.spin_once(self, timeout_sec=2)

        # If function did not return in the for loop, then it failed to connect to the camera
        error_msg = (f'{total_tries} attempts were made to connect to the USB camera {self.camera} at '
                        f'{self.device_path}. All attempts failed.')
        raise RuntimeError(error_msg)

    def publish_frame(self) -> None:
        """Get one frame from the camera and publish it."""
        success, img = self.cap.read()
        if success:
            # Convert image read from cv2.videoCapture to image message to be published
            image_msg = self.image_tools.convert_to_ros_compressed_msg(img)  # Compress image
            # Publish the image
            self.publisher.publish(image_msg)


def main(args: list[str] | None = None) -> None:
    """Run the USB camera node."""
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
