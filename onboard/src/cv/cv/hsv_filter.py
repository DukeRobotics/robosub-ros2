import cv2
import numpy as np
import rclpy
from custom_msgs.msg import CVObject
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

from cv.config import Bins, MonoCam, USB_Camera
from cv.utils import calculate_relative_pose, compute_center_distance, compute_yaw

class HSVFilter(Node):
    """Quick maths."""
    def __init__(self, name: str, camera: USB_Camera, mask: np.array, filtering: callable) -> None:
        super().__init__('')

def main(args: list[str] | None = None) -> None:
    """Run the node."""
    rclpy.init(args=args)
    hsv_filter = HSVFilter()

    try:
        rclpy.spin(hsv_filter)
    except KeyboardInterrupt:
        pass
    finally:
        hsv_filter.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
