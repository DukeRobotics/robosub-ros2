from functools import reduce

import cv2
import numpy as np
import ros_numpy
import rospy

import rclpy
from custom_msgs.msg import CVObject
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

from cv.config import MonoCam
from cv.utils import calculate_relative_pose, compute_center_distance, compute_yaw

from ultralytics import YOLO

detection_model = YOLO("yolo26m.pt")
segmentation_model = YOLO("yolo26m-seg.pt")

# https://docs.ultralytics.com/guides/ros-quickstart/ this is used as a basis for everything

class Yolo(Node):

    def __init__(self, name: str, camera: str, mask_ranges: np.ndarray, width: float, height: float | None = None,
                 pubs: list[str] | None = None, retrieval: int = cv2.RETR_TREE,
                 approx: int = cv2.CHAIN_APPROX_SIMPLE) -> None:
        super().__init__(f'{name}_hsv_filter')

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(CompressedImage, f'/camera/usb/{camera}/compressed',
                                                  self.image_callback, 10)
        if pubs is None:
            self.det_image_pub = [self.create_publisher(Image, f'/cv/{camera}_usb/{name}/distance', 10)]
            self.seg_image_pub = [self.create_publisher(Image, f'/cv/{camera}_usb/{name}/distance', 10)]

        else:
            self.det_image_pub = []
            self.seg_image_pub = []

            for pub in pubs:
                self.det_image_pub = [self.create_publisher(Image, f'/cv/{camera}_usb/{name}/{pub}/distance', 10)]
                self.seg_image_pub = [self.create_publisher(Image, f'/cv/{camera}_usb/{name}/{pub}distance', 10)]

        self.create_additional_pubs_subs_vars()

    def image_callback(self, data: CompressedImage) -> None:
        array = ros_numpy.numpify(data)
        if self.det_image_pub.get_num_connections():
            det_result = detection_model(array)
            det_annotated = det_result[0].plot(show=False)
            self.det_image_pub.publish(ros_numpy.msgify(Image, det_annotated, encoding="rgb8"))

        if self.seg_image_pub.get_num_connections():
            seg_result = segmentation_model(array)
            seg_annotated = seg_result[0].plot(show=False)
            self.seg_image_pub.publish(ros_numpy.msgify(Image, seg_annotated, encoding="rgb8"))


    def create_additional_pubs_subs_vars(self) -> None:
        """Additional publishers and subscribers to be used later."""

def main(args: list[str] | None = None) -> None:
    """DO NOT RUN this node."""
    rclpy.init(args=args)
    yolo_detection = Yolo()

    try:
        rclpy.spin(yolo_detection)
    except KeyboardInterrupt:
        pass
    finally:
        yolo_detection.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
