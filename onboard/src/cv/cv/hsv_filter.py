from abc import ABC, abstractmethod
from functools import reduce

import cv2
import numpy as np
import rclpy
from custom_msgs.msg import CVObject
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

from cv.config import MonoCam
from cv.utils import calculate_relative_pose, compute_center_distance, compute_yaw


class HSVFilter(Node, ABC):
    """Parent class for all HSV filtering scripts."""
    def __init__(self, name: str, camera: str, mask_ranges: np.ndarray, width: float, height: float | None = None,
                 pubs: list[str] | None = None, retrieval: int = cv2.RETR_TREE,
                 approx: int = cv2.CHAIN_APPROX_SIMPLE) -> None:
        super().__init__(f'{name}_hsv_filter')

        self.bridge = CvBridge()

        # Up to child classes to define
        self.mask_ranges = mask_ranges
        self.retrieval = retrieval
        self.approx = approx
        self.width = width  # Width of object in meters
        self.height = width if height is None else height  # Height of object in meters

        self.image_sub = self.create_subscription(CompressedImage, f'/camera/usb/{camera}/compressed',
                                                  self.image_callback, 10)

        self.hsv_filtered_pub = self.create_publisher(Image, f'/cv/{camera}_usb/{name}/hsv_filtered', 10)

        self.all_contours_pub = self.create_publisher(Image, f'/cv/{camera}_usb/{name}/all_contours', 10)

        self.bounding_box_pub = []
        self.contour_image_pub = []
        self.distance_pub = []

        self.pubs = pubs if pubs else [None]

        for pub in self.pubs:
            suffix = f'/{pub}' if pub is not None else ''
            base = f'/cv/{camera}_usb/{name}{suffix}'

            self.bounding_box_pub.append(self.create_publisher(CVObject, f'{base}/bounding_box', 10))
            self.contour_image_pub.append(self.create_publisher(Image, f'{base}/contour_image', 10))
            self.distance_pub.append(self.create_publisher(Point, f'{base}/distance', 10))

        self.create_additional_pubs_subs_vars()

    def create_additional_pubs_subs_vars(self) -> None:
        """Additional publishers and subscribers to be used later."""

    def actual_to_opencv_hsv(self, hsv_actual: np.ndarray) -> np.ndarray:
        """
        Convert actual HSV values to OpenCV HSV.

        Parameters:
            hsv_actual (np.ndarray): Array of shape (..., 3) with HSV values:
                                    H in [0, 360], S and V in [0, 100]

        Returns:
            np.ndarray: Converted HSV in OpenCV format:
                        H in [0,179], S and V in [0,255], same shape as input
        """
        hsv_opencv = np.empty_like(hsv_actual, dtype=np.uint8)
        hsv_opencv[..., 0] = (hsv_actual[..., 0] / 2).astype(np.uint8)          # Hue
        hsv_opencv[..., 1] = (hsv_actual[..., 1] / 100 * 255).astype(np.uint8)  # Saturation
        hsv_opencv[..., 2] = (hsv_actual[..., 2] / 100 * 255).astype(np.uint8)  # Value
        return hsv_opencv

    def create_pub_group(self, msg_type, base: str, pubs: list[str] | None = None, qos: int = 10):
        """
        Create a group of publishers under base topic.

        - base: e.g. f'/cv/{camera}_usb/{name}/bounding_box'
        - pubs: list of suffix names or None => a single publisher at base
        Returns list of publishers (length == len(pubs or [None])).
        """
        pubs_list = pubs if pubs else [None]
        created = []
        for p in pubs_list:
            suffix = f'/{p}' if p is not None else ''
            topic = f'{base}{suffix}'
            created.append(self.create_publisher(msg_type, topic, qos))
        return created

    def handle_detections(self, final_contours: list[np.ndarray], image: np.ndarray, bbox_img: np.ndarray) -> None:
        """
        Handle contours.

        Default handler to iterate contours and publish:
        - contour image for each publisher (self.contour_image_pub)
        - CVObject bounding boxes and distances (self.bounding_box_pub, self.distance_pub)
        Children may override this to publish other messages (angles, compressed images, etc).
        """
        for i in range(min(len(final_contours), len(self.contour_image_pub))):
            contour = final_contours[i]
            if contour is None:
                continue
            rect = cv2.minAreaRect(contour)
            box = np.int0(cv2.boxPoints(rect))

            # publish contour visualization
            image_with_contours = image.copy()
            cv2.drawContours(image_with_contours, [box], 0, (0, 0, 255), 3)
            self.contour_image_pub[i].publish(self.bridge.cv2_to_imgmsg(image_with_contours, 'bgr8'))

            # compute rectangle center/size
            rect_center = rect[0]
            x_f, y_f = rect_center[0], rect_center[1]
            w_f, h_f = rect[1][0], rect[1][1]

            if w_f <= 0 or h_f <= 0:
                continue

            meters_per_pixel = self.width / w_f

            # create basic CVObject (same fields your parent currently fills)
            bounding_box = CVObject()
            sec, nsec = self.get_clock().now().seconds_nanoseconds()
            bounding_box.header.stamp.sec = sec
            bounding_box.header.stamp.nanosec = nsec

            bounding_box.xmin = x_f * meters_per_pixel
            bounding_box.ymin = y_f * meters_per_pixel
            bounding_box.xmax = (x_f + w_f) * meters_per_pixel
            bounding_box.ymax = (y_f + h_f) * meters_per_pixel
            bounding_box.score = cv2.contourArea(contour)
            final_x_normalized = x_f / MonoCam.IMG_SHAPE[0]
            bounding_box.yaw = compute_yaw(final_x_normalized, final_x_normalized, MonoCam.IMG_SHAPE[0])
            bounding_box.width, bounding_box.height = int(w_f), int(h_f)

            dist_x, dist_y = compute_center_distance(x_f, y_f, *MonoCam.IMG_SHAPE,
                                                     height_adjustment_constant=15,
                                                     width_adjustment_constant=10)
            dist_point = Point()
            dist_point.x = dist_x
            dist_point.y = -dist_y

            self.bounding_box_pub[i].publish(bounding_box)
            self.distance_pub[i].publish(dist_point)

            # draw bbox on image used for all_contours_pub
            cv2.rectangle(bbox_img, (int(x_f), int(y_f)), (int(x_f + w_f), int(y_f + h_f)), (0, 255, 0), 2)

        # publish the stitched image with all bboxes
        self.all_contours_pub.publish(self.bridge.cv2_to_imgmsg(bbox_img, 'bgr8'))

    @abstractmethod
    def filter(self, contours: list[np.ndarray]) -> list[np.ndarray]:
        """Filter out list of contours."""
        return contours

    @abstractmethod
    def morphology(self, mask: np.ndarray) -> np.ndarray:
        """Apply morphology to mask."""
        return mask

def main(args: list[str] | None = None) -> None:
    """DO NOT RUN this node."""
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
