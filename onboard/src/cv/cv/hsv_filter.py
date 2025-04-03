from functools import reduce

import cv2
import numpy as np
import rclpy
from custom_msgs.msg import CVObject
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

from cv.config import MonoCam, USBCamera
from cv.utils import compute_yaw


class HSVFilter(Node):
    """Parent class for all HSV filtering scripts."""
    def __init__(self, name: str, camera: str, mask_ranges: np.ndarray, retrieval: int = cv2.RETR_TREE,
                    approx: int = cv2.CHAIN_APPROX_SIMPLE) -> None:
        super().__init__(f'{name}_hsv_filter')

        set_params(self)

        self.bridge = CvBridge()

        # Up to child classes to define
        self.mask_ranges = mask_ranges
        self.retrieval = retrieval
        self.approx = approx

        # Subscribers and publishers
        self.image_sub = self.create_subscription(CompressedImage, f'/camera/usb/{camera}/compressed',
                                                  self.image_callback, 10)
        self.bounding_box_pub = self.create_publisher(list,
                                                            f'/cv/{camera}_usb/{name}/upper/bounding_box', 1)
        self.hsv_filtered_pub = self.create_publisher(Image, f'/cv/{camera}_usb/{name}/hsv_filtered', 1)
        self.contour_image_pub = self.create_publisher(Image, f'/cv/{camera}_usb/{name}/contour_image', 1)
        self.contour_image_with_bbox_pub = self.create_publisher(Image, f'/cv/{camera}_usb/{name}/detections', 1)

    def image_callback(self, data: CompressedImage) -> None:
        """Attemp to convert image and apply contours."""
        try:
            # Convert the image from the compressed format to OpenCV format
            np_arr = np.frombuffer(data.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        except (TypeError, AttributeError) as t:
            self.get_logger().error(f'Failed to convert image: {t}')

        masks = [None] * len(self.mask_ranges)

        # Apply HSV filtering on the image
        for r in self.mask_ranges:
            masks.append(cv2.inRange(hsv_image, r[0], r[1]))

        mask = reduce(cv2.bitwise_or, masks)

        # Apply morphological filters as necessary to clean up binary image
        final_hsv = self.morphology(mask)

        # Publish the HSV filtered image
        hsv_filtered_msg = self.bridge.cv2_to_imgmsg(final_hsv, 'mono8')
        self.hsv_filtered_pub.publish(hsv_filtered_msg)

        # Find contours in the image
        contours, _ = cv2.findContours(final_hsv, self.retrieval, self.approx)

        # Filter contours as desired
        final_contours = self.filter(contours)

        image_with_contours = image.copy()
        cv2.drawContours(image_with_contours, contours, -1, (255, 0, 0), 2)
        contour_image_msg = self.bridge.cv2_to_imgmsg(image_with_contours, 'bgr8')
        self.contour_image_pub.publish(contour_image_msg)

        bboxes = [None] * len(final_contours)

        for i in range(len(final_contours)):
            contour = final_contours[i]

            x, y, w, h = cv2.boundingRect(contour)

            # Create CVObject message, and populate relavent attributes
            bbox = CVObject()

            bbox.header.stamp.sec, bbox.header.stamp.nanosec = self.get_clock().now().seconds_nanoseconds()

            # Get dimensions that CVObject wants for our rectangle
            bbox.xmin = (x)
            bbox.ymin = (y)
            bbox.xmax = (x + w)
            bbox.ymax = (y + h)

            bbox.yaw = compute_yaw(x, x + w, MonoCam.SENSOR_SIZE[0])  # width of camera in in mm

            bbox.width = int(w)
            bbox.height = int(h)

            bboxes[i] = bbox

            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Publish list of bboxes
        self.bounding_box_pub.publish(bboxes)

        # Convert the image with the bounding box to ROS Image message and publish
        image_msg = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        self.contour_image_with_bbox_pub.publish(image_msg)

    def filter(self, contours: list) -> list:
        """Filter out list of contours."""
        return contours

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
