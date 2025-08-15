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


class HSVFilter(Node):
    """Parent class for all HSV filtering scripts."""
    def __init__(self, name: str, camera: str, mask_ranges: np.ndarray, width: int, retrieval: int = cv2.RETR_TREE,
                    approx: int = cv2.CHAIN_APPROX_SIMPLE) -> None:
        super().__init__(f'{name}_hsv_filter')

        self.bridge = CvBridge()

        # Up to child classes to define
        self.mask_ranges = mask_ranges
        self.retrieval = retrieval
        self.approx = approx
        self.width = width

        # Subscribers and publishers
        self.image_sub = self.create_subscription(CompressedImage, f'/camera/usb/{camera}/compressed',
                                                  self.image_callback, 10)
        self.bounding_box_pub = self.create_publisher(CVObject,
                                                            f'/cv/{camera}_usb/{name}/bounding_box1', 10)
        self.hsv_filtered_pub = self.create_publisher(Image, f'/cv/{camera}_usb/{name}/hsv_filtered1', 10)
        self.contour_image_pub = self.create_publisher(Image, f'/cv/{camera}_usb/{name}/contour_image1', 10)
        self.distance_pub = self.create_publisher(Point, f'/cv/{camera}_usb/{name}/distance1', 10)

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

    def image_callback(self, data: CompressedImage) -> None:
        """Attemp to convert image and apply contours."""
        try:
            # Convert the image from the compressed format to OpenCV format
            np_arr = np.frombuffer(data.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        except (TypeError, AttributeError) as t:
            self.get_logger().error(f'Failed to convert image: {t}')

        # Apply HSV filtering on the image
        masks = [cv2.inRange(hsv_image, self.actual_to_opencv_hsv(r[0]), self.actual_to_opencv_hsv(r[1])) for r in self.mask_ranges]
        mask = reduce(cv2.bitwise_or, masks)

        # Apply morphological filters as necessary to clean up binary image
        final_hsv = self.morphology(mask)

        # Publish the HSV filtered image
        hsv_filtered_msg = self.bridge.cv2_to_imgmsg(final_hsv, 'mono8')
        self.hsv_filtered_pub.publish(hsv_filtered_msg)

        # Find contours in the image
        contours, _ = cv2.findContours(final_hsv, self.retrieval, self.approx)

        if not contours:
            return

        # Filter contours as desired
        final_contour = self.filter(contours)

        # Allow filter function to determine that a contour set is invalid, even if detections exist
        if final_contour is None:
            return

        # Combine all contours to form the large rectangle
        all_points = np.vstack(final_contour)

        # Get the minimum area rectangle that encloses the combined contour
        rect = cv2.minAreaRect(all_points)

        # Draw contours onto image and publish
        image_with_contours = image.copy()
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(image_with_contours, [box], 0, (0, 0, 255), 3)
        contour_image_msg = self.bridge.cv2_to_imgmsg(image_with_contours, 'bgr8')
        self.contour_image_pub.publish(contour_image_msg)

        # Obtain the center of the rectangle
        rect_center = rect[0]
        x, y, w, h = (rect_center[0], rect_center[1], rect[1][0], rect[1][1])

        # Check if width is 0, and return base case of None
        if (w == 0):
            return

        # Get dimensions, attributes of relevant shapes
        meters_per_pixel = self.width / w

        # Create CVObject message, and populate relavent attributes
        bounding_box = CVObject()

        bounding_box.header.stamp.sec, bounding_box.header.stamp.nanosec = self.get_clock().now().seconds_nanoseconds()

        # Get dimensions that CVObject wants for our rectangle
        bounding_box.xmin = (x) * meters_per_pixel
        bounding_box.ymin = (y) * meters_per_pixel
        bounding_box.xmax = (x + w) * meters_per_pixel
        bounding_box.ymax = (y + h) * meters_per_pixel
        bounding_box.score = cv2.contourArea(final_contour)

        final_x_normalized = x / MonoCam.IMG_SHAPE[0]
        bounding_box.yaw = compute_yaw(final_x_normalized, final_x_normalized, MonoCam.IMG_SHAPE[0])  # width of camera in in mm
        #self.get_logger().info(f'yaw thoughts {bounding_box.yaw}')

        bounding_box.width = int(w)
        bounding_box.height = int(h)

        # Compute distance between center of bounding box and center of image
        # Here, image x is robot's y, and image y is robot's z
        dist_x, dist_y = compute_center_distance(x, y, *MonoCam.IMG_SHAPE, height_adjustment_constant=15,
                                                 width_adjustment_constant=10)

        # Create Point message and populate x and y distances
        dist_point = Point()
        dist_point.x = dist_x
        dist_point.y = -dist_y

        bbox_bounds = (x / MonoCam.IMG_SHAPE[0], y / MonoCam.IMG_SHAPE[1], (x+w) /
                       MonoCam.IMG_SHAPE[0], (y+h) / MonoCam.IMG_SHAPE[1])

        # Point coords represents the 3D position of the object represented by the bounding box relative to the robot
        coords_list = calculate_relative_pose(bbox_bounds,
                                              MonoCam.IMG_SHAPE,
                                              (self.width, 0),
                                              MonoCam.FOCAL_LENGTH,
                                              MonoCam.SENSOR_SIZE, 1)
        bounding_box.coords.x, bounding_box.coords.y, bounding_box.coords.z = coords_list

        self.bounding_box_pub.publish(bounding_box)

        self.distance_pub.publish(dist_point)

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
