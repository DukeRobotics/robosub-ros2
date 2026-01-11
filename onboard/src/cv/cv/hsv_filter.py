from abc import ABC, abstractmethod
from functools import reduce

import cv2
import numpy as np
from custom_msgs.msg import CVObject
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

from cv.config import MonoCam
from cv.utils import calculate_relative_pose, compute_center_distance, compute_yaw


class HSVFilter(Node, ABC):
    """Parent class for all HSV filtering scripts."""
    def __init__(self, name: str, camera: str, mask_ranges: list[np.ndarray],
                 width: float | None = None, height: float | None = None,
                 pubs: list[str] | None = None, retrieval: int = cv2.RETR_TREE,
                 approx: int = cv2.CHAIN_APPROX_SIMPLE) -> None:
        """
        Initiate an HSV filtering script.

        # TODO: detailed docstring to explain each argument.
        """
        super().__init__(f'{name}_hsv_filter')

        self.bridge = CvBridge()

        # Up to child classes to define
        self.mask_ranges = mask_ranges
        self.retrieval = retrieval
        self.approx = approx
        # NOTE: detectors whose process_contours rely on width must have width defined,
        # else they must supply their own process_contour
        # TODO: is there any Pythonic systemic way to make this idiot proof
        self.width = width if width else 1  # Width of object in meters
        self.height = height if height else width  # Height of object in meters

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

    def image_callback(self, data: CompressedImage) -> None:
        """Attempt to convert image and apply contours."""
        try:
            # Convert the image from the compressed format to OpenCV format
            np_arr = np.frombuffer(data.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        except (TypeError, AttributeError) as t:
            self.get_logger().error(f'Failed to convert image: {t}')
            return

        # Apply HSV filtering on the image
        masks = [cv2.inRange(hsv_image, self.actual_to_opencv_hsv(r[0]),
                             self.actual_to_opencv_hsv(r[1])) for r in self.mask_ranges]
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
        final_contours = self.filter(contours)

        # Allow filter function to determine that a contour set is invalid, even if detections exist
        if final_contours == []:
            return

        bbox_img = image.copy()

        # Process contours
        self.process_contours(final_contours, image, bbox_img)

    def process_contours(self, final_contours: list[np.ndarray], image: np.ndarray, bbox_img: np.ndarray) -> None:
        """
        Handle contours.

        TODO: docstring.
        """
        for i in range(min(len(final_contours), len(self.contour_image_pub))):
            contour = final_contours[i]
            if contour is None:
                continue

            # Get the minimum area rectangle that encloses the combined contour
            rect = cv2.minAreaRect(contour)

            # Draw contours onto image and publish
            image_with_contours = image.copy()
            box = np.int0(cv2.boxPoints(rect))
            cv2.drawContours(image_with_contours, [box], 0, (0, 0, 255), 3)
            self.contour_image_pub[i].publish(self.bridge.cv2_to_imgmsg(image_with_contours, 'bgr8'))

            # Obtain the center of the rectangle
            rect_center = rect[0]
            x, y, w, h = (rect_center[0], rect_center[1], rect[1][0], rect[1][1])

            # Check if width is 0, and return base case of None
            if (w == 0 or h == 0):
                continue

            # Get dimensions, attributes of relevant shapes
            meters_per_pixel = self.width / w

            # Create CVObject message, and populate relevant attributes
            bounding_box = CVObject()
            sec, nsec = self.get_clock().now().seconds_nanoseconds()
            bounding_box.header.stamp.sec = sec
            bounding_box.header.stamp.nanosec = nsec

            # Get dimensions that CVObject wants for our rectangle
            bounding_box.xmin = (x) * meters_per_pixel
            bounding_box.ymin = (y) * meters_per_pixel
            bounding_box.xmax = (x + w) * meters_per_pixel
            bounding_box.ymax = (y + h) * meters_per_pixel
            bounding_box.score = cv2.contourArea(contour)

            final_x_normalized = x / MonoCam.IMG_SHAPE[0]
            bounding_box.yaw = compute_yaw(final_x_normalized, final_x_normalized, MonoCam.IMG_SHAPE[0])
            bounding_box.width, bounding_box.height = int(w), int(h)

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

            # Point coords represents the 3D position of the object represented by the bounding box relative to robot
            coords_list = calculate_relative_pose(bbox_bounds,
                                                  MonoCam.IMG_SHAPE,
                                                  (self.width, self.height),
                                                  MonoCam.FOCAL_LENGTH,
                                                  MonoCam.SENSOR_SIZE, 1)
            bounding_box.coords.x, bounding_box.coords.y, bounding_box.coords.z = coords_list

            self.bounding_box_pub[i].publish(bounding_box)
            self.distance_pub[i].publish(dist_point)

            # Draw bounding box on the image
            cv2.rectangle(bbox_img, (x, y), (x + w, y + h), (0, 255, 0), 2)

        self.all_contours_pub.publish(self.bridge.cv2_to_imgmsg(bbox_img, 'bgr8'))

    @abstractmethod
    def filter(self, contours: list[np.ndarray]) -> list[np.ndarray]:
        """Filter out list of contours."""
        return contours

    @abstractmethod
    def morphology(self, mask: np.ndarray) -> np.ndarray:
        """Apply morphology to mask."""
        return mask
