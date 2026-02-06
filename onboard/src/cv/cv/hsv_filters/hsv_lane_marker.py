import math

import cv2
import numpy as np
import rclpy
from custom_msgs.msg import CVObject
from cv_bridge import CvBridgeError
from geometry_msgs.msg import Point
from std_msgs.msg import Float64

import cv.config as cv_constants
from cv.hsv_filter import HSVFilter


class HSVLaneMarker(HSVFilter):
    """HSV Lane Marker Detector. Detects lane marker in Taishoff Aquatics Pavillion."""

    def __init__(self) -> None:
        super().__init__(
            name='lane_marker',
            camera='bottom',
            mask_ranges=[
                [cv_constants.LaneMarker.LANE_MARKER_BOT, cv_constants.LaneMarker.LANE_MARKER_TOP],
            ],
        )

    def create_additional_pubs_subs_vars(self) -> None:
        """Additional publishers and subscribers specific to this class."""
        self.angle_pubs = [self.create_publisher(Float64, '/cv/bottom_usb/lane_marker/angle', 10)]

    def process_contours(self, final_contours: list[np.ndarray], image: np.ndarray) -> None:
        """Process lane marker contour."""
        if not final_contours:
            self.get_logger().error('No contours found.')

        angle_in_degrees = None
        distance = None
        bounding_box = None

        # Combine all contours to form the large rectangle
        all_points = np.vstack(final_contours)

        # Get the minimum area rectangle that encloses the combined contour
        rect = cv2.minAreaRect(all_points)

        # Draw the rectangle on the frame
        image_with_contours = image.copy()
        box = np.int0(cv2.boxPoints(rect))
        cv2.drawContours(image_with_contours, [box], 0, (0, 0, 255), 3)
        self.contour_image_pubs[0].publish(self.bridge.cv2_to_compressed_imgmsg(image_with_contours, 'bgr8'))

        # Sort the points based on their x-coordinates to identify left and right sides
        box = sorted(box, key=lambda pt: pt[0])

        # Identify left and right side points
        left_pts = box[:2]
        right_pts = box[2:]

        # Determine which point is higher on the left side
        left_top = min(left_pts, key=lambda pt: pt[1])

        # Determine which point is higher on the right side
        right_top = min(right_pts, key=lambda pt: pt[1])

        angle_in_degrees = rect[-1]

        # Compare the y-coordinates
        if right_top[1] < left_top[1]:
            # Right side is higher than left side
            angle_in_degrees = rect[-1] - 90

        if angle_in_degrees in {-90, 90}:
            angle_in_degrees = 0.0

        angle_msg = Float64()
        angle_msg.data = angle_in_degrees
        self.angle_pubs[0].publish(angle_msg)

        # Calculate the center of the rectangle
        rect_center = rect[0]

        # Calculate the center of the frame
        frame_center = (image.shape[1] / 2, image.shape[0] / 2)

        # Compute distance between center of bounding box and center of image
        # Here, image x is robot's y, and image y is robot's z
        distance = Point()
        distance.x = rect_center[0] - frame_center[0]
        distance.y = frame_center[1] - rect_center[1]

        self.distance_pubs[0].publish(distance)

        # Create CVObject message
        bounding_box = CVObject()
        bounding_box.header.stamp = self.get_clock().now().to_msg()
        bounding_box.coords = Point()
        bounding_box.coords.x = rect_center[0]
        bounding_box.coords.y = rect_center[1]
        bounding_box.width = rect[1][0]
        bounding_box.height = rect[1][1]
        bounding_box.yaw = math.radians(angle_in_degrees)

        self.bounding_box_pubs[0].publish(bounding_box)

        # TODO: draw contour as a rotated rectangle onto a copy of the image (image.copy())
        # Reference https://github.com/DukeRobotics/robosub-ros/blob/master/onboard/catkin_ws/src/cv/scripts/path_marker_detector.py
        cv2.rectangle(bbox_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
        self.all_contours_pub.publish(self.bridge.cv2_to_imgmsg(bbox_img, 'bgr8'))

    def filter(self, contours: list) -> list:
        """Pick the largest contour onguly."""
        final_contours = sorted(contours, key=cv2.contourArea, reverse=True)
        return final_contours[0]

    def morphology(self, mask: np.ndarray) -> np.ndarray:
        """Apply a kernel morphology."""
        kernel = np.ones((5, 5), np.uint8)
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)


def main(args: list[str] | None = None) -> None:
    """Run the node."""
    rclpy.init(args=args)
    hsv_filter = HSVLaneMarker()

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
