import math

import cv2
import numpy as np
import rclpy
from custom_msgs.msg import CVObject
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

from cv.utils import compute_center_distance


class PathMarkerDetector(Node):
    """Detect path marker."""
    MONO_CAM_IMG_SHAPE = (640, 480)  # Width, height in pixels

    def __init__(self) -> None:
        """Init nodes."""
        super().__init__('path_marker_detector')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(CompressedImage, '/camera/usb/bottom/compressed',
                                                  self.image_callback, 10)

        # define information published for path marker
        self.path_marker_hsv_filtered_pub = self.create_publisher(Image, '/cv/bottom/path_marker/hsv_filtered', 10)
        self.path_marker_contour_image_pub = self.create_publisher(Image, '/cv/bottom/path_marker/contour_image', 10)
        self.path_marker_bounding_box_pub = self.create_publisher(CVObject, '/cv/bottom/path_marker/bounding_box', 10)
        self.path_marker_distance_pub = self.create_publisher(Point, '/cv/bottom/path_marker/distance', 10)

    def image_callback(self, data: CompressedImage) -> None:
        """Get and process image."""
        # Convert the compressed ROS image to OpenCV format
        np_arr = np.frombuffer(data.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        frame = frame[:, :-20]

        # Process the frame to find and publish information on the bin
        self.process_frame(frame)

    def process_frame(self, frame: np.array) -> None:
        """Process frame."""
        # Convert frame to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define range for blue color and create mask
        lower_orange = np.array([0, 130, 100])
        upper_orange = np.array([20, 255, 255])
        mask = cv2.inRange(hsv, lower_orange, upper_orange)

        hsv_filtered_msg = self.bridge.cv2_to_imgmsg(mask, 'mono8')
        self.path_marker_hsv_filtered_pub.publish(hsv_filtered_msg)

        # Apply morphological operations to clean up the binary image
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Find contours in the mask
        MIN_CONTOUR_LENGTH = 5  # noqa: N806
        MIN_CONTOUR_AREA = 500  # noqa: N806

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours = [contour for contour in contours if len(contour) > MIN_CONTOUR_LENGTH and cv2.contourArea(contour) >
                    MIN_CONTOUR_AREA]

        # Fit a line to the largest contour
        if len(contours) > 0:
            largest_contour = max(contours, key=cv2.contourArea)
            center, dimensions, orientation = cv2.fitEllipse(largest_contour)

            # NOTE: CLOCKWISE yaw from 12 o'clock
            orientation_in_radians = math.radians(orientation)

            # create CVObject message and populate relevant attributes
            bounding_box = CVObject()

            bounding_box.header.stamp = self.get_clock().now().to_msg()

            orientation_in_radians = math.pi / 2 - orientation_in_radians

            bounding_box.yaw = orientation_in_radians

            bounding_box.width = int(dimensions[0])
            bounding_box.height = int(dimensions[1])

            # Compute distance between center of bounding box and center of image
            # Here, image x is robot's y, and image y is robot's z
            dist_x, dist_y = compute_center_distance(center[0], center[1], *self.MONO_CAM_IMG_SHAPE)

            # get distances into publishable format
            # also fix axis from camera's POV
            dist_point = Point()
            dist_point.x = dist_x
            dist_point.y = -dist_y

            # publish dist, bbox
            self.path_marker_distance_pub.publish(dist_point)
            self.path_marker_bounding_box_pub.publish(bounding_box)

            visualized_frame = self.visualize_path_marker_detection(frame, center, bounding_box,
                                                                    math.pi / 2 - orientation_in_radians)
            cv2.circle(visualized_frame, (int(center[0]), int(center[1])), 5, (0, 0, 255), -1)
            self.path_marker_contour_image_pub.publish(self.bridge.cv2_to_imgmsg(visualized_frame))

    def visualize_path_marker_detection(self, frame: np.array, center: tuple, bounding_box: CVObject,
                                        orientation: float) -> np.array:
        """Return frame with bounding boxes of the detection."""
        frame_copy = frame.copy()

        center_x, center_y = center
        width, height = bounding_box.width, bounding_box.height
        orientation = orientation if orientation > 0 else orientation + math.pi

        # Calculate the four corners of the rectangle
        angle_cos = math.cos(orientation)
        angle_sin = math.sin(orientation)
        half_width = width / 2
        half_height = height / 2

        x1 = int(center_x - half_width * angle_cos + half_height * angle_sin)
        y1 = int(center_y - half_width * angle_sin - half_height * angle_cos)
        x2 = int(center_x + half_width * angle_cos + half_height * angle_sin)
        y2 = int(center_y + half_width * angle_sin - half_height * angle_cos)
        x3 = int(center_x + half_width * angle_cos - half_height * angle_sin)
        y3 = int(center_y + half_width * angle_sin + half_height * angle_cos)
        x4 = int(center_x - half_width * angle_cos - half_height * angle_sin)
        y4 = int(center_y - half_width * angle_sin + half_height * angle_cos)

        # Draw the rotated rectangle
        cv2.line(frame_copy, (x1, y1), (x2, y2), (0, 0, 255), 3)
        cv2.line(frame_copy, (x2, y2), (x3, y3), (0, 0, 255), 3)
        cv2.line(frame_copy, (x3, y3), (x4, y4), (0, 0, 255), 3)
        cv2.line(frame_copy, (x4, y4), (x1, y1), (0, 0, 255), 3)

        return frame_copy


def main(args: None=None) -> None:
    """Start node."""
    rclpy.init(args=args)
    path_marker_detector = PathMarkerDetector()

    try:
        rclpy.spin(path_marker_detector)
    except KeyboardInterrupt:
        pass
    finally:
        path_marker_detector.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
