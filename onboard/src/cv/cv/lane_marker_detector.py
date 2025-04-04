import cv2
import numpy as np
import rclpy
from custom_msgs.msg import RectInfo
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64

from cv.config import LaneMarker


class LaneMarkerDetector(Node):
    """Detect lane marker in Taishoff Aquatics Pavillion."""
    def __init__(self) -> None:
        """Initialize node."""
        super().__init__('lane_marker_detector')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(CompressedImage, '/camera/usb/bottom/compressed', self.image_callback,
                                                   10)
        self.angle_pub = self.create_publisher(Float64, '/cv/bottom/lane_marker/angle', 10)
        self.distance_pub = self.create_publisher(Point, '/cv/bottom/lane_marker/distance', 10)
        self.detections_pub = self.create_publisher(CompressedImage, '/cv/bottom/detections/compressed', 10)
        self.rect_info_pub = self.create_publisher(RectInfo, '/cv/bottom/lane_marker', 10)

    def image_callback(self, data: CompressedImage) -> None:
        """Process image and, if proper, draws rectangle and publishes image."""
        try:
            # Convert the compressed ROS image to OpenCV format
            np_arr = np.frombuffer(data.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # Process the frame to find the angle of the lane marker and draw a rectangle around it
            angle, distance, rect_info, processed_frame = self.get_angle_and_distance_of_rectangle(frame)
            if angle is not None:
                angle_msg = Float64()
                angle_msg.data = angle
                self.angle_pub.publish(angle_msg)

            if distance is not None:
                distance_msg = Point()
                distance_msg.y = distance
                self.distance_pub.publish(distance_msg)

            if rect_info is not None:
                self.rect_info_pub.publish(rect_info)

            # Publish the processed frame with the rectangle drawn
            try:
                compressed_image_msg = self.bridge.cv2_to_compressed_imgmsg(processed_frame)
                self.detections_pub.publish(compressed_image_msg)
            except CvBridgeError as e:
                self.get_logger().error(f'Failed to publish processed image: {e}')

        except CvBridgeError as e:
            self.get_logger().error(f'Could not convert image: {e}')

    def get_angle_and_distance_of_rectangle(self, frame: np.array) -> tuple[float, int, RectInfo, np.array]:
        """Get angle and distance of rectangle contour."""
        # Convert frame to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define range for blue color of lane marker and create mask
        mask = cv2.inRange(hsv, LaneMarker.LANE_MARKER_BOT, LaneMarker.LANE_MARKER_TOP)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        angle = None
        distance = None
        rect_info = None
        if contours:
            # Combine all contours to form the large rectangle
            all_points = np.vstack(contours)

            # Get the minimum area rectangle that encloses the combined contour
            rect = cv2.minAreaRect(all_points)

            # Draw the rectangle on the frame
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            cv2.drawContours(frame, [box], 0, (0, 0, 255), 3)

            # Sort the points based on their x-coordinates to identify left and right sides
            box = sorted(box, key=lambda pt: pt[0])

            # Identify left and right side points
            left_pts = box[:2]
            right_pts = box[2:]

            # Determine which point is higher on the left side
            left_top = min(left_pts, key=lambda pt: pt[1])

            # Determine which point is higher on the right side
            right_top = min(right_pts, key=lambda pt: pt[1])

            angle = rect[-1]

            # Compare the y-coordinates
            if right_top[1] < left_top[1]:
                # Right side is higher than left side
                angle = rect[-1] - 90

            if angle in (-90, 90): # this is NOT standard set notation (as in mathematics); this is a tuple
                angle = 0.0

            # Calculate the center of the rectangle
            rect_center = rect[0]

            # Calculate the center of the frame
            frame_center = (frame.shape[1] / 2, frame.shape[0] / 2)

            # Calculate the vertical distance between the two centers
            vertical_distance = rect_center[1] - frame_center[1]

            # The distance is positive if the rectangle is below the centerline, negative if above
            distance = -vertical_distance

            # Create RectInfo message
            rect_info = RectInfo()
            rect_info.center_x = rect_center[0]
            rect_info.center_y = rect_center[1]
            rect_info.width = rect[1][0]
            rect_info.height = rect[1][1]
            rect_info.angle = angle

        return angle, distance, rect_info, frame


def main(args: None = None) -> None:
    """Start the node."""
    rclpy.init(args=args)
    lane_marker_detector = LaneMarkerDetector()

    try:
        rclpy.spin(lane_marker_detector)
    except KeyboardInterrupt:
        pass
    finally:
        lane_marker_detector.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
