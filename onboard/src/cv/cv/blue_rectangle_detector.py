#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError
from custom_msgs.msg import RectInfo


class BlueRectangleDetector(Node):
    def __init__(self):
        super().__init__("blue_rectangle_detector")
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(CompressedImage, "/camera/usb_camera/compressed", self.image_callback, 10)
        self.angle_pub = self.create_publisher(Float64, "/cv/bottom/lane_marker_angle", 10)
        self.distance_pub = self.create_publisher(Float64, "/cv/bottom/lane_marker_dist", 10)
        self.detections_pub = self.create_publisher(CompressedImage, "/cv/bottom/detections/compressed", 10)
        self.rect_info_pub = self.create_publisher(RectInfo, "/cv/bottom/lane_marker", 10)

    def image_callback(self, data):
        try:
            # Convert the compressed ROS image to OpenCV format
            np_arr = np.frombuffer(data.data, np.uint8)
            frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

            # Process the frame to find the angle of the blue rectangle and draw the rectangle
            angle, distance, rect_info, processed_frame = self.get_angle_and_distance_of_rectangle(frame)
            if angle is not None:
                angle_pub = Float64()
                angle_pub.data = float(angle)
                self.angle_pub.publish(angle_pub)

            if distance is not None:
                distance_pub = Float64()
                distance_pub.data = float(distance)
                self.distance_pub.publish(distance_pub)

            if rect_info is not None:
                self.rect_info_pub.publish(rect_info)

            # Publish the processed frame with the rectangle drawn
            try:
                compressed_image_msg = self.bridge.cv2_to_compressed_imgmsg(processed_frame)
                self.detections_pub.publish(compressed_image_msg)
            except CvBridgeError as e:
                self.get_logger().error(f"Failed to publish processed image: {e}")

        except CvBridgeError as e:
            self.get_logger().error(f"Could not convert image: {e}")

    def get_angle_and_distance_of_rectangle(self, frame):
        # Convert frame to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define range for blue color and create mask
        lower_blue = np.array([100, 150, 50])
        upper_blue = np.array([140, 255, 255])
        mask = cv2.inRange(hsv, lower_blue, upper_blue)

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

            if angle == -90 or angle == 90:
                angle = 0

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
            rect_info.center_x = float(rect_center[0])
            rect_info.center_y = float(rect_center[1])
            rect_info.width = float(rect[1][0])
            rect_info.height = float(rect[1][1])
            rect_info.angle = float(angle)
        return angle, distance, rect_info, frame


def main(args=None):
    rclpy.init(args=args)
    blue_rectangle_detector = BlueRectangleDetector()

    try:
        rclpy.spin(blue_rectangle_detector)
    except KeyboardInterrupt:
        pass
    finally:
        blue_rectangle_detector.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()