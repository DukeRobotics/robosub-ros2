import cv.config as cv_constants
import cv2
import numpy as np
import rclpy
from cv.hsv_filter import HSVFilter
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float64



class HSVLaneMarker(HSVFilter):
    """HSV Lane Marker Detector."""
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
        # TODO: implement this to add publishers from lane_marker_detector.py that are missing in the default HSVFilter
        self.detections_pub = self.create_publisher(CompressedImage, '/cv/bottom/detections/compressed', 10)
        self.angle_pub = self.create_publisher(Float64, '/cv/bottom/lane_marker/angle', 10)

    def process_contours(self, final_contours: list[np.ndarray], image: np.ndarray, bbox_img: np.ndarray) -> None:
        """Process lane marker contour."""
        # TODO: implement this to match behavior of lane_marker_detector.py perfectly
        contours = final_contours
        angle_in_degrees = None
        distance = None
        bounding_box = None
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

            angle_in_degrees = rect[-1]

            # Compare the y-coordinates
            if right_top[1] < left_top[1]:
                # Right side is higher than left side
                angle_in_degrees = rect[-1] - 90

            if angle_in_degrees in {-90, 90}:
                angle_in_degrees = 0.0

            # Calculate the center of the rectangle
            rect_center = rect[0]

            # Calculate the center of the frame
            frame_center = (frame.shape[1] / 2, frame.shape[0] / 2)

            # Compute distance between center of bounding box and center of image
            # Here, image x is robot's y, and image y is robot's z
            distance = Point()
            distance.x = rect_center[0] - frame_center[0]
            distance.y = frame_center[1] - rect_center[1]

            # Create CVObject message
            bounding_box = CVObject()
            bounding_box.header.stamp = self.get_clock().now().to_msg()
            bounding_box.coords = Point()
            bounding_box.coords.x = rect_center[0]
            bounding_box.coords.y = rect_center[1]
            bounding_box.width = rect[1][0]
            bounding_box.height = rect[1][1]
            bounding_box.yaw = math.radians(angle_in_degrees)


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
