import cv2
import numpy as np
import rclpy
from custom_msgs.msg import CVObject
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image

from cv.config import Bins, MonoCam
from cv.utils import calculate_relative_pose, compute_center_distance, compute_yaw


class BinDetector(Node):
    """Detect bins with HSV filtering."""
    def __init__(self) -> None:

        super().__init__('bin_detector')
        self.bridge = CvBridge()
        # Subscribe to image topic to get images
        self.image_sub = self.create_subscription(CompressedImage, '/camera/usb/bottom/compressed', self.image_callback,
                                                   10)

        # Declare blue bin publishers
        self.blue_bin_hsv_filtered_pub = self.create_publisher(Image, '/cv/bottom/bin_blue/hsv_filtered', 10)
        self.blue_bin_contour_image_pub = self.create_publisher(Image, '/cv/bottom/bin_blue/contour_image', 10)
        self.blue_bin_bounding_box_pub = self.create_publisher(CVObject, '/cv/bottom/bin_blue/bounding_box', 10)
        self.blue_bin_distance_pub = self.create_publisher(Point, '/cv/bottom/bin_blue/distance', 10)

        # Declare red bin publishers
        self.red_bin_hsv_filtered_pub = self.create_publisher(Image, '/cv/bottom/bin_red/hsv_filtered', 10)
        self.red_bin_contour_image_pub = self.create_publisher(Image, '/cv/bottom/bin_red/contour_image', 10)
        self.red_bin_bounding_box_pub = self.create_publisher(CVObject, '/cv/bottom/bin_red/bounding_box', 10)
        self.red_bin_distance_pub = self.create_publisher(Point, '/cv/bottom/bin_red/distance', 10)

    def image_callback(self, data: CompressedImage) -> None:
        """Convert and process ROS image to OpenCV-accessible format."""
        # Convert the compressed ROS image to OpenCV format
        np_arr = np.frombuffer(data.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        frame = frame[:, :-20] # crops out last 20 pixels on the y-axis to fix resolution, robot camera placement issues
        # Process the frame to find and publish information on the bin
        self.process_frame(frame)

    def process_frame(self, frame: np.array) -> None:
        """Act on frames: filters, applies contours, and publishes results."""
        # Convert frame to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create mask
        mask_blue = cv2.inRange(hsv, Bins.LOWER_BLUE, Bins.UPPER_BLUE)
        blue_hsv_filtered_msg = self.bridge.cv2_to_imgmsg(mask_blue, 'mono8')
        self.blue_bin_hsv_filtered_pub.publish(blue_hsv_filtered_msg)

        # Apply HSV filtering on the image
        mask_red_low = cv2.inRange(hsv, Bins.RED_LOW_BOT, Bins.RED_LOW_TOP)
        mask_red_high = cv2.inRange(hsv, Bins.RED_HIGH_BOT, Bins.RED_HIGH_TOP)
        mask_red = cv2.bitwise_or(mask_red_low, mask_red_high)

        # Convert cv2 image to img message, and publish the image
        red_hsv_filtered_msg = self.bridge.cv2_to_imgmsg(mask_red, 'mono8')
        self.red_bin_hsv_filtered_pub.publish(red_hsv_filtered_msg)

        # Apply morphological operations to clean up the binary image
        kernel = np.ones((5, 5), np.uint8)
        mask_blue = cv2.morphologyEx(mask_blue, cv2.MORPH_OPEN, kernel)

        # Apply morphological operations to clean up the binary image
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)
        mask_red = cv2.morphologyEx(mask_red, cv2.MORPH_OPEN, kernel)

        # Find contours in the mask
        contours_blue, _ = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours_red, _ = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        MIN_AREA_OF_CONTOUR = 500  # noqa: N806

        if contours_blue:
            # Take largest contour
            contours_blue = sorted(contours_blue, key=cv2.contourArea, reverse=True)
            contours_blue = contours_blue[0]

            # Call process_contours for the blue contour
            bbox, image, dist = self.process_contours(frame.copy(), contours_blue)

            # Publish bbox, image, distance if the contour area is greater than MIN_AREA_OF_CONTOUR
            if bbox and image and dist and cv2.contourArea(contours_blue) > MIN_AREA_OF_CONTOUR:
                self.blue_bin_contour_image_pub.publish(image)
                self.blue_bin_bounding_box_pub.publish(bbox)
                self.blue_bin_distance_pub.publish(dist)

        if contours_red:
            # Take largest contour
            contours_red = sorted(contours_red, key=cv2.contourArea, reverse=True)
            contours_red = contours_red[0]

            # Call process_contours for the red contour
            bbox, image, dist = self.process_contours(frame.copy(), contours_red)

            # Publish bbox, image, distance if the contour area is greater than MIN_AREA_OF_CONTOUR
            if bbox and image and dist and cv2.contourArea(contours_red) > MIN_AREA_OF_CONTOUR:
                self.red_bin_contour_image_pub.publish(image)
                self.red_bin_bounding_box_pub.publish(bbox)
                self.red_bin_distance_pub.publish(dist)

    def process_contours(self, frame: np.array, contours: np.array) -> tuple[CVObject, Image, Point]:
        """Filter contours as needed."""
        # Combine all contours to form the large rectangle
        all_points = np.vstack(contours)

        # Get the minimum area rectangle that encloses the combined contour
        rect = cv2.minAreaRect(all_points)

        # Draw the rectangle on the frame
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        cv2.drawContours(frame, [box], 0, (0, 0, 255), 3)

        # Calculate the center of the rectangle
        rect_center = rect[0]

        x, y, w, h = (rect_center[0], rect_center[1], rect[1][0], rect[1][1])

        # Check if width is 0, and return base case of None
        if (w == 0):
            return None, None, None

        # Get dimensions, attributes of relevant shapes
        meters_per_pixel = Bins.WIDTH / w

        # Create CVObject message, and populate relavent attributes
        bounding_box = CVObject()

        bounding_box.header.stamp.sec, bounding_box.header.stamp.nanosec = self.get_clock().now().seconds_nanoseconds()

        # Get dimensions that CVObject wants for our rectangle
        bounding_box.xmin = (x) * meters_per_pixel
        bounding_box.ymin = (y) * meters_per_pixel
        bounding_box.xmax = (x + w) * meters_per_pixel
        bounding_box.ymax = (y + h) * meters_per_pixel

        bounding_box.yaw = compute_yaw(x, x + w, MonoCam.SENSOR_SIZE[0])  # width of camera in in mm

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
                                              (Bins.WIDTH, 0),
                                              MonoCam.FOCAL_LENGTH,
                                              MonoCam.SENSOR_SIZE, 1)
        bounding_box.coords.x, bounding_box.coords.y, bounding_box.coords.z = coords_list

        # Convert the image with the bounding box to ROS Image message and publish
        cv2.circle(frame, (int(x), int(y)), 5, (0, 0, 255), -1)
        image_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        return bounding_box, image_msg, dist_point

    def mono_cam_dist_with_obj_width(self, width_pixels: int, width_meters: int) -> float:
        """Calculate mono cam distance with object width."""
        return(MonoCam.FOCAL_LENGTH * width_meters * MonoCam.IMG_SHAPE[0]) \
            / (width_pixels * MonoCam.SENSOR_SIZE[0])

def main(args: None = None) -> None:
    """Run the node."""
    rclpy.init(args=args)
    bin_detector = BinDetector()
    try:
        rclpy.spin(bin_detector)
    except KeyboardInterrupt:
        pass
    finally:
        bin_detector.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
