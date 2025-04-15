from pathlib import Path

import cv2
import numpy as np
import rclpy
import resource_retriever as rr
from custom_msgs.msg import CVObject
from cv_bridge import CvBridge
from rclpy.node import Node, Publisher
from sensor_msgs.msg import CompressedImage, Image

from cv.config import MonoCam, Torpedo
from cv.utils import calculate_relative_pose, compute_yaw


class TorpedoTargetDetector(Node):
    """Match contour with torpedo targets."""

    def __init__(self) -> None:
        super().__init__('torpedo_target_detector')

        self.MIN_AREA_OF_CONTOUR = 75
        self.MATCH_TOLERANCE = 0.2

        # Load the reference image in grayscale (assumes the image is already binary: white and black)
        reference_image_path = 'package://cv/assets/torpedo_target_mask.png'
        self.reference_image = cv2.imread(rr.get_filename(reference_image_path, use_protocol=False),
                                          cv2.IMREAD_GRAYSCALE)

        # Compute the contours directly on the binary image
        self.ref_contours, _ = cv2.findContours(self.reference_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(CompressedImage, '/camera/usb/front/compressed', self.image_callback,
                                                  10)
        self.upper_bounding_box_pub = self.create_publisher(CVObject,'/cv/front_usb/torpedo/upper/bounding_box', 1)
        self.lower_bounding_box_pub = self.create_publisher(CVObject,'/cv/front_usb/torpedo/lower/bounding_box', 1)
        self.hsv_filtered_pub = self.create_publisher(Image, '/cv/front_usb/torpedo/hsv_filtered', 1)
        self.contour_image_pub = self.create_publisher(Image, '/cv/front_usb/torpedo/contour_image', 1)
        self.contour_image_with_bbox_pub = self.create_publisher(Image, '/cv/front_usb/torpedo/detections', 1)

        self.last_n_bboxes = []
        self.n = 10  # Set the number of last bounding boxes to store


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

        # Define the range for HSV filtering on the outer red circle
        lower_red = np.array([0, 70, 90])
        upper_red = np.array([12, 100, 110])
        #lower_red_upper = np.array([175, 180, 190])  # noqa: ERA001
        #upper_red_upper = np.array([179, 255, 255])  # noqa: ERA001

        # Apply HSV filtering on the image
        mask1 = cv2.inRange(hsv_image, lower_red, upper_red)
        #mask2 = cv2.inRange(hsv_image, lower_red_upper, upper_red_upper) # noqa: ERA001
        #red_hsv = cv2.bitwise_or(mask1, mask2) # noqa: ERA001
        red_hsv = mask1

        # Apply morphological operations to clean up the binary image
        kernel = np.ones((5, 5), np.uint8)
        red_hsv = cv2.morphologyEx(red_hsv, cv2.MORPH_CLOSE, kernel)

        # Publish the HSV filtered image
        hsv_filtered_msg = self.bridge.cv2_to_imgmsg(red_hsv, 'mono8')
        self.hsv_filtered_pub.publish(hsv_filtered_msg)

        # Find contours in the image
        contours, _ = cv2.findContours(red_hsv, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Sort contours by area
        contours = sorted(contours, key=cv2.contourArea, reverse=True)

        # Get the top 2 contours with the largest area
        contours = contours[:2]

        # Draw contours onto image, and publish the image
        image_with_contours = image.copy()
        cv2.drawContours(image_with_contours, contours, -1, (255, 0, 0), 2)
        contour_image_msg = self.bridge.cv2_to_imgmsg(image_with_contours, 'bgr8')
        self.contour_image_pub.publish(contour_image_msg)

        # only processes contours w/ area > MIN_AREA_OF_CONTOUR
        contours = [contour for contour in contours if cv2.contourArea(contour) > self.MIN_AREA_OF_CONTOUR]

        upper_cnt = None
        lower_cnt = None
        similar_size_contours = []

        # Match contours with the reference image contours
        for cnt in contours:
            match = cv2.matchShapes(self.ref_contours[0], cnt, cv2.CONTOURS_MATCH_I1, 0.0)
            if match < self.MATCH_TOLERANCE:
                similar_size_contours.append(cnt)

        # Find highest and lower contour, assuming that those two will represent the upper and lower holes
        if similar_size_contours:
            similar_size_contours.sort(key=lambda x: cv2.contourArea(x))
            upper_cnt = similar_size_contours[0]
            lower_cnt = similar_size_contours[0]
            for cnt in similar_size_contours:
                x, y, w, h = cv2.boundingRect(cnt)
                if y > cv2.boundingRect(upper_cnt)[1]:
                    upper_cnt = cnt
                if y < cv2.boundingRect(lower_cnt)[1]:
                    lower_cnt = cnt

        if upper_cnt is not None:
            x, y, w, h = cv2.boundingRect(upper_cnt)
            bbox = (x, y, w, h)
            self.publish_bbox(bbox, self.upper_bounding_box_pub)

            # Draw bounding box on the image
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        if lower_cnt is not None:
            x, y, w, h = cv2.boundingRect(upper_cnt)
            bbox = (x, y, w, h)
            self.publish_bbox(bbox, self.lower_bounding_box_pub)

            # Draw bounding box on the image
            cv2.rectangle(image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Convert the image with the bounding box to ROS Image message and publish
        image_msg = self.bridge.cv2_to_imgmsg(image, 'bgr8')
        self.contour_image_with_bbox_pub.publish(image_msg)

    def filter_outliers(self, bboxes: np.array) -> np.array:
        """Filter out outliers if there are more than two bounding boxes."""
        if len(bboxes) <= 2:  # noqa: PLR2004
            return bboxes

        centers = [(x + w / 2, y + h / 2) for x, y, w, h in bboxes]
        mean_center = np.mean(centers, axis=0)
        distances = [np.linalg.norm(np.array(center) - mean_center) for center in centers]
        std_distance = np.std(distances)

        areas = [w * h for x, y, w, h in bboxes]
        mean_area = np.mean(areas)
        std_area = np.std(areas)

        return [
            bboxes[i] for i in range(len(bboxes))
            if distances[i] <= mean_center[0] + 2 * std_distance and abs(areas[i] - mean_area) <= std_area
        ]


    def publish_bbox(self, bbox: tuple[int, int, int, int], publisher: Publisher) -> None:
        """
        Create a CVObject message to publish to the bounding box publisher.

        Calculations are done based off of the pased in x, y, width, and height of the rectangle.
        """
        x, y, w, h = bbox

        bounding_box = CVObject()

        bounding_box.header.stamp.sec, bounding_box.header.stamp.nanosec = self.get_clock().now().seconds_nanoseconds()

        bounding_box.xmin = float(x)
        bounding_box.ymin = float(y)
        bounding_box.xmax = float(x + w)
        bounding_box.ymax = float(y + h)

        bounding_box.yaw = -float(compute_yaw(x / MonoCam.IMG_SHAPE[0], (x + w) / MonoCam.IMG_SHAPE[0],
                                        MonoCam.IMG_SHAPE[0])) # Update 0 with self.camera_pixel_width

        bounding_box.width = int(w)
        bounding_box.height = int(h)

        bbox_bounds = (x / MonoCam.IMG_SHAPE[0], y / MonoCam.IMG_SHAPE[1],
                       (x+w) / MonoCam.IMG_SHAPE[0], (y+h) / MonoCam.IMG_SHAPE[1])

        # Point coords represents the 3D position of the object represented by the bounding box relative to the robot
        coords_list = calculate_relative_pose(bbox_bounds,
                                              MonoCam.IMG_SHAPE,
                                              (Torpedo.WIDTH, 0),
                                              MonoCam.FOCAL_LENGTH,
                                              MonoCam.SENSOR_SIZE, 1)
        bounding_box.coords.x, bounding_box.coords.y, bounding_box.coords.z = coords_list

        publisher.publish(bounding_box)


def main(args: None = None) -> None:
    """Run the node."""
    rclpy.init(args=args)
    torpedo_target_detector = TorpedoTargetDetector()

    try:
        rclpy.spin(torpedo_target_detector)
    except KeyboardInterrupt:
        pass
    finally:
        torpedo_target_detector.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
