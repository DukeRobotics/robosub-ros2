import cv2
import numpy as np
import rclpy
import resource_retriever as rr
from custom_msgs.msg import CVObject
from cv_bridge import CvBridge
from rclpy.node import Node, Publisher
from sensor_msgs.msg import CompressedImage, Image
from functools import reduce
from rclpy.clock import Clock
from rclpy.logging import get_logger

from cv.config import MonoCam, Torpedo
from cv.utils import calculate_relative_pose, compute_yaw, group_contours_by_distance

logger = get_logger('torpedo_target_detector')

class TorpedoTargetDetector(Node):
    """Match contour with torpedo targets."""

    def __init__(self) -> None:
        super().__init__('torpedo_target_detector')

        self.MIN_AREA_OF_CONTOUR = 75
        self.MATCH_TOLERANCE = 2.0

        self.mask_ranges=[
                [Torpedo.LOW_BOT, Torpedo.LOW_TOP],
                [Torpedo.HIGH_BOT, Torpedo.HIGH_TOP]
            ]

        # Load the reference image in grayscale (assumes the image is already binary: white and black)
        reference_image_path = 'package://cv/assets/torpedo_target_mask.png'
        self.reference_image = cv2.imread(rr.get_filename(reference_image_path, use_protocol=False),
                                          cv2.IMREAD_GRAYSCALE)

        # Compute the contours directly on the binary image
        self.ref_contours, _ = cv2.findContours(self.reference_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(CompressedImage, '/camera/usb/front/compressed', self.image_callback,
                                                  10)
        self.fish_bbox_pub = self.create_publisher(CVObject,'/cv/front_usb/torpedo_sawfish_target/bounding_box', 1) # named differently so as to accomodate CV object
        self.shark_bbox_pub = self.create_publisher(CVObject,'/cv/front_usb/torpedo_reef_shark_target/bounding_box', 1)
        self.largest_bbox_pub = self.create_publisher(CVObject, '/cv/front_usb/torpedo_largest_target/bounding_box', 1)
        self.hsv_filtered_pub = self.create_publisher(Image, '/cv/front_usb/torpedo/hsv_filtered', 1)
        self.contour_image_pub = self.create_publisher(Image, '/cv/front_usb/torpedo/contour_image', 1)
        self.contour_image_with_bbox_pub = self.create_publisher(Image, '/cv/front_usb/torpedo/detections', 1)
        self.shark_target = self.create_publisher(Image, '/cv/front_usb/torpedo/shark_target', 1)
        self.fish_target = self.create_publisher(Image, '/cv/front_usb/torpedo/fish_target', 1)
        self.largest_target = self.create_publisher(Image, '/cv/front_usb/torpedo/largest_target', 1)

        self.torpedo_shark_sub = self.create_subscription(CVObject, '/cv/front/shark_front', self.update_shark_bounding_boxes, 1)
        self.torpedo_sawfish_sub = self.create_subscription(CVObject, '/cv/front/swordfish_front', self.update_sawfish_bounding_boxes, 1)
        self.shark_coords = None
        self.sawfish_coords = None
        self.last_update_shark = 0
        self.last_update_sawfish = 0

        self.last_n_bboxes = []
        self.n = 10  # Set the number of last bounding boxes to store

    def update_shark_bounding_boxes(self, data: CVObject) -> None:
        self.last_update_shark = Clock().now().seconds_nanoseconds()[0]
        self.shark_coords = data.coords
        # x, y

    def update_sawfish_bounding_boxes(self, data: CVObject) -> None:
        self.last_update_sawfish = Clock().now().seconds_nanoseconds()[0]
        self.sawfish_coords = data.coords

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

    def score_contour(self, contour: np.ndarray) -> float:
        """
        Score a contour based on its shape similarity to the reference image.

        Parameters:
            contour (np.ndarray): Contour to score.

        Returns:
            float: Similarity score.
        """
        return cv2.matchShapes(self.reference_image, contour, cv2.CONTOURS_MATCH_I1, 0.0)

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
        masks = [cv2.inRange(hsv_image, self.actual_to_opencv_hsv(r[0]), self.actual_to_opencv_hsv(r[1])) for r in self.mask_ranges]
        mask = reduce(cv2.bitwise_or, masks)

        # Apply morphological operations to clean up the binary image
        kernel = np.ones((5, 5), np.uint8)
        red_hsv = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        red_hsv = cv2.dilate(red_hsv, kernel, iterations=1)

        # Publish the HSV filtered image
        hsv_filtered_msg = self.bridge.cv2_to_imgmsg(red_hsv, 'mono8')
        self.hsv_filtered_pub.publish(hsv_filtered_msg)

        # Find contours in the image
        contours, _ = cv2.findContours(red_hsv, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Sort contours by area and shape similarity to the reference image
        contours = sorted(contours, key=lambda cnt: (cv2.contourArea(cnt) / cv2.minEnclosingCircle(cnt)[1]), reverse=True)
        contours = contours[:3]

        # Group contours by distance
        contours = group_contours_by_distance(self, contours, 20)

        # Sort contours by radius of min. enclosing circle
        contours = sorted(contours, key=lambda cnt: cv2.minEnclosingCircle(cnt)[1], reverse=True)
        contours = contours[:2]

        contours = sorted(contours, key=lambda cnt: cv2.matchShapes(self.reference_image, cnt, cv2.CONTOURS_MATCH_I1, 0.0), reverse=True)

        # Get the top 2 contours with the closest match to the shape of the reference image
        contours = contours[:2]

        # Draw contours onto image, and publish the image
        image_with_contours = image.copy()
        cv2.drawContours(image_with_contours, contours, -1, (255, 0, 0), 2)
        contour_image_msg = self.bridge.cv2_to_imgmsg(image_with_contours, 'bgr8')
        self.contour_image_pub.publish(contour_image_msg)

        # only processes contours w/ area > MIN_AREA_OF_CONTOUR
        contours = [contour for contour in contours if cv2.contourArea(contour) > self.MIN_AREA_OF_CONTOUR]

        shark_cnt = None
        fish_cnt = None
        largest_cnt = None
        similar_size_contours = []

        # Match contours with the reference image contours
        for cnt in contours:
            match = cv2.matchShapes(self.ref_contours[0], cnt, cv2.CONTOURS_MATCH_I1, 0.0)
            if match < self.MATCH_TOLERANCE:
                similar_size_contours.append(cnt)

        similar_size_contours = sorted(similar_size_contours, key=cv2.contourArea, reverse=True)

        LATENCY_SEC = 2

        # Find highest and lower contour, assuming that those two will represent the upper and lower holes
        if abs(self.last_update_shark - Clock().now().seconds_nanoseconds()[0]) < LATENCY_SEC and len(similar_size_contours) == 2 and self.shark_coords is not None:
            largest_cnt = similar_size_contours[0]

            x0, y0, _, _ = cv2.boundingRect(similar_size_contours[0])
            x1, y1, _, _ = cv2.boundingRect(similar_size_contours[1])

            dist_shark_0 = (x0 - self.shark_coords.x) ** 2 + (y0 - self.shark_coords.y) ** 2
            dist_shark_1 = (x1 - self.shark_coords.x) ** 2 + (y1 - self.shark_coords.y) ** 2

            if (dist_shark_0 < dist_shark_1):
                shark_cnt = similar_size_contours[0]
                fish_cnt = similar_size_contours[1]
            else:
                fish_cnt = similar_size_contours[0]
                shark_cnt = similar_size_contours[1]

        elif abs(self.last_update_shark - Clock().now().seconds_nanoseconds()[0]) < LATENCY_SEC and abs(self.last_update_sawfish - Clock().now().seconds_nanoseconds()[0]) < LATENCY_SEC and len(similar_size_contours) == 1 and self.shark_coords is not None and self.sawfish_coords is not None:
            largest_cnt = similar_size_contours[0]

            x, y, _, _ = cv2.boundingRect(similar_size_contours[0])
            dist_shark = (x - self.shark_coords.x) ** 2 + (y - self.shark_coords.y) ** 2
            dist_sawfish = (x - self.sawfish_coords.x) ** 2 + (y - self.sawfish_coords.y) ** 2

            if (dist_shark > dist_sawfish):
                shark_cnt = similar_size_contours[0]
            else:
                fish_cnt = similar_size_contours[0]

        bbox_img = image.copy()

        if fish_cnt is not None:
            x, y, w, h = cv2.boundingRect(fish_cnt)
            bbox = (x, y, w, h)
            self.publish_bbox(bbox, self.fish_bbox_pub)
            fish_img = image.copy()
            cv2.rectangle(fish_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            fish_img_msg = self.bridge.cv2_to_imgmsg(fish_img, 'bgr8')
            self.fish_target.publish(fish_img_msg)

            # Draw bounding box on the image
            cv2.rectangle(bbox_img, (x, y), (x + w, y + h), (0, 255, 0), 2)

        if shark_cnt is not None:
            x, y, w, h = cv2.boundingRect(shark_cnt)
            bbox = (x, y, w, h)
            self.publish_bbox(bbox, self.shark_bbox_pub)
            shark_img = image.copy()
            cv2.rectangle(shark_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            shark_img_msg = self.bridge.cv2_to_imgmsg(shark_img, 'bgr8')
            self.shark_target.publish(shark_img_msg)

            # Draw bounding box on the image
            cv2.rectangle(bbox_img, (x, y), (x + w, y + h), (0, 255, 0), 2)

        if largest_cnt is not None:
            x, y, w, h = cv2.boundingRect(largest_cnt)
            bbox = (x, y, w, h)
            self.publish_bbox(bbox, self.largest_bbox_pub)
            largest_img = image.copy()
            cv2.rectangle(largest_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            largest_img_msg = self.bridge.cv2_to_imgmsg(largest_img, 'bgr8')
            self.largest_target.publish(largest_img_msg)

            # Draw bounding box on the image
            cv2.rectangle(bbox_img, (x, y), (x + w, y + h), (0, 255, 0), 2)

        # Convert the image with the bounding box to ROS Image message and publish
        image_msg = self.bridge.cv2_to_imgmsg(bbox_img, 'bgr8')
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
