#!/usr/bin/env python

from functools import reduce

import cv2
import numpy as np
import rclpy
from custom_msgs.msg import CVObject
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from sklearn.cluster import DBSCAN

from cv.utils import compute_yaw


class PinkBinsDetector(Node):
    """Detect pink bins (deprecated)."""

    MONO_CAM_IMG_SHAPE = (640, 480)  # Width, height in pixels

    def __init__(self) -> None:
        super().__init__('pink_bins_detector')
        self.bridge = CvBridge()
        self.camera = self.declare_parameter('camera', 'front').value
        self.image_sub = self.create_subscription(CompressedImage, f'/camera/usb/{self.camera}/compressed',
                                                  self.image_callback, 10)

        self.pink_bins_hsv_filtered_pub = self.create_publisher(Image, f'/cv/{self.camera}/pink_bins/hsv_filtered',
                                                          10)
        self.pink_bins_dbscan_pub = self.create_publisher(Image, f'/cv/{self.camera}/pink_bins/dbscan', 10)
        self.pink_bins_detections_pub = self.create_publisher(Image, f'/cv/{self.camera}/pink_bins/detections', 10)
        self.pink_bins_bounding_box_pub = self.create_publisher(CVObject, f'/cv/{self.camera}/pink_bins/bounding_box',
                                                          10)

    def image_callback(self, data: CompressedImage) -> None:
        """Convert and processes frames."""
        # Convert the compressed ROS image to OpenCV format
        np_arr = np.frombuffer(data.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        # Process the frame to find and publish information on the bin
        self.process_frame(frame)

    def process_frame(self, frame: np.array) -> None:
        """Process farmes via masking etc."""
        # Convert frame to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define range for blue color and create mask
        mask_1 = cv2.inRange(hsv, np.array([110, 50, 130]), np.array([130, 100, 200]))
        mask_2 = cv2.inRange(hsv, np.array([130, 80, 130]), np.array([160, 150, 255]))
        mask_3 = cv2.inRange(hsv, np.array([155, 100, 150]), np.array([175, 255, 255]))

        if self.camera == 'bottom':
            mask = cv2.inRange(hsv, np.array([160, 150, 200]), np.array([170, 255, 255]))
        else:
            mask = reduce(cv2.bitwise_or, [mask_1, mask_2, mask_3])

        # apply filter and publish
        hsv_filtered_msg = self.bridge.cv2_to_imgmsg(mask, 'mono8')
        self.pink_bins_hsv_filtered_pub.publish(hsv_filtered_msg)

        # handle if nothing detected
        points = np.argwhere(mask > 0)
        if len(points) == 0:
            self.publish_with_no_detection(frame, hsv_filtered_msg)
            return

        dbscan_img = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

        db = DBSCAN(eps=2, min_samples=10).fit(points)
        labels = db.labels_

        unique_labels = set(labels)
        cluster_counts = {k: np.sum(labels == k) for k in unique_labels if k != -1}

        # handle if no clusters
        if cluster_counts == {}:
            self.publish_with_no_detection(frame, hsv_filtered_msg)
            return

        sorted_cluster_labels = sorted(cluster_counts, key=cluster_counts.get, reverse=True)
        colors = [(0, 0, 255), (0, 255, 255), (255, 0, 0)]
        final_x, final_y = 0, 0
        chosen_label_score = None
        for label in zip(sorted_cluster_labels[:3], colors, strict=False):
            class_member_mask = (labels == label)
            max_clust_points = points[class_member_mask]

            for point in max_clust_points:
                dbscan_img[point[0], point[1]] = (0, 255, 0)

            center_x = np.mean(max_clust_points[:, 1])
            center_y = np.mean(max_clust_points[:, 0])

            if center_y > final_y:
                final_x, final_y = center_x, center_y
                chosen_label_score = max_clust_points.shape[0]

        MAX_SCORE = 100  # noqa: N806

        if chosen_label_score < MAX_SCORE:
            self.publish_with_no_detection(frame, hsv_filtered_msg)
            return

        final_point_int = (int(final_x), int(final_y))

        cv2.circle(dbscan_img, final_point_int, 7, (0, 0, 255), -1)

        dbscan_msg = self.bridge.cv2_to_imgmsg(dbscan_img, 'bgr8')
        self.pink_bins_dbscan_pub.publish(dbscan_msg)

        # Publish the frame with lowest point marked
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        cv2.circle(frame, final_point_int, 7, (255, 0, 0), -1)
        frame_msg = self.bridge.cv2_to_imgmsg(frame, 'rgb8')
        self.pink_bins_detections_pub.publish(frame_msg)

        # Publish the bounding box of the bin
        final_x_normalized = final_x / self.MONO_CAM_IMG_SHAPE[0]
        cv_object = CVObject()
        cv_object.header.stamp.sec, cv_object.header.stamp.nanosec  = self.get_clock().now().seconds_nanoseconds()
        cv_object.yaw = -compute_yaw(final_x_normalized, final_x_normalized, self.MONO_CAM_IMG_SHAPE[0])
        cv_object.score = float(chosen_label_score)
        self.pink_bins_bounding_box_pub.publish(cv_object)

    # called if no detections are made
    def publish_with_no_detection(self, frame: np.array, hsv_filtered_msg: Image) -> None:
        """Publish without bounding boxes."""
        frame_msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        self.pink_bins_dbscan_pub.publish(hsv_filtered_msg)
        self.pink_bins_detections_pub.publish(frame_msg)


def main(args:None=None)->None:
    """Run the node."""
    rclpy.init(args=args)
    pink_bins_detector = PinkBinsDetector()

    try:
        rclpy.spin(pink_bins_detector)
    except KeyboardInterrupt:
        pass
    finally:
        pink_bins_detector.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
