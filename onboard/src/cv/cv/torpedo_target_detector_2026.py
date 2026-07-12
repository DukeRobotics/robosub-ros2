from functools import reduce

import cv2
import numpy as np
import rclpy
import resource_retriever as rr
from custom_msgs.msg import CVObject
from cv_bridge import CvBridge
from rclpy.clock import Clock
from rclpy.logging import get_logger
from rclpy.node import Node, Publisher
from sensor_msgs.msg import CompressedImage, Image

from cv.config import MonoCam, Torpedo
from cv.utils import calculate_relative_pose, compute_yaw, group_contours_by_distance

logger = get_logger('torpedo_target_detector_2026')

# DepthAI coarse class name -> USB fine-target topic suffix
TARGETS = {
    'ambulance_front': 'ambulance',
    'blood_front': 'blood',
    'firetruck_front': 'firetruck',
    'fire_front': 'fire',
}


class TorpedoTargetDetector2026(Node):
    """Match USB contours with 2026 torpedo targets using DepthAI coarse detections."""

    def __init__(self) -> None:
        super().__init__('torpedo_target_detector_2026')

        self.MIN_AREA_OF_CONTOUR = 75
        self.mask_ranges = [
            [Torpedo.LOW_BOT, Torpedo.LOW_TOP],
            [Torpedo.HIGH_BOT, Torpedo.HIGH_TOP],
        ]

        reference_image_path = 'package://cv/assets/torpedo_target_mask.png'
        self.reference_image = cv2.imread(
            rr.get_filename(reference_image_path, use_protocol=False),
            cv2.IMREAD_GRAYSCALE,
        )

        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            CompressedImage, '/camera/usb/front/compressed', self.image_callback, 10,
        )

        self.bbox_pubs: dict[str, Publisher] = {}
        self.debug_pubs: dict[str, Publisher] = {}
        for coarse_class, target_name in TARGETS.items():
            self.bbox_pubs[coarse_class] = self.create_publisher(
                CVObject, f'/cv/front_usb/torpedo_{target_name}_target/bounding_box', 1,
            )
            self.debug_pubs[coarse_class] = self.create_publisher(
                Image, f'/cv/front_usb/torpedo/{target_name}_target', 1,
            )

        self.hsv_filtered_pub = self.create_publisher(Image, '/cv/front_usb/torpedo1/hsv_filtered', 1)
        self.contour_image_pub = self.create_publisher(Image, '/cv/front_usb/torpedo1/contour_image', 1)
        self.contour_image_with_bbox_pub = self.create_publisher(Image, '/cv/front_usb/torpedo1/detections', 1)
        self.largest_bbox_pub = self.create_publisher(
            CVObject, '/cv/front_usb/torpedo1/largest_target/bounding_box', 1,
        )

        self.coarse_detections: dict[str, dict] = {
            coarse_class: {'coords': None, 'last_update': 0}
            for coarse_class in TARGETS
        }

        for coarse_class in TARGETS:
            self.create_subscription(
                CVObject,
                f'/cv/front/{coarse_class}',
                lambda msg, coarse_class=coarse_class: self._update_coarse_detection(msg, coarse_class),
                1,
            )

    def _update_coarse_detection(self, data: CVObject, coarse_class: str) -> None:
        self.coarse_detections[coarse_class]['last_update'] = Clock().now().seconds_nanoseconds()[0]
        self.coarse_detections[coarse_class]['coords'] = data.coords

    def actual_to_opencv_hsv(self, hsv_actual: np.ndarray) -> np.ndarray:
        hsv_opencv = np.empty_like(hsv_actual, dtype=np.uint8)
        hsv_opencv[..., 0] = (hsv_actual[..., 0] / 2).astype(np.uint8)
        hsv_opencv[..., 1] = (hsv_actual[..., 1] / 100 * 255).astype(np.uint8)
        hsv_opencv[..., 2] = (hsv_actual[..., 2] / 100 * 255).astype(np.uint8)
        return hsv_opencv

    def _find_target_contours(self, red_hsv: np.ndarray) -> list:
        contours, _ = cv2.findContours(red_hsv, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = [cnt for cnt in contours if cv2.contourArea(cnt) > self.MIN_AREA_OF_CONTOUR]

        contours = sorted(
            contours,
            key=lambda cnt: cv2.contourArea(cnt) / max(cv2.minEnclosingCircle(cnt)[1], 1),
            reverse=True,
        )
        contours = contours[:4]
        contours = group_contours_by_distance(contours, 20)
        contours = sorted(contours, key=lambda cnt: cv2.boundingRect(cnt)[3], reverse=True)
        contours = contours[:4]
        return sorted(
            contours,
            key=lambda cnt: cv2.matchShapes(self.reference_image, cnt, cv2.CONTOURS_MATCH_I1, 0.0),
        )

    def _assign_contours(self, contours: list, current_time: int, latency_sec: int = 2) -> dict[str, np.ndarray]:
        active_targets = {
            coarse_class: state['coords']
            for coarse_class, state in self.coarse_detections.items()
            if state['coords'] is not None
            and abs(state['last_update'] - current_time) < latency_sec
        }
        if not active_targets or not contours:
            return {}

        pairs = []
        for coarse_class, coords in active_targets.items():
            for idx, contour in enumerate(contours):
                x, y, w, h = cv2.boundingRect(contour)
                center_x = x + w / 2
                center_y = y + h / 2
                dist = (center_x - coords.x) ** 2 + (center_y - coords.y) ** 2
                pairs.append((dist, idx, coarse_class))

        pairs.sort(key=lambda pair: pair[0])
        assignments: dict[str, np.ndarray] = {}
        used_contours: set[int] = set()
        used_targets: set[str] = set()
        for _, idx, coarse_class in pairs:
            if idx in used_contours or coarse_class in used_targets:
                continue
            assignments[coarse_class] = contours[idx]
            used_contours.add(idx)
            used_targets.add(coarse_class)

        return assignments

    def image_callback(self, data: CompressedImage) -> None:
        try:
            np_arr = np.frombuffer(data.data, np.uint8)
            image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        except (TypeError, AttributeError) as error:
            self.get_logger().error(f'Failed to convert image: {error}')
            return

        masks = [
            cv2.inRange(hsv_image, self.actual_to_opencv_hsv(r[0]), self.actual_to_opencv_hsv(r[1]))
            for r in self.mask_ranges
        ]
        mask = reduce(cv2.bitwise_or, masks)

        kernel = np.ones((5, 5), np.uint8)
        red_hsv = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        red_hsv = cv2.dilate(red_hsv, kernel, iterations=1)

        self.hsv_filtered_pub.publish(self.bridge.cv2_to_imgmsg(red_hsv, 'mono8'))

        contours = self._find_target_contours(red_hsv)
        image_with_contours = image.copy()
        cv2.drawContours(image_with_contours, contours, -1, (255, 0, 0), 2)
        self.contour_image_pub.publish(self.bridge.cv2_to_imgmsg(image_with_contours, 'bgr8'))

        current_time = Clock().now().seconds_nanoseconds()[0]
        assignments = self._assign_contours(contours, current_time)
        bbox_img = image.copy()

        if contours:
            largest = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest)
            self.publish_bbox((x, y, w, h), self.largest_bbox_pub)
            cv2.rectangle(bbox_img, (x, y), (x + w, y + h), (255, 255, 0), 2)

        for coarse_class, contour in assignments.items():
            x, y, w, h = cv2.boundingRect(contour)
            self.publish_bbox((x, y, w, h), self.bbox_pubs[coarse_class])

            target_img = image.copy()
            cv2.rectangle(target_img, (x, y), (x + w, y + h), (0, 255, 0), 2)
            self.debug_pubs[coarse_class].publish(self.bridge.cv2_to_imgmsg(target_img, 'bgr8'))
            cv2.rectangle(bbox_img, (x, y), (x + w, y + h), (0, 255, 0), 2)

        self.contour_image_with_bbox_pub.publish(self.bridge.cv2_to_imgmsg(bbox_img, 'bgr8'))

    def publish_bbox(self, bbox: tuple[int, int, int, int], publisher: Publisher) -> None:
        x, y, w, h = bbox
        bounding_box = CVObject()
        bounding_box.header.stamp.sec, bounding_box.header.stamp.nanosec = (
            self.get_clock().now().seconds_nanoseconds()
        )

        bounding_box.xmin = float(x)
        bounding_box.ymin = float(y)
        bounding_box.xmax = float(x + w)
        bounding_box.ymax = float(y + h)
        bounding_box.yaw = -float(
            compute_yaw(x / MonoCam.IMG_SHAPE[0], (x + w) / MonoCam.IMG_SHAPE[0], MonoCam.IMG_SHAPE[0]),
        )
        bounding_box.width = int(w)
        bounding_box.height = int(h)

        bbox_bounds = (
            x / MonoCam.IMG_SHAPE[0],
            y / MonoCam.IMG_SHAPE[1],
            (x + w) / MonoCam.IMG_SHAPE[0],
            (y + h) / MonoCam.IMG_SHAPE[1],
        )
        coords_list = calculate_relative_pose(
            bbox_bounds,
            MonoCam.IMG_SHAPE,
            (Torpedo.WIDTH, Torpedo.WIDTH),
            MonoCam.FOCAL_LENGTH,
            MonoCam.SENSOR_SIZE,
            1,
        )
        bounding_box.coords.x, bounding_box.coords.y, bounding_box.coords.z = coords_list
        publisher.publish(bounding_box)


def main(args: None = None) -> None:
    rclpy.init(args=args)
    node = TorpedoTargetDetector2026()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
