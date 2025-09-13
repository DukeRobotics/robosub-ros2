import cv2
import numpy as np
import rclpy

from cv import hsv_filter

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


class TorpedoTargetDetector(hsv_filter.HSVFilter):
    def __init__(self) -> None:
        super().__init__(
            name='torpedos1',
            camera='front',
            mask_ranges=np.array([
                [Torpedo.LOW_BOT, Torpedo.HIGH_BOT],
                [Torpedo.LOW_TOP, Torpedo.HIGH_TOP]
            ]),
            width=Torpedo.WIDTH,
            pubs=['sword', 'sawfish', 'largest']
        )

    def create_additional_pubs_subs_vars(self):
        self.torpedo_shark_sub = self.create_subscription(CVObject, '/cv/front/shark_front', self.update_shark_bounding_boxes, 1)
        self.torpedo_sawfish_sub = self.create_subscription(CVObject, '/cv/front/swordfish_front', self.update_sawfish_bounding_boxes, 1)
        self.shark_coords = None
        self.sawfish_coords = None
        self.last_update_shark = 0
        self.last_update_sawfish = 0

        self.last_n_bboxes = []
        self.n = 10  # Set the number of last bounding boxes to store

        # Load the reference image in grayscale (assumes the image is already binary: white and black)
        reference_image_path = 'package://cv/assets/torpedo_target_mask.png'
        self.reference_image = cv2.imread(rr.get_filename(reference_image_path, use_protocol=False),
                                          cv2.IMREAD_GRAYSCALE)

    def update_shark_bounding_boxes(self, data: CVObject) -> None:
        self.last_update_shark = Clock().now().seconds_nanoseconds()[0]
        self.shark_coords = data.coords
        # x, y

    def update_sawfish_bounding_boxes(self, data: CVObject) -> None:
        self.last_update_sawfish = Clock().now().seconds_nanoseconds()[0]
        self.sawfish_coords = data.coords

    def morphology(self, mask: np.ndarray) -> np.ndarray:
        # Apply morphological operations to clean up the binary image
        kernel = np.ones((5, 5), np.uint8)
        red_hsv = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=1)
        red_hsv = cv2.dilate(red_hsv, kernel, iterations=1)

        return red_hsv

    def filter(self, contours: list) -> list:
        """Filter out list of contours."""
        # Sort contours by area and shape similarity to the reference image
        contours = sorted(contours, key=lambda cnt: (cv2.contourArea(cnt) / cv2.minEnclosingCircle(cnt)[1]), reverse=True)
        contours = contours[:3]

        # Group contours by distance
        contours = group_contours_by_distance(self, contours, 20)

        # Sort contours by radius of min. enclosing circle
        contours = sorted(contours, key=lambda cnt: cv2.boundingRect(cnt)[3], reverse=True)
        contours = contours[:2]

        contours = sorted(contours, key=lambda cnt: cv2.matchShapes(self.reference_image, cnt, cv2.CONTOURS_MATCH_I1, 0.0), reverse=False)

        # Get the top 2 contours with the closest match to the shape of the reference image
        contours = contours[:2]

        # only processes contours w/ area > MIN_AREA_OF_CONTOUR
        # contours = [contour for contour in contours if cv2.contourArea(contour) > self.MIN_AREA_OF_CONTOUR]

        shark_cnt = None
        fish_cnt = None
        largest_cnt = None
        similar_size_contours = contours # this works

        # Match contours with the reference image contours
        # tbh idk why it thinks some of the circles are 2.x but shrug just gonna ignore this for now
        # for cnt in contours:
        #     match = cv2.matchShapes(self.ref_contours[0], cnt, cv2.CONTOURS_MATCH_I1, 0.0)
        #     logger.info(f'{match}')
        #     if match < self.MATCH_TOLERANCE:
        #         similar_size_contours.append(cnt)

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

        return [shark_cnt, fish_cnt, largest_cnt]