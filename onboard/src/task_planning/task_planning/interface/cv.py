from enum import Enum
from pathlib import Path
from typing import ClassVar

import numpy as np
import resource_retriever as rr
import yaml
from custom_msgs.msg import CVObject
from geometry_msgs.msg import Point, Pose
from rclpy.clock import Clock
from rclpy.logging import get_logger
from rclpy.node import Node
from std_msgs.msg import Float64
from task_planning.utils.other_utils import singleton

logger = get_logger('cv_interface')


class CVObjectType(Enum):
    """Enum for the types of CV objects available through the CV interface."""
    BIN_BLUE = 'bin_blue'
    BIN_RED = 'bin_red'
    BIN_WHOLE = 'bin_whole'
    BUOY = 'buoy'
    GATE_REEF_SHARK = 'b'
    GATE_SAWFISH = 'c'
    GATE_WHOLE = 'a'
    LANE_MARKER = 'lane_marker'
    PATH_MARKER = 'path_marker'
    TORPEDO_BUOY = 'd'
    TORPEDO_REEF_SHARK = 'e'
    TORPEDO_SAWFISH = 'f'
    TORPEDO_LOWER_TARGET = 'h'
    TORPEDO_UPPER_TARGET = 'g'


@singleton
class CV:
    """
    Interface for the computer vision subsystem.

    Attributes:
        _instance: The singleton instance of this class. Is a static attribute.
        _bounding_boxes: The dictionary containing the bounding boxes of each CV-detected object.
        _distances: The dictionary containing the positions of each CV-detected object from the center of the frame.
        _angles: The dictionary containing the angles (in radians) of each CV-detected object to the frame's horizontal.
        _lane_marker_data: The dictionary containing lane marker-specific data.
        _distance_queues: The dictionary mapping each CV-detected object to a queue
            that stores the last 10 distance values for computing a moving average.
        _angle_queues: The dictionary mapping each CV-detected object to a queue
            that stores the last 10 angle values for computing a moving average.
        _lane_marker_heights: The queue storing the last 10 lane marker height values for computing a moving average.
    """
    # NOTE: Initialized all objects so as to avoid accessing fields of None values at the beginning.
    # In CV, we tend to always check for detection recency so we wouldn't have an issue with wrong default values.

    MODELS_PATH = 'package://cv/models/depthai_models.yaml'
    CV_CAMERA = 'front'
    # TODO: add other CV models here as defined in depthai_models.yaml. Modify the Enum strings correspondingly.
    CV_MODELS: ClassVar[list[str]] = ['yolov7_tiny_2023_main']

    BOUNDING_BOX_TOPICS: ClassVar[dict[CVObjectType, str]] = {
        CVObjectType.BUOY: '/cv/front_usb/buoy/bounding_box',
        CVObjectType.BIN_BLUE: '/cv/bottom/bin_blue/bounding_box',
        CVObjectType.BIN_RED: '/cv/bottom/bin_red/bounding_box',
        CVObjectType.LANE_MARKER: '/cv/bottom/lane_marker/bounding_box',
        CVObjectType.PATH_MARKER: '/cv/bottom/path_marker/bounding_box',
    }

    DISTANCE_TOPICS: ClassVar[dict[CVObjectType, str]] = {
        CVObjectType.BIN_BLUE: '/cv/bottom/bin_blue/distance',
        CVObjectType.BIN_RED: '/cv/bottom/bin_red/distance',
        CVObjectType.LANE_MARKER: '/cv/bottom/lane_marker/distance',
        CVObjectType.PATH_MARKER: '/cv/bottom/path_marker/distance',
    }

    ANGLE_TOPICS: ClassVar[dict[CVObjectType, str]] = {
        CVObjectType.LANE_MARKER: '/cv/bottom/lane_marker/angle',
    }

    FRAME_HEIGHT = 480

    def __init__(self, node: Node, bypass: bool = False) -> None:
        self.node = node
        self.bypass = bypass

        # Subscribe to bounding box topics
        self._bounding_boxes: dict[CVObjectType, CVObject] = dict.fromkeys(self.BOUNDING_BOX_TOPICS, CVObject())
        with Path(rr.get_filename(self.MODELS_PATH, use_protocol=False)).open() as f:
            models_dict = yaml.safe_load(f)

            for model in models_dict.values():
                for model_class in model['classes']:
                    topic = f"{model['topic']}/{self.CV_CAMERA}/{model_class}"
                    node.create_subscription(
                        CVObject,
                        topic,
                        lambda msg, model_class=model_class: self._on_receive_bounding_box_data(msg, model_class),
                        10,
                    )

        for object_type, object_topic in self.BOUNDING_BOX_TOPICS.items():
            node.create_subscription(
                CVObject,
                object_topic,
                lambda msg, object_type=object_type: self._on_receive_bounding_box_data(msg, object_type),
                10,
            )

        # Subscribe to distance topics
        self._distances: dict[CVObjectType, Point] = dict.fromkeys(self.DISTANCE_TOPICS, Point())
        self._distance_queues: dict[CVObjectType, dict[str, list[float]]] = {
            object_type: {'x': [], 'y': []}
            for object_type in self.DISTANCE_TOPICS
        }
        for object_type, object_topic in self.DISTANCE_TOPICS.items():
            node.create_subscription(
                Point,
                object_topic,
                lambda msg, object_type=object_type: self._on_receive_distance_data(msg, object_type),
                10,
            )

        # Subscribe to angle topics
        self._angles: dict[CVObjectType, float] = dict.fromkeys(self.ANGLE_TOPICS, 0)
        self._angle_queues: dict[CVObjectType, list[float]] = {object_type: [] for object_type in self.ANGLE_TOPICS}
        for object_type, object_topic in self.ANGLE_TOPICS.items():
            node.create_subscription(
                Float64,
                object_topic,
                lambda msg, object_type=object_type: self._on_receive_angle_data(msg, object_type),
                10,
            )

        # Lane marker-specific data
        self._lane_marker_data = {}
        self._lane_marker_heights = []

    @property
    def bounding_boxes(self) -> dict[CVObjectType, CVObject]:
        """The dictionary containing the bounding boxes of each CV-detected object."""
        return self._bounding_boxes

    @property
    def distances(self) -> dict[CVObjectType, Point]:
        """The dictionary containing the positions of each CV-detected object from the center of the frame."""
        return self._distances

    @property
    def angles(self) -> dict[CVObjectType, float]:
        """The dictionary containing the angles (in radians) of each CV-detected object to the frame's horizontal."""
        return self._angles

    @property
    def lane_marker_data(self) -> dict:
        """The dictionary containing lane marker-specific data."""
        return self._lane_marker_data

    def _on_receive_bounding_box_data(self, cv_data: CVObject, object_type: CVObjectType) -> None:
        """
        Store the received CV bounding box.

        Args:
            cv_data (CVObject): The received CV data.
            object_type (CVObjectType): The name/type of the object.
        """
        self._bounding_boxes[object_type] = cv_data

        if object_type == CVObjectType.LANE_MARKER:
            self._lane_marker_data['height'] = self.update_moving_average(self._lane_marker_heights, cv_data.height)
            # Determine if lane marker is touching top and/or bottom of frame
            self._lane_marker_data['touching_top'] = cv_data.coords.y - cv_data.height / 2 <= 0
            self._lane_marker_data['touching_bottom'] = cv_data.coords.y + cv_data.height / 2 >= self.FRAME_HEIGHT

    def _on_receive_distance_data(self, distance_data: Point, object_type: CVObjectType, filter_len: int = 10) -> None:
        """
        Parse the received distance data and store it.

        Args:
            distance_data (Point): The received distance data.
            object_type (CVObjectType): The name/type of the object.
            filter_len (int, optional): The maximum number of distance data points to retain
                for the moving average filter. Defaults to 10.
        """
        avg_dist = Point()
        avg_dist.x = self.update_moving_average(self._distance_queues[object_type]['x'], distance_data.x, filter_len)
        avg_dist.y = self.update_moving_average(self._distance_queues[object_type]['y'], distance_data.y, filter_len)
        self._distances[object_type] = avg_dist

        if object_type in [CVObjectType.BIN_RED, CVObjectType.BIN_BLUE]:
            # This angle is calculated from averaged distance values, so no moving average filter is needed here
            self._angles[CVObjectType.BIN_WHOLE] = self.compute_angle_from_horizontal(
                self._distances[CVObjectType.BIN_RED], self._distances[CVObjectType.BIN_BLUE],
            )

    def _on_receive_angle_data(self, angle_data: Float64, object_type: CVObjectType, filter_len: int = 10) -> None:
        """
        Parse the received angle data and store it.

        Args:
            angle_data (Float64): The received angle data in degrees.
            object_type (CVObjectType): The name/type of the object.
            filter_len (int, optional): The maximum number of angle data points to retain
                for the moving average filter. Defaults to 10.
        """
        self._angles[object_type] = self.update_moving_average(self._angle_queues[object_type],
                                                               angle_data.data, filter_len)

    def compute_angle_from_horizontal(self, p1: Point, p2: Point) -> float:
        """
        Compute the angle (in radians) between the horizontal and the line connecting two points.

        The points are expected to be distances from the center of the frame.

        Args:
            p1 (Point): The distance of point 1 from the center of the frame.
            p2 (Point): The distance of point 2 from the center of the frame.

        Returns:
            float: Angle in radians between -π/2 and π/2.
        """
        dx = p1.x - p2.x
        dy = p1.y - p2.y
        angle = np.arctan2(dy, dx)

        # Normalize to [-π/2, π/2]
        if angle > np.pi / 2:
            angle -= np.pi
        elif angle < -np.pi / 2:
            angle += np.pi

        return angle

    def update_moving_average(self, queue: list[float], new_value: float, filter_len: int = 10) -> float:
        """
        Update the moving average filter with a new value.

        Args:
            queue (list): The current queue containing previous values.
            new_value (float): The new value to be added.
            filter_len (int, optional): The size of the moving window. Defaults to 10.

        Returns:
            float: The new moving average.
        """
        queue.append(new_value)
        if len(queue) > filter_len:
            queue.pop(0)

        return sum(queue) / len(queue)

    def get_pose(self, name: CVObjectType) -> Pose:
        """
        Get the pose of a detected object.

        Args:
            name (CVObjectType): The name/type of the object.

        Returns:
            Pose: The pose of the object.
        """
        pose = Pose()

        if name not in self._bounding_boxes:
            logger.warning(f'Attempted to get pose of unrecognized CV object {name}')
            return pose

        data = self._bounding_boxes[name]
        pose.position.x = data.coords.x
        pose.position.y = data.coords.y
        pose.position.z = data.coords.z
        pose.orientation.x = 0
        pose.orientation.y = 0
        pose.orientation.z = 0
        pose.orientation.w = 1
        return pose

    def is_receiving_recent_cv_data(
        self,
        name: CVObjectType,
        latency: float,
        last_detection_time: int | None = None,
    ) -> bool:
        """
        Check if a detection for the specified object has been received within the given latency window (in seconds).

        Optionally, also check if this is within the latency window of the previous detection.

        Args:
            name (CVObjectType): The name/type of the object.
            latency (float): Maximum allowed time (in seconds) since the last detection.
            last_detection_time (int | None): Optional timestamp (in seconds) of the previous detection for this object.
                If provided, also check whether both the current and previous detections are within the
                latency threshold of each other.

        Returns:
            bool: True if recent data has been received, False otherwise.
        """
        if name not in self._bounding_boxes:
            return False

        current_time = Clock().now().seconds_nanoseconds()[0]
        detection_time = self._bounding_boxes[name].header.stamp.secs

        recent = current_time - detection_time < latency

        if last_detection_time is not None:
            return recent and abs(detection_time - last_detection_time) < latency

        return recent
