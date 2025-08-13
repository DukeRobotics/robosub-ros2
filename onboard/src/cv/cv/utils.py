import math
import os
from pathlib import Path

import cv2
import numpy as np
from custom_msgs.msg import CVObject
from geometry_msgs.msg import Point, Polygon

from scipy.spatial.distance import cdist


def check_file_writable(filepath: Path) -> bool:
    """
    Check if a file can be created or overwritten.

    Args:
        filepath (Path): The path to the file to check.

    Returns:
        bool: True if the file can be created or overwritten, False otherwise.
    """
    if filepath.exists():
        # Path exists
        if filepath.is_file():
            # Path is a file, check if it is writable
            return os.access(filepath, os.W_OK)
        # Path is a dir, so cannot write as a file
        return False
    # Path does not exist, check permissions on parent directory
    pdir = filepath.parent
    if not pdir:
        pdir = '.'
    # Target is creatable if parent dir is writable
    return os.access(pdir, os.W_OK)


def cam_dist_with_obj_width(width_pixels: float, width_meters: float,
                            focal_length: float, img_shape: tuple[float, float], sensor_size: tuple[float, float],
                            adjustment_factor: int = 1) -> float:
    """Note that adjustment factor is 1 for mono camera and 2 for depthAI camera."""
    return (focal_length * width_meters * img_shape[0]) \
        / (width_pixels * sensor_size[0]) * adjustment_factor


def cam_dist_with_obj_height(height_pixels: float, height_meters: float,
                             focal_length: float, img_shape: tuple[float, float], sensor_size: tuple[float, float],
                             adjustment_factor: int = 1) -> float:
    """Return camera distance with object height."""
    return (focal_length * height_meters * img_shape[1]) \
        / (height_pixels * sensor_size[1]) * adjustment_factor


def compute_yaw(xmin: float, xmax: float, camera_pixel_width: float) -> float:
    """Find the yaw angle offset."""
    left_end_compute = compute_angle_from_x_offset(xmin * camera_pixel_width, camera_pixel_width)
    right_end_compute = compute_angle_from_x_offset(xmax * camera_pixel_width, camera_pixel_width)
    midpoint = (left_end_compute + right_end_compute) / 2.0
    return (midpoint) * (math.pi / 180.0)  # Degrees to radians


def compute_angle_from_x_offset(x_offset: float, camera_pixel_width: float) -> float:
    """
    Compute angle from x offset.

    See: https://math.stackexchange.com/questions/1320285/convert-a-pixel-displacement-to-angular-rotation
    for implementation details.

    Args:
        x_offset (int): x pixels from center of image.
        camera_pixel_width (int): Width of the camera in pixels.

    Returns:
        float: Angle in degrees.
    """
    image_center_x = camera_pixel_width / 2.0
    return math.degrees(math.atan((x_offset - image_center_x) * 0.005246675486))


def calculate_relative_pose(bbox_bounds: list[int | float], input_size: tuple[float, float],
                            label_shape: tuple[float, float], focal_length: float,
                            sensor_size: tuple[float, float], adjustment_factor: int) -> list[float]:
    """
    Return relative pose, to be used as a part of the CVObject.

    Args:
        bbox_bounds (object): The detection object.
        input_size (list[float]): Array with input size, where [0] is width and [1] is height.
        label_shape (list[float]): The label shape, where [0] is width (only this is accessed) and [1] is height.
        focal_length (float): The distance between the lens and the image sensor when the lens is focused on a subject.
        sensor_size (tuple[float, float]): The physical size of the camera's image sensor.
        adjustment_factor (int): 1 if mono, 2 if depthai.

    Returns:
        list[float]: The relative pose of the object.
    """
    xmin, ymin, xmax, ymax = bbox_bounds

    bbox_width = (xmax - xmin) * input_size[0]
    bbox_center_x = (xmin + xmax) / 2 * input_size[0]
    bbox_center_y = (ymin + ymax) / 2 * input_size[1]
    meters_per_pixel = label_shape[0] / bbox_width
    dist_x = bbox_center_x - input_size[0] // 2
    dist_y = bbox_center_y - input_size[1] // 2

    y_meters = dist_x * meters_per_pixel * -1
    z_meters = dist_y * meters_per_pixel * -1

    x_meters = cam_dist_with_obj_width(bbox_width, label_shape[0], focal_length, input_size, sensor_size,
                                       adjustment_factor)

    return [x_meters, y_meters, z_meters]


def compute_bbox_dimensions(polygon: Polygon) -> CVObject:
    """
    Return a CVObject message, containing the following properties of the given Polygon.

    width, height, xmin, ymin, xmax, ymax

    Args:
        polygon: Polygon object

    Returns:
        CVObject: The CVObject message.
    """
    # Ensure there are points in the polygon
    max_points = 4
    if len(polygon.points) < max_points:
        msg = 'Polygon does not represent a bounding box with four points.'
        raise ValueError(msg)

    # Initialize min_x, max_x, min_y, and max_y with the coordinates of the first point
    min_x = polygon.points[0].x
    max_x = polygon.points[0].x
    min_y = polygon.points[0].y
    max_y = polygon.points[0].y

    # Iterate through all points to find the min and max x and y coordinates
    for point in polygon.points:
        min_x = min(point.x, min_x)
        max_x = max(point.x, max_x)
        min_y = min(point.y, min_y)
        max_y = max(point.y, max_y)

    # Compute the width and height
    width = max_x - min_x
    height = max_y - min_y

    # Create and populate message of CVObject type using polygon fields
    msg = CVObject()

    msg.width = width
    msg.height = height

    msg.xmin = min_x
    msg.xmax = max_x

    msg.ymin = min_y
    msg.ymax = max_y

    center = Point()
    center.x = (min_x + max_x) / 2
    center.y = (min_y + max_y) / 2

    msg.coords = center
    msg.yaw = 0

    return msg


def compute_center_distance(bbox_center_x: float, bbox_center_y: float, frame_width: float, frame_height: float,
                            width_adjustment_constant: float = 0,
                            height_adjustment_constant: float = 0) -> tuple[float, float]:
    """Note that x, y is in the camera's reference frame."""
    # Compute the center of the frame
    frame_center_x = frame_width / 2
    frame_center_y = frame_height / 2

    # Compute the distances between the centers
    distance_x = bbox_center_x - frame_center_x + height_adjustment_constant
    distance_y = bbox_center_y - frame_center_y + width_adjustment_constant

    return distance_x, distance_y

def group_contours_by_distance(self, contours, dist_thresh):
        centers = []
        valid_contours = []
        for contour in contours:
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                centers.append([cx, cy])
                valid_contours.append(contour)

        centers = np.array(centers)
        n = len(centers)
        if n == 0:
            return []

        dist_matrix = cdist(centers, centers)
        groups = []
        visited = set()

        for i in range(n):
            if i in visited:
                continue
            group = {i}
            neighbors = set(np.where(dist_matrix[i] < dist_thresh)[0])
            group = group.union(neighbors)

            expanded = True
            while expanded:
                expanded = False
                new_neighbors = set()
                for idx in group:
                    idx_neighbors = set(np.where(dist_matrix[idx] < dist_thresh)[0])
                    if not idx_neighbors.issubset(group):
                        new_neighbors = new_neighbors.union(idx_neighbors.difference(group))
                        expanded = True
                group = group.union(new_neighbors)

            visited = visited.union(group)
            groups.append(list(group))

        merged_contours = []
        for group_indices in groups:
            merged_points = np.vstack([valid_contours[idx] for idx in group_indices])
            merged_contours.append(merged_points)

        return merged_contours


class DetectionVisualizer:
    """
    Helper methods to visualize detections on an image feed.

    Adapted from class TextHelper:
    https://github.com/luxonis/depthai-experiments/blob/master/gen2-display-detections/utility.py
    """

    def __init__(self, classes: list[str], colors: list[str], show_class_name: bool = True,
                 show_confidence: bool=True) -> None:
        self.text_type = cv2.FONT_HERSHEY_SIMPLEX
        self.line_type = cv2.LINE_AA

        # A list of classes of the model used for detection
        self.classes = classes
        self.colors = []
        for color in colors:
            self.colors.append(self.hex_to_rgb(color))
        self.show_class_name = show_class_name
        self.show_confidence = show_confidence

    def hex_to_rgb(self, hex_str: str) -> tuple[int, int, int]:
        """Convert the hex string passed in by the args into a tuple representing the corresponding rgb color."""
        return tuple(int(hex_str[i:i+2], 16) for i in (0, 2, 4))

    def putText(self, frame: np.array, text: str, coords: list[float], color: tuple[int, int, int]) -> None:
        """Add text to frame, such as class label or confidence value."""
        (w, h), _ = cv2.getTextSize(text, self.text_type, 0.75, 2)
        # places the text labeling the class and/or confidence value of the bbox
        if coords[1]-h-10 > 0:
            # text is placed above the top left corner of the bbox by default
            new_coords = (coords[0], coords[1]-10)
            startpoint = (new_coords[0], new_coords[1]-h)
            endpoint = (new_coords[0] + w, new_coords[1]+10)
        else:
            # if there is not enough space above the top left corner of the bbox then
            # the text is placed right below the top left corner, within the bbox
            new_coords = (coords[0], coords[1]+h)
            startpoint = (new_coords[0],  new_coords[1]-h)
            endpoint = (new_coords[0] + w, new_coords[1])
        cv2.rectangle(frame, startpoint, endpoint, color, -1)
        cv2.putText(frame, text, new_coords, self.text_type, 0.75, (255, 255, 255), 2, self.line_type)

    def rectangle(self, frame: np.array, bbox: tuple[float, float, float, float], color: tuple[int, int, int]) -> None:
        """Add a rectangle to frame, such as a bounding box."""
        x1, y1, x2, y2 = bbox
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 3)

    def frame_norm(self, frame: np.array, bbox: tuple[float, float, float, float]) -> int:
        """Normalize bbox locations between frame width/height."""
        norm_vals = np.full(len(bbox), frame.shape[0])
        norm_vals[::2] = frame.shape[1]
        return (np.clip(np.array(bbox), 0, 1) * norm_vals).astype(int)

    def visualize_detections(self, frame: np.array, detections: list[object]) -> np.array:
        """Return frame with bounding boxes, classes, and labels of each detection overlaid."""
        frame_copy = frame.copy()

        for detection in detections:
            bbox = self.frame_norm(frame_copy, (detection.xmin, detection.ymin, detection.xmax, detection.ymax))
            # the code below specifies whether to display the bbox's class name and/or confidence value
            if self.show_class_name and self.show_confidence:
                self.putText(frame_copy, f'{self.classes[detection.label]} {int(detection.confidence * 100)}%',
                             (bbox[0], bbox[1]), self.colors[detection.label])
            elif self.show_class_name and not self.show_confidence:
                self.putText(frame_copy, self.classes[detection.label],
                             (bbox[0], bbox[1]), self.colors[detection.label])
            elif not self.show_class_name and self.show_confidence:
                self.putText(frame_copy, f'{int(detection.confidence * 100)}%',
                             (bbox[0], bbox[1]), self.colors[detection.label])

            self.rectangle(frame_copy, bbox, self.colors[detection.label])

        return frame_copy
