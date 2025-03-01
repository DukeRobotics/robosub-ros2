from enum import Enum

import numpy as np


class MonoCam:
    """Mono cam sensor and frame constants."""
    IMG_SHAPE = (640, 480)  # Width, height in pixels
    SENSOR_SIZE = (3.054, 1.718)  # Width, height in mm
    FOCAL_LENGTH = 2.65  # Focal length in mm

class Bins:
    """Bin dimension and color constants."""
    WIDTH = 0.3048  # width of one square of the bin, in m

    # Define the range for HSV filtering on the red bin
    RED_LOW_BOT = np.array([0, 110, 150])
    RED_LOW_TOP = np.array([12, 255, 255])
    RED_HIGH_BOT = np.array([170, 50, 85])
    RED_HIGH_TOP = np.array([179, 255, 255])

    # Define the range for HSV filtering on the blue bin
    lower_blue = np.array([90, 150, 50])
    upper_blue = np.array([125, 255, 255])

class Buoy:
    """Buoy dimension and color constants."""
    WIDTH = 0.2032  # Width of buoy in meters

class LaneMarker:
    """Lane marker color constants."""
    LANE_MARKER_BOT = np.array([100, 150, 50])
    LANE_MARKER_TOP = np.array([140, 255, 255])
class Torpoedo:
    """Torpedo dimension and color constants."""
    WIDTH = 0.2032 # TODO update this when dimensions are released  # noqa: TD004

class BlueRect:
    """BlueRect color constants."""
    BLUE_BOT = np.array([100, 150, 50])
    BLUE_TOP = np.array([140, 255, 255])

class PathMarker:
    """Path marker color bounds and contour parameters."""
    ORANGE_BOT = np.array([0, 130, 100])
    ORANGE_TOP = np.array([20, 255, 255])

    MIN_CONTOUR_LENGTH = 5
    MIN_CONTOUR_AREA = 500

class PinkBins:
    """Pink bins color bounds."""
    PINK_1_BOT = np.array([110, 50, 130])
    PINK_1_TOP = np.array([130, 100, 200])
    PINK_2_BOT = np.array([130, 80, 130])
    PINK_2_TOP = np.array([160, 150, 255])
    PINK_3_BOT = np.array([155, 100, 150])
    PINK_3_TOP = np.array([175, 255, 255])

class USBCamera(Enum):
    """Enum for different USB cameras."""
    front = 0
    bottom = 1
