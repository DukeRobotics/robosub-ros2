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
    LOWER_BLUE = np.array([90, 150, 50])
    UPPER_BLUE = np.array([125, 255, 255])

class Buoy:
    """Buoy dimension and color constants."""
    WIDTH = 0.2032  # Width of buoy in meters

class LaneMarker:
    """Lane marker color constants."""
    LANE_MARKER_BOT = np.array([100, 150, 50])
    LANE_MARKER_TOP = np.array([140, 255, 255])

class Torpedo:
    """Torpedo dimension and color constants."""
    WIDTH = 0.1016

    # No sun flickering values
    LOW_BOT = np.array([0, 50, 45])
    LOW_TOP = np.array([5, 95, 95])
    HIGH_BOT = np.array([330, 50, 45])
    HIGH_TOP = np.array([360, 95, 95])

class BlueRect:
    """BlueRect color constants."""
    BLUE_BOT = np.array([100, 150, 50])
    BLUE_TOP = np.array([140, 255, 255])

class PathMarker:
    """Path marker color bounds and contour parameters."""
    ORANGE_BOT = np.array([0, 130, 100])
    ORANGE_TOP = np.array([50, 255, 255])

    MIN_CONTOUR_LENGTH = 5
    MIN_CONTOUR_AREA = 500

class PinkBins:
    """Pink bins color bounds."""
    PINK_1_BOT = np.array([0, 0, 30])
    PINK_1_TOP = np.array([15, 70, 100])

    PINK_2_BOT = np.array([260, 0, 50])
    PINK_2_TOP = np.array([360, 70, 100])

    PINK_3_BOT = np.array([190, 0, 40])
    PINK_3_TOP = np.array([250, 30, 70])

    PINK_4_BOT = np.array([215, 12, 40])
    PINK_4_TOP = np.array([280, 33, 50])

class YellowBins:
    """Pink bins color bounds."""
    YELLOW_1_BOT = np.array([20, 60, 60])
    YELLOW_1_TOP = np.array([30, 100, 100])

    YELLOW_2_BOT = np.array([35, 43, 40])
    YELLOW_2_TOP = np.array([60, 95, 90])

    YELLOW_3_BOT = np.array([30, 70, 70])
    YELLOW_3_TOP = np.array([35, 95, 95])

    YELLOW_4_BOT = np.array([60, 45, 35])
    YELLOW_4_TOP = np.array([70, 80, 75])

class USBCamera(Enum):
    """Enum for different USB cameras."""
    front = 0
    bottom = 1
