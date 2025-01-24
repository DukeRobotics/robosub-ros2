import numpy as np


class mono_cam:
    """Mono cam sensor and frame constants."""
    IMG_SHAPE = (640, 480)  # Width, height in pixels
    SENSOR_SIZE = (3.054, 1.718)  # Width, height in mm
    FOCAL_LENGTH = 2.65  # Focal length in mm

class bins:
    """Bin dimension and color constants."""
    WIDTH = 0.3048  # width of one square of the bin, in m

    # Define the range for HSV filtering on the red bin
    lower_red_low = np.array([0, 110, 150])
    upper_red_low = np.array([12, 255, 255])
    lower_red_high = np.array([170, 50, 85])
    upper_red_high = np.array([179, 255, 255])

    # Define the range for HSV filtering on the blue bin
    lower_blue = np.array([90, 150, 50])
    upper_blue = np.array([125, 255, 255])


class buoy:
    """Buoy dimension and color constants."""
    WIDTH = 0.2032  # Width of buoy in meters

class blue_rect:
    lower_blue = np.array([100, 150, 50])
    upper_blue = np.array([140, 255, 255])