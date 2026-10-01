import cv2
import numpy as np
import rclpy

import cv.config as cv_constants
from cv import hsv_filter


class HSVRedBin(hsv_filter.HSVFilter):
    """Parent class for all HSV filtering scripts."""
    def __init__(self) -> None:
        super().__init__(
            name='pole_red',
            camera='bottom',                                                    #   FIX: Check which cam to use for pole detection
            mask_ranges=[
                [cv_constants.Bins.RED_LOW_BOT, cv_constants.Bins.RED_LOW_TOP], #   FIX:   These ranges were used for red bin. 
                                                                                #   Modify constants in cv.config for pole detection
                [cv_constants.Bins.RED_HIGH_BOT, cv_constants.Bins.RED_HIGH_TOP],
            ],
            width=cv_constants.Bins.WIDTH,                                      #   FIX: Check the width constant for pole detection
        )

    def filter(self, contours: list) -> list:                                   #   CHECK: Largest contour is likely fine for this node, 
                                                                                #   but test on video data to see what contours look like first
        """Pick the largest contour only."""
        final_contours = sorted(contours, key=cv2.contourArea, reverse=True)
        return final_contours[0]

    def morphology(self, mask: np.ndarray) -> np.ndarray:
        """Apply a kernel morphology."""
        kernel = np.ones((5, 5), np.uint8)
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

def main(args: list[str] | None = None) -> None:
    """Run the node."""
    rclpy.init(args=args)
    hsv_filter = HSVRedBin()

    try:
        rclpy.spin(hsv_filter)
    except KeyboardInterrupt:
        pass
    finally:
        hsv_filter.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
