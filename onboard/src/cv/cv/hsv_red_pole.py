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
            camera='front',                                                     #   FIX: Check which cam to use for pole detection. Pretty sure it's front cam
            mask_ranges=[
                [cv_constants.Bins.RED_LOW_BOT, cv_constants.Bins.RED_LOW_TOP], #   FIX:   These ranges were used for red bin. 
                                                                                #   Modify constants in cv.config for pole detection
                [cv_constants.Bins.RED_HIGH_BOT, cv_constants.Bins.RED_HIGH_TOP],
            ],
            width=cv_constants.SlalomPole.WIDTH,                                      
            height=cv_constants.SlalomPole.HEIGHT,                              #   Monocular depth estimate depends on the "known height" here, so this is critical
                                                                                #   If we don't move to stereo vision, probably don't want to hardcode height in case robot is
                                                                                #   rolling/pitching.
                                                                                
            pubs=['pole_1', 'pole_2', 'pole_3'],                                #   These are 3 publishers for the three red poles
        )

    def filter(self, contours: list) -> list:                                   #   FIX: May want to filter by aspect ratio for pole detection
                                                                                #   but test on video data to see what contours look like first
                                                                                #   Slalom task seems to have 3 red poles total, so will return largest 3 for now
        """Pick the largest 3 contours for now."""
        final_contours = sorted(contours, key=cv2.contourArea, reverse=True)
        return final_contours[:3]

    def morphology(self, mask: np.ndarray) -> np.ndarray:
        """Apply a kernel morphology."""
        kernel = np.ones((3, 3), np.uint8)                                      #   FIX: Not too sure what sort of morphology masks to use. Closing may merge adjacent poles,
                                                                                #   since poles are thin and tall, arranged in a sort of line, opening may thin them further,
                                                                                #   since aspect ratio of poles is pretty high (about 30:1 height-to-width) Need to test
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


# Dev notes:
# It would be great to set up an automated finetuning loop. We can acquire "labeled CV data" by just running a powerful bounding-box model on existing real footage 
# to create pseudo-ground-truth training data. We can define some sort of custom loss function that fits the task, like just squared loss for bounding-box center XY coords or something?
# Then it would be easy to automate finetuning of morphology or hsv params with Optuna or something. Then we don't have to rely on water tests