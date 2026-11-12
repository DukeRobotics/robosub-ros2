import cv2
import numpy as np
import rclpy

import cv.config as cv_constants
from cv import hsv_filter


class HSVRedPole(hsv_filter.HSVFilter):
    """Parent class for all HSV filtering scripts."""
    def __init__(self) -> None:
        super().__init__(
            name='pole_red',
            camera='front',                                                     #   FIX: Check which cam to use for pole detection. Pretty sure it's front cam
            mask_ranges=[
                [cv_constants.SlalomPole.RED_LOW_BOT, cv_constants.SlalomPole.RED_LOW_TOP], #   FIX:   These ranges were used for red bin. 
                                                                                #   Modify constants in cv.config for pole detection
                [cv_constants.SlalomPole.RED_HIGH_BOT, cv_constants.SlalomPole.RED_HIGH_TOP],
            ],
            width=cv_constants.SlalomPole.WIDTH,                                      
            height=cv_constants.SlalomPole.HEIGHT,                              #   Monocular depth estimate depends on the "known height" here, so this is critical
                                                                                #   If we don't move to stereo vision, probably don't want to hardcode height in case robot is
                                                                                #   rolling/pitching.
                                                                                
            pubs=['pole_1', 'pole_2', 'pole_3'],                                #   These are 3 publishers for the three red poles. For actual use, we probably only need the first, closest one.
        )

    def filter(self, contours: list) -> list:
        kept = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 100: 
                continue 

            x, y, w, h = cv2.boundingRect(contour)
            if w <= 0 or h <= 0:
                continue

            aspect = float(w) / float(h)
            # pole is tall and thin
            if aspect < 0.05 or aspect > 0.6:
                continue

            kept.append(contour)

        if not kept:
            return []

        kept = sorted(kept, key=cv2.contourArea, reverse=True)
        return kept[:3] #probably only need first, closest pole. In fact, smaller poles in distance may confuse task planning.

    def morphology(self, mask: np.ndarray) -> np.ndarray:
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 15))             #   Rectangular kernel to close small gaps and remove noise
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)                  #   Close small gaps in the mask
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)                   #   Remove noise from the mask

def main(args: list[str] | None = None) -> None:
    """Run the node."""
    rclpy.init(args=args)
    hsv_filter = HSVRedPole()

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