import cv2
import numpy as np
import rclpy

import cv.config as cv_constants
from cv import hsv_filter


class HSVRedPole(hsv_filter.HSVFilter):
    """Detect red slalom poles using HSV filtering."""
    def __init__(self) -> None:
        super().__init__(
            name='pole_red',
            camera='front',
            mask_ranges=[
                [cv_constants.SlalomPole.RED_LOW_BOT, cv_constants.SlalomPole.RED_LOW_TOP],

                [cv_constants.SlalomPole.RED_HIGH_BOT, cv_constants.SlalomPole.RED_HIGH_TOP],
            ],
            width=cv_constants.SlalomPole.WIDTH,

            # Monocular depth estimate depends on the "known height" here, so this is critical
            # If we don't move to stereo vision, don't want to hardcode height in case robot is rolling/pitching.
            height=cv_constants.SlalomPole.HEIGHT,

            # These are 3 publishers for the three red poles.
            # For actual use, we probably only need the first, closest one.
            pubs=['pole_1', 'pole_2', 'pole_3'],
        )

    def filter(self, contours: list) -> list:
        """Filter contours to keep only those that are likely red slalom poles."""
        min_area = 50
        kept = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < min_area:
                continue

            _x, _y, w, h = cv2.boundingRect(contour)
            if w <= 0 or h <= 0: # discard contours with non-positive dimensions
                continue

            aspect = float(w) / float(h)
            # discard contours that are too fat
            if aspect > (1/10): # Aspect ratio of poles is supposed to be 1:30,
                                # but should have some padding since filter probably wont pick up the whole thing
                continue

            kept.append(contour)

        if not kept:
            return []

        kept = sorted(kept, key=cv2.contourArea, reverse=True)
        return kept[:3] # probably only need first, closest pole.
                        # In fact, smaller poles in distance may confuse task planning.

    def morphology(self, mask: np.ndarray) -> np.ndarray:
        """Apply morphological operations to clean up the HSV mask."""
        # Rectangular kernel to close gaps and remove noise
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 20))

        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)        #   Close small gaps in the mask
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)         #   Remove noise from the mask

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
