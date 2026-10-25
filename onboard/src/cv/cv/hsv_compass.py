import cv2
import numpy as np
import rclpy

import cv.config as cv_constants
from cv import hsv_filter


class HSVCompass(hsv_filter.HSVFilter):
    """Detect the yellow circular compass emoji on the front USB camera."""

    def __init__(self) -> None:
        super().__init__(
            name='compass',
            camera='front',
            mask_ranges=[
                [cv_constants.Compass.YELLOW_BOT, cv_constants.Compass.YELLOW_TOP],
            ],
            width=cv_constants.Compass.WIDTH,
        )

    def filter(self, contours: list) -> list:
        """Keep the largest yellow contour that looks circular."""
        candidates = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < cv_constants.Compass.MIN_CONTOUR_AREA:
                continue

            (_, _), radius = cv2.minEnclosingCircle(contour)
            if radius <= 0:
                continue

            circularity = area / (np.pi * radius * radius)
            if circularity < cv_constants.Compass.MIN_CIRCULARITY:
                continue

            candidates.append((area, contour))

        if not candidates:
            return []

        candidates.sort(key=lambda item: item[0], reverse=True)
        return [candidates[0][1]]

    def morphology(self, mask: np.ndarray) -> np.ndarray:
        """Clean up the yellow mask."""
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)


def main(args: list[str] | None = None) -> None:
    """Run the node."""
    rclpy.init(args=args)
    node = HSVCompass()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
