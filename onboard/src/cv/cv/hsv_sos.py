import cv2
import numpy as np
import rclpy

import cv.config as cv_constants
from cv import hsv_filter


class HSVSos(hsv_filter.HSVFilter):
    """Detect the red square SOS emoji on the front USB camera."""

    def __init__(self) -> None:
        super().__init__(
            name='sos',
            camera='front',
            mask_ranges=[
                [cv_constants.Sos.RED_LOW_BOT, cv_constants.Sos.RED_LOW_TOP],
                [cv_constants.Sos.RED_HIGH_BOT, cv_constants.Sos.RED_HIGH_TOP],
            ],
            width=cv_constants.Sos.WIDTH,
        )

    def filter(self, contours: list) -> list:
        """Keep the largest red contour that looks roughly square and filled."""
        candidates = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < cv_constants.Sos.MIN_CONTOUR_AREA:
                continue

            _, _, w, h = cv2.boundingRect(contour)
            if w <= 0 or h <= 0:
                continue

            aspect = min(w, h) / max(w, h)
            if aspect < cv_constants.Sos.MIN_ASPECT_RATIO:
                continue

            solidity = area / float(w * h)
            if solidity < cv_constants.Sos.MIN_SOLIDITY:
                continue

            candidates.append((area, contour))

        if not candidates:
            return []

        candidates.sort(key=lambda item: item[0], reverse=True)
        return [candidates[0][1]]

    def morphology(self, mask: np.ndarray) -> np.ndarray:
        """Clean up the red mask."""
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)


def main(args: list[str] | None = None) -> None:
    """Run the node."""
    rclpy.init(args=args)
    node = HSVSos()

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
