import cv.config as cv_constants
import cv2
import numpy as np
import rclpy
from cv import hsv_filter


class HSVLaneMarker(hsv_filter.HSVFilter):
    """HSV Lane Marker Detector."""
    def __init__(self) -> None:
        super().__init__(
            name='lane_marker',
            camera='bottom',
            mask_ranges=[
                [cv_constants.LaneMarker.LANE_MARKER_BOT, cv_constants.LaneMarker.LANE_MARKER_TOP],
            ],
        )

    def create_additional_pubs_subs_vars(self) -> None:
        """Additional publishers and subscribers specific to this class."""
        # TODO: implement this to add publishers from lane_marker_detector.py that are missing in the default HSVFilter

    def process_contours(self, final_contours: list[np.ndarray], image: np.ndarray, bbox_img: np.ndarray) -> None:
        """Process lane marker contour."""
        contour = final_contours[0]
        # TODO: implement this to match behavior of lane_marker_detector.py perfectly

    def filter(self, contours: list) -> list:
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
    hsv_filter = HSVLaneMarker()

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
