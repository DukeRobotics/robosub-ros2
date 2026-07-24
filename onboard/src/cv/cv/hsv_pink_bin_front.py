import cv2
import numpy as np
import rclpy

import cv.config as cv_constants
from cv import hsv_filter
from onboard.src.cv.cv import utils


class HSVPinkBinFront(hsv_filter.HSVFilter):
    """Parent class for all HSV filtering scripts."""
    def __init__(self) -> None:
        super().__init__(
            name='bin_pink_front',
            camera='front',
            mask_ranges=[
                [cv_constants.YellowBins.YELLOW_2_BOT, cv_constants.YellowBins.YELLOW_2_TOP],
                [cv_constants.YellowBins.YELLOW_1_BOT, cv_constants.YellowBins.YELLOW_1_TOP],
                [cv_constants.YellowBins.YELLOW_3_BOT, cv_constants.YellowBins.YELLOW_3_TOP],
            ],
            width=cv_constants.Bins.WIDTH,
        )

        self.group_contours_by_distance = utils.group_contours_by_distance

    def filter(self, contours: list) -> list | None:
        """Filter the contours via various criteria."""
        min_area = 85 # Minimum area of contour to be valid
        threshold_ratio = 0.50 # Contour within scaled of max contour area
        dist_thresh = 5 # Group all contours within this threshold

        final_contours = sorted(contours, key=cv2.contourArea, reverse=True)

        grouped_contours = self.group_contours_by_distance(final_contours[:min(20,len(final_contours))],
                                                           dist_thresh=dist_thresh)

        final_x, _ = 0, 0
        chosen_contour_score = None
        chosen_contour = None

        max_coutour_area = 0
        for contour in grouped_contours:
            max_coutour_area = max(max_coutour_area, cv2.contourArea(contour))

        for contour in grouped_contours:
            # Check ratio and sanity check size
            cluster_point_count = cv2.contourArea(contour)
            if cluster_point_count < threshold_ratio * max_coutour_area or cluster_point_count < min_area:
                continue

            # Get center (mean of all contour points)
            m = cv2.moments(contour)
            if m['m00'] != 0:
                center_x = int(m['m10'] / m['m00'])
                center_y = int(m['m01'] / m['m00'])
            else:
                continue

            # Filter out potential upper reflection by exluding top 1/4
            if center_y <= cv_constants.MonoCam.IMG_SHAPE[1]/4 and len(grouped_contours) > 1:
                continue

            # Pick the contour that is right most, in case of yellow cups giving multiple readings
            if center_x > final_x:
                final_x, _ = center_x, center_y
                chosen_contour_score = cluster_point_count
                chosen_contour = contour

        if chosen_contour_score is not None and chosen_contour_score >= min_area:
            # Draw chosen contour center in red
            return chosen_contour

        return None

    def morphology(self, mask: np.ndarray) -> np.ndarray:
        """Apply a kernel morphology."""
        kernel = np.ones((2, 2), np.uint8)
        mask_closed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        return cv2.morphologyEx(mask_closed, cv2.MORPH_OPEN, kernel)

def main(args: list[str] | None = None) -> None:
    """Run the node."""
    rclpy.init(args=args)
    hsv_filter = HSVPinkBinFront()

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
