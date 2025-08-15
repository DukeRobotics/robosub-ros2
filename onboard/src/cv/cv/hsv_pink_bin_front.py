import cv2
import numpy as np
import rclpy

import cv.config as cv_constants
from cv import hsv_filter
from scipy.spatial.distance import cdist


class HSVPinkBinFront(hsv_filter.HSVFilter):
    """Parent class for all HSV filtering scripts."""
    def __init__(self) -> None:
        super().__init__(
            name='bin_pink_front',
            camera='front',
            mask_ranges=[
                [cv_constants.YellowBins.YELLOW_2_BOT, cv_constants.YellowBins.YELLOW_2_TOP],
                # [cv_constants.YellowBins.YELLOW_3_BOT, cv_constants.YellowBins.YELLOW_3_TOP],
            ],
            width=cv_constants.Bins.WIDTH,
        )

    def group_contours_by_distance(self, contours, dist_thresh):
        centers = []
        valid_contours = []
        for contour in contours:
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                centers.append([cx, cy])
                valid_contours.append(contour)

        centers = np.array(centers)
        n = len(centers)
        if n == 0:
            return []

        dist_matrix = cdist(centers, centers)
        groups = []
        visited = set()

        for i in range(n):
            if i in visited:
                continue
            group = {i}
            neighbors = set(np.where(dist_matrix[i] < dist_thresh)[0])
            group = group.union(neighbors)

            expanded = True
            while expanded:
                expanded = False
                new_neighbors = set()
                for idx in group:
                    idx_neighbors = set(np.where(dist_matrix[idx] < dist_thresh)[0])
                    if not idx_neighbors.issubset(group):
                        new_neighbors = new_neighbors.union(idx_neighbors.difference(group))
                        expanded = True
                group = group.union(new_neighbors)

            visited = visited.union(group)
            groups.append(list(group))

        merged_contours = []
        for group_indices in groups:
            merged_points = np.vstack([valid_contours[idx] for idx in group_indices])
            merged_contours.append(merged_points)

        return merged_contours

    def filter(self, contours: list) -> list:
        """Pick the largest and lowest contour only."""
        MIN_AREA = 100 # Minimum area of contour to be valid
        THRESHOLD_RATIO = 0.75 # Contour within scaled of max contour area
        DIST_THRESH = 25 # Group all contours within this threshold

        final_contours = sorted(contours, key=cv2.contourArea, reverse=True)

        #self.get_logger().info(f'final count {len(final_contours)}')
        grouped_contours = self.group_contours_by_distance(final_contours[:min(20,len(final_contours))], dist_thresh=DIST_THRESH)

        final_x, final_y = 0, 0
        chosen_contour_score = None
        chosen_contour = None

        max_coutour_area = 0
        for contour in grouped_contours:
            max_coutour_area = max(max_coutour_area, cv2.contourArea(contour))

        #self.get_logger().info(f'grouped count {len(grouped_contours)}')
        for contour in grouped_contours:
            # Get center (mean of all contour points)
            M = cv2.moments(contour)
            if M["m00"] != 0:
                center_x = int(M["m10"] / M["m00"])
                center_y = int(M["m01"] / M["m00"])
            else:
                continue

            cluster_point_count = cv2.contourArea(contour)
            if cluster_point_count < THRESHOLD_RATIO * max_coutour_area or cluster_point_count < MIN_AREA:
                continue

            # Pick the contour with the lowest center_y
            if center_y > final_y:
                final_x, final_y = center_x, center_y
                chosen_contour_score = cluster_point_count
                chosen_contour = contour

        if chosen_contour_score is not None and chosen_contour_score >= MIN_AREA:
            # Draw chosen contour center in red
            # self.get_logger().info(f'Octagon bin score {chosen_contour_score}')
            return chosen_contour

        return None

    def morphology(self, mask: np.ndarray) -> np.ndarray:
        """Apply a kernel morphology."""
        kernel = np.ones((3, 3), np.uint8)
        return cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

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
