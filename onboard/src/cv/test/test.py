import os
import cv2
import numpy as np
from functools import reduce
from sklearn.cluster import DBSCAN

# --- PinkBins and MonoCam configs (you can adjust these as needed) ---

class PinkBins:
    PINK_1_BOT = np.array([160, 50, 50])
    PINK_1_TOP = np.array([180, 255, 255])
    PINK_2_BOT = np.array([145, 50, 50])
    PINK_2_TOP = np.array([160, 255, 255])
    PINK_3_BOT = np.array([130, 50, 50])
    PINK_3_TOP = np.array([145, 255, 255])
    PINK_4_BOT = np.array([215, 12, 40])
    PINK_4_TOP = np.array([280, 33, 50])

class MonoCam:
    IMG_SHAPE = (640, 480)  # adjust based on your setup

# --- process_frame function adapted from pink_bins_detector.py ---

def process_frame(frame: np.array, camera: str = 'front') -> np.array:
    """Process a frame and return a visualization of the detection result (no ROS)."""

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Define pink masks
    mask_1 = cv2.inRange(hsv, PinkBins.PINK_1_BOT, PinkBins.PINK_1_TOP)
    mask_2 = cv2.inRange(hsv, PinkBins.PINK_2_BOT, PinkBins.PINK_2_TOP)
    mask_3 = cv2.inRange(hsv, PinkBins.PINK_3_BOT, PinkBins.PINK_3_TOP)
    mask_4 = cv2.inRange(hsv, PinkBins.PINK_4_BOT, PinkBins.PINK_4_TOP)

    if camera == 'bottom':
        mask = cv2.inRange(hsv, np.array([160, 150, 200]), np.array([170, 255, 255]))
    else:
        mask = reduce(cv2.bitwise_or, [mask_1, mask_2, mask_3, mask_4])

    # If no pink found, return filtered mask in color
    points = np.argwhere(mask > 0)
    if len(points) == 0:
        return cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

    dbscan_img = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

    db = DBSCAN(eps=2, min_samples=10).fit(points)
    labels = db.labels_
    unique_labels = set(labels)
    cluster_counts = {k: np.sum(labels == k) for k in unique_labels if k != -1}
    max_cluster_size = max(cluster_counts.values()) if cluster_counts else 0
    print("Max cluster size: ", max_cluster_size)

    if not cluster_counts:
        return dbscan_img  # No valid clusters

    sorted_cluster_labels = sorted(cluster_counts, key=cluster_counts.get, reverse=True)
    final_x, final_y = 0, 0

    MIN_CLUSTER_SIZE = 100
    THRESHOLD_RATIO = 0.5

    for label in sorted_cluster_labels[:3]:
        class_member_mask = (labels == label)
        max_clust_points = points[class_member_mask]

        for point in max_clust_points:
            dbscan_img[point[0], point[1]] = (0, 255, 0)

        center_x = np.mean(max_clust_points[:, 1])
        center_y = np.mean(max_clust_points[:, 0])
        point_count = max_clust_points.shape[0]
        print(point_count)

        if point_count < THRESHOLD_RATIO * max_cluster_size or point_count < MIN_CLUSTER_SIZE:
            continue

        if center_y > final_y:
            final_x, final_y = center_x, center_y

    # Draw red circle on detected point
    if (final_x != -1 and final_y != -1):
        final_point_int = (int(final_x), int(final_y))
        cv2.circle(dbscan_img, final_point_int, 7, (0, 0, 255), -1)

    return dbscan_img

# --- main loop ---

def main():
    image_dir = os.path.join(os.path.dirname(__file__), 'images')
    image_files = [f for f in os.listdir(image_dir) if f.lower().endswith('.png')]

    for filename in image_files:
        image_path = os.path.join(image_dir, filename)
        frame = cv2.imread(image_path)

        if frame is None:
            print(f"Failed to load {filename}")
            continue

        print(f"Processing: {filename}")
        processed = process_frame(frame)

        # Resize for better viewing if needed
        disp_orig = cv2.resize(frame, (640, 480))
        disp_proc = cv2.resize(processed, (640, 480))
        combined = cv2.hconcat([disp_orig, disp_proc])

        cv2.imshow("Original (left) | Processed (right)", combined)
        key = cv2.waitKey(0)
        if key == ord('q'):
            break

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
