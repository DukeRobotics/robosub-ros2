import math
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from sklearn.cluster import DBSCAN
from sklearn.linear_model import LinearRegression
from sklearn.mixture import GaussianMixture

from sonar import decode_ping_python_360

SONAR_IMAGE_WIDTH = 16
SONAR_IMAGE_HEIGHT = 2

def build_color_sonar_image_from_int_array(int_array: list, npy_save_path: str | None = None,
                                           jpeg_save_path: str | None = None) -> np.ndarray:
    """
    Build a sonar image from a list of data messages.

    Args:
        int_array (List): List of data messages from either the Sonar device
            or from a .bin file.
        npy_save_path (str, optional): Path to save the sonar image as a
            .npy file. Defaults to None.
        jpeg_save_path (str, optional): Path to save the sonar image as a
            .jpeg file. Defaults to None.

    Returns:
        ndarray: Sonar image from the scan.
    """
    sonar_img = cv2.cvtColor(int_array.astype(np.uint8), cv2.COLOR_GRAY2BGR)
    sonar_img = cv2.applyColorMap(sonar_img, cv2.COLORMAP_VIRIDIS)
    if jpeg_save_path:
        plt.imsave(jpeg_save_path, sonar_img)
    if npy_save_path:
        np.save(npy_save_path, sonar_img)

    return sonar_img

def find_center_point_and_angle(array: np.ndarray, threshold: int, eps: float,
                                min_samples: int, get_plot: bool = True) -> tuple:
    """
    Find the center point and angle of the largest cluster in the array.

    Args:
        array (ndarray): The sonar image array.
        threshold (int): Threshold to apply to the array.
        eps (float): Maximum distance between two samples for one to be
            considered as in the neighborhood of the other.
        min_samples (int): Number of samples in a neighborhood for a point
            to be considered as a core point.
        get_plot (bool, optional): Whether to return the plot of the results.
            Defaults to True.

    Returns:
        float: The average column index of the largest cluster.
        float: The angle of the largest cluster.
        ndarray: The sonar image with the results plotted.
    """
    if get_plot:
        plt.figure(figsize=(SONAR_IMAGE_WIDTH, SONAR_IMAGE_HEIGHT))
        plt.imshow(array, cmap='viridis', aspect='auto')

    # Convert values > threshold to list of points
    points = np.argwhere(array > threshold)
    if points.size == 0:
        return None, None, None

    # Perform DBSCAN clustering
    db = DBSCAN(eps=eps, min_samples=min_samples).fit(points)
    labels = db.labels_

    # Get cluster with the most points
    unique_labels = set(labels)
    cluster_counts = {k: np.sum(labels == k) for k in unique_labels if k != -1}

    if not cluster_counts:
        return None, None, None

    # Get the points of the largest cluster and calculate the average column index
    largest_cluster_label = max(cluster_counts, key=cluster_counts.get)
    class_member_mask = (labels == largest_cluster_label)
    max_clust_points = points[class_member_mask]
    average_column_index = np.mean(max_clust_points[:, 1])

    # Linear regression to find slope
    x = max_clust_points[:, 0].reshape(-1, 1)
    y = max_clust_points[:, 1]
    linreg_sklearn = LinearRegression()
    linreg_sklearn.fit(x, y)

    slope_sklearn = linreg_sklearn.coef_[0]
    intercept_sklearn = linreg_sklearn.intercept_
    angle = math.degrees(math.atan(slope_sklearn))

    # Plot the results
    if get_plot:
        x_vals_plot = np.arange(array.shape[0])
        y_vals_plot = intercept_sklearn + slope_sklearn * x_vals_plot
        plt.plot(
            y_vals_plot, x_vals_plot, 'r', linewidth=4,
            label=f'Line: y = {slope_sklearn:.2f}x + {intercept_sklearn:.2f}',
        )
        plt.scatter(
            average_column_index, array.shape[0]/2,
            color='k', s=150, zorder=3, label='Center Point',
        )
        plt.xticks([])
        plt.yticks([])
        plt.subplots_adjust(top=1, bottom=0, right=1, left=0, hspace=0, wspace=0)

        fig = plt.gcf()
        canvas = FigureCanvas(fig)
        canvas.draw()

        image = np.frombuffer(canvas.tostring_argb(), dtype='uint8')
        image = image.reshape((*fig.canvas.get_width_height()[::-1], 4))
        array = cv2.cvtColor(image[:, :, 1:], cv2.COLOR_RGB2BGR)

    return average_column_index, angle, array

def build_sonar_img_from_log_file(filename: str, start_index: int = 49, end_index: int = 149) -> np.ndarray:
    """
    Build a sonar image from a .bin file.

    Args:
        filename (str): Path to the .bin file.
        start_index (int, optional): The index to start building the sonar image.
            Defaults to 49.
        end_index (int, optional): The index to stop building the sonar image.
            Defaults to 149.

    Returns:
        ndarray: Sonar image from the scan.
    """
    assert filename.endswith('.bin'), 'filename must be a .bin file'

    parser = decode_ping_python_360.get_bin_file_parser(filename)

    data_list = []
    for index, (_, decoded_message) in enumerate(parser):
        if start_index <= index <= end_index:
            data_list.append(decoded_message.data)

    jpeg_save_path = Path(__file__).parent / 'sampleData' / 'Sonar_Image.jpeg'

    return build_sonar_image(data_list, display_results=True, jpeg_save_path=jpeg_save_path)

def build_sonar_image(data_list: list, display_results: bool = False, npy_save_path: str | None = None,
                      jpeg_save_path: str | None = None) -> np.ndarray:
    """
    Build a sonar image from a list of data messages.

    Args:
        data_list (List): List of data messages from either the Sonar device
            or from a .bin file.
        display_results (bool, optional): Whether to display the resulting
            sonar image. Defaults to False.
        npy_save_path (str, optional): Path to save the sonar image as a
            .npy file. Defaults to None.
        jpeg_save_path (str, optional): Path to save the sonar image as a
            .jpeg file. Defaults to None.

    Returns:
        ndarray: Sonar image from the scan.
    """
    sonar_img = None
    for data in data_list:
        # Split the data into individual bytes and filter out noise
        split_bytes = [data[i:i+1] for i in range(len(data))]
        split_bytes = split_bytes[100:]

        byte_val = int.from_bytes(split_bytes[0], 'big')
        intarray = np.array([byte_val])

        for i in range(len(split_bytes) - 1):
            byte_val = int.from_bytes(split_bytes[i+1], 'big')
            intarray = np.append(intarray, [byte_val])

        sonar_img = np.asarray(intarray) if sonar_img is None else np.vstack((sonar_img, intarray))

    sonar_img = sonar_img.astype(np.uint8)

    if jpeg_save_path:
        plt.imsave(jpeg_save_path, sonar_img)
    if npy_save_path:
        np.save(npy_save_path, sonar_img)

    if display_results:
        cv2.imshow('sonar_img', sonar_img)
        cv2.waitKey(0)

    return sonar_img

def sonar_gaussian_mixture_model_cluster(sonar_data: np.ndarray) -> np.ndarray:
    """
    Cluster a sonar scan into background, walls, and buoys using GMM clustering. Adapted from Pranav Bijith's GMM code.

    Args:
        sonar_data (ndarray): a sonar scan in cartesian coordinates which may contain nothing, walls, and buoys

    Returns:
        ndarray: ndarray of sonar_data segmented into three categories: nothing, walls, and buoys
    """
    finalcopygrid = sonar_data
    finalcopygrid[finalcopygrid != 0] = 255
    h, w = finalcopygrid.shape
    x = np.column_stack((finalcopygrid.reshape(-1), np.repeat(np.arange(h), w), np.tile(np.arange(w), h)))
    mask = finalcopygrid.reshape(-1) != 0
    x_masked = x[mask]

    if x_masked.shape[0] >= 2:
        gmm = GaussianMixture(n_components=2, random_state=42)
        gmm.fit(x_masked)
        cluster_labels = np.full(mask.shape, -1)
        cluster_labels[mask] = gmm.predict(x_masked)
    else:
        print('No nonzero pixels found — skipping GMM')
        cluster_labels = np.full(mask.shape, -1)

    return cluster_labels.reshape(finalcopygrid.shape)

