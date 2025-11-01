import math
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_agg import FigureCanvasAgg as FigureCanvas
from sklearn.cluster import DBSCAN
from sklearn.linear_model import LinearRegression

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

def fourier_signal_processing(
        data: np.ndarray,
        inner_radius: float,
        outer_radius: float,
        threshold: float
        ) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Denoise a sonar scan using the Fast Fourier Transform.

    Args:
        data (ndarray): an ndarray representing the sonar data
        inner_radius (float): the radius of a circle in the frequency domain, all signal within will be removed
        outer_radius (float): the radius of a circle in the frequency domain, all signal without will be removed
        threshold (float): the threshhold

    Returns:
        Tuple[ndarray, ndarray, ndarray]: tuple containing the filtered gradian image, cartesian image and the angle
        domain of the cartesian image, respectively
    """
    xv, yv = np.meshgrid(np.fft.fftfreq(data.shape[1]), np.fft.fftfreq(data.shape[0]))
    xv = np.fft.fftshift(xv)
    yv = np.fft.fftshift(yv)
    fftimg = np.fft.fftshift(np.fft.fft2(data))

    #Applies the Radial Mask
    radius = np.sqrt(xv**2 + yv**2)
    mask = (radius < outer_radius) & (radius >= inner_radius)
    mask = mask.astype(np.float32)
    if data.ndim == 3 and data.shape[2] == 3:
        mask = np.repeat(mask[:, :, np.newaxis], 3, axis=2)
    fimg = np.fft.fftshift(np.fft.fft2(data, axes=(0, 1))) * mask

    #Filter
    filtered = np.fft.ifft2(np.fft.ifftshift(fimg))
    filtered = np.abs(filtered)
    filtered[filtered < threshold] = 0

    #Convert to Cartesian
    if filtered.ndim == 3:
        num_theta, num_radius, num_channels = filtered.shape
    else:
        num_theta, num_radius = filtered.shape
        num_channels = 1
    x = np.arange(num_radius)
    y = np.arange(num_radius)
    x, y = np.meshgrid(x, y)
    angles = np.arctan2(y, x)
    theta_idx = np.clip(np.round((angles / (np.pi / 2)) * (num_theta - 1)).astype(int), 0, num_theta-1)
    r = np.clip(np.round(np.sqrt(x**2 + y**2)).astype(int), 0, num_radius-1)
    if num_channels > 1:
        cartesian_grid = np.zeros((num_radius, num_radius, num_channels))
        for c in range(num_channels):
            cartesian_grid[:, :, c] = filtered[theta_idx, r, c]
    else:
        cartesian_grid = filtered[theta_idx, r]

    cartesian_grid = np.clip(
        np.nan_to_num(cartesian_grid, nan=0.0, posinf=0.0, neginf=0.0),
        0,
        np.percentile(cartesian_grid, 99),
    )

    #Return Cartesian grid and frequency domain
    return (filtered, cartesian_grid, np.abs(fftimg))

