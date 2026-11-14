from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np

from sonar import decode_ping_python_360


def build_color_sonar_image_from_int_array(int_array: np.ndarray, npy_save_path: str | None = None,
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
    int_array = np.nan_to_num(int_array, nan=0.0, posinf=0.0, neginf=0.0)
    sonar_img = np.stack([int_array, int_array, int_array], axis=-1)
    if jpeg_save_path:
        plt.imsave(jpeg_save_path, sonar_img)
    if npy_save_path:
        np.save(npy_save_path, sonar_img)

    sonar_img = sonar_img - np.min(sonar_img)
    max_val = np.max(sonar_img)
    sonar_img = np.zeros_like(sonar_img) if max_val == 0 else sonar_img / max_val * 255

    return sonar_img.astype(np.uint8)

def draw_label_overlay(color_image: np.ndarray, labels: np.ndarray) -> np.ndarray:
    """Tint Cartesian labeled regions and draw their numeric IDs on a copy of a BGR image."""
    overlay = color_image.copy()
    for label_id in np.unique(labels):
        if label_id <= 0:
            continue
        mask = labels == label_id
        hue = int(label_id * 37) % 180
        color = cv2.cvtColor(np.uint8([[[hue, 220, 255]]]), cv2.COLOR_HSV2BGR)[0, 0]
        overlay[mask] = (0.5 * overlay[mask] + 0.5 * color).astype(np.uint8)
        rows, cols = np.nonzero(mask)
        # Anchor inside the region, even when a curved object's centroid lies outside it.
        nearest = np.argmin((rows - rows.mean()) ** 2 + (cols - cols.mean()) ** 2)
        origin = (int(cols[nearest]), int(rows[nearest]))
        text = str(int(label_id))
        cv2.putText(overlay, text, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(overlay, text, origin, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    return overlay


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
