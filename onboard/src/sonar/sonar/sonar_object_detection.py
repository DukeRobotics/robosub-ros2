from enum import Enum

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import gridspec
from scipy.signal import convolve2d
from skimage.filters import sobel
from skimage.measure import label
from skimage.segmentation import watershed


class SonarDenoiser:
    """Class to denoise sonar scans to prepare them for segmentation and pose estimation."""
    def __init__(self, data: np.ndarray) -> None:
        """
        Construct a SonarDenoising object.

        Args:
            data (ndarray): data in gradian space
        """
        self.data = data
        self.shape_theta = min(100, self.data.shape[0])
        self.shape_radius = self.data.shape[1]

        #Reshape data
        processed_data = np.zeros(shape=(
            100,
            np.floor(self.shape_radius * 1.41421356).astype(int))
        )
        processed_data[:self.shape_theta, :self.shape_radius] = self.data[: self.shape_theta]

        self.data = processed_data
        self.cartesian: np.ndarray

    def wall_block(self, threshold: float = 0.95) -> 'SonarDenoiser':
        """
        Remove signal behind a known wall.

        This follows the justification that any signal behind a known object
        is noise.

        Args:
            threshold (float): the threshold to consider some signal as a "known object".

        Returns:
            SonarDenoiser: returns itself to allow for method chaining.
        """
        for theta in range(self.shape_theta):
            max_along_theta = 0
            for r in range(self.shape_radius):
                if (self.data[theta][r] > max_along_theta * threshold):
                    max_along_theta = self.data[theta][r]
                else:
                    self.data[theta][r] = 0
        return self

    def percentile_filter(self, threshold: float = 0.7) -> 'SonarDenoiser':
        """
        Apply percentile filtering to reduce noise.

        Args:
            threshold (float): the threshold for percentile filtering.

        Returns:
            SonarDenoiser: returns itself to allow for method chaining.
        """
        threshold = float(np.percentile(np.percentile(self.data[self.data > 0], threshold), threshold))
        self.data[self.data < threshold] = 0

        return self

    def fourier_signal_processing(
        self,
        inner_radius: float = 0.001,
        outer_radius: float = 0.25,
        threshold: float = 40,
        ) -> 'SonarDenoiser':
        """
        Denoise a sonar scan using the Fast Fourier Transform. Adapted from Pranav Bijith's Fourier analysis.

        Args:
            data (ndarray): an ndarray representing the sonar data
            inner_radius (float): the radius of a circle in the frequency domain, all signal within will be removed
            outer_radius (float): the radius of a circle in the frequency domain, all signal without will be removed
            threshold (float): the threshhold

        Returns:
            SonarDenoiser: returns itself to allow for method chaining.
        """
        xv, yv = np.meshgrid(np.fft.fftfreq(self.data.shape[1]), np.fft.fftfreq(self.data.shape[0]))
        xv = np.fft.fftshift(xv)
        yv = np.fft.fftshift(yv)

        #Applies the Radial Mask
        radius = np.sqrt(xv**2 + yv**2)
        mask = (radius < outer_radius) & (radius >= inner_radius)
        mask = mask.astype(np.float32)
        if self.data.ndim == 3 and self.data.shape[2] == 3:
            mask = np.repeat(mask[:, :, np.newaxis], 3, axis=2)
        fimg = np.fft.fftshift(np.fft.fft2(self.data, axes=(0, 1))) * mask

        #Filter
        self.data = np.fft.ifft2(np.fft.ifftshift(fimg))
        self.data = np.abs(self.data)
        self.data[self.data < threshold] = 0

        #Return self
        return self

    def init_cartesian(self) -> 'SonarDenoiser':
        """
        Update cartesian data based on gradian data.

        Returns:
            SonarDenoiser: returns itself to allow for method chaining.
        """
        shape_array = np.arange(self.shape_radius)
        x, y = np.meshgrid(shape_array, shape_array)

        theta = np.zeros(shape=(self.shape_radius, self.shape_radius), dtype=x.dtype)
        theta[:, 0] = 89 # x=0
        theta[:, 1:] = np.arctan(y[:, 1:] / x[:, 1:]) / np.pi * 180
        theta_gradians = (theta / 90 * 100).astype(int)
        r = (np.floor(np.sqrt(x ** 2 + y ** 2))).astype(int)

        self.cartesian = self.data[theta_gradians, r]
        return self

    def normalize(self) -> 'SonarDenoiser':
        """
        Normalize the cartesian image.

        Returns:
            SonarDenoiser: returns itself to allow for method chaining.
        """
        self.cartesian = self.cartesian - np.min(self.cartesian)
        self.cartesian = self.cartesian / np.max(self.cartesian)

        return self

    def blur(self, factor: int = 16) -> 'SonarDenoiser':
        """
        Apply box blur onto cartesian image.

        Args:
            factor (int): the size of the box for the box blur.

        Returns:
            SonarDenoiser: returns itself to allow for method chaining.
        """
        blur_kernel = np.ones((factor, factor), np.float32) / (factor**2)
        self.cartesian = convolve2d(self.cartesian, blur_kernel)

        self.normalize()

        self.cartesian = np.where(self.cartesian > 1/5, self.cartesian, 0)

        return self

class OrthogonalRegression:
    """A class representing the Orthogonal Regression of a group of points."""
    def __init__(self, points: np.ndarray) -> None:
        """
        Construct a OrthogonalRegression object given a group of points.

        Args:
            points (ndarray): the points within this orthogonal regression. Points should be (y, x)
        """
        self.points = points

        e_val, e_vect = np.linalg.eig(np.cov(self.points, rowvar=False))

        self.unit_tangent = e_vect[:, np.argmin(e_val)]
        self.unit_tangent[1] *= -1
        self.unit_normal = np.array([-self.unit_tangent[1], self.unit_tangent[0]])

        if self.unit_tangent[0] == 0:
            self.slope = 999
        else:
            self.slope = self.unit_tangent[1] / self.unit_tangent[0]

        self.intercept = self.points[:,0].mean() - self.slope * self.points[:,1].mean()

        self.orthogonal_projections = np.matmul(
            np.dot(self.points - np.array([self.intercept, 0]), self.unit_tangent[::-1])[:, np.newaxis],
            self.unit_tangent[np.newaxis, ::-1],
        )
        self.residual_vectors = self.points - np.array([self.intercept, 0]) - self.orthogonal_projections

        residuals = np.linalg.norm(self.residual_vectors, axis=1)

        self.mse = np.sum(np.square(residuals)) / residuals.shape[0]
        self.r2 = 1 - np.sum(np.square(residuals)) / (np.sum(np.square(self.points[:,0] - np.mean(self.points[:,0]))))

    def y_given_x(self, x: float) -> float:
        """
        Get a value of y for some input of x.

        Args:
            x (float): The input for x.

        Returns:
            float: The value of y in this regression given a value of x.
        """
        return self.slope * x + self.intercept

    def x_given_y(self, y: float) -> float:
        """
        Get a value of x for some input of y.

        Args:
            y (float): The input for y.

        Returns:
            float: The value of x in this regression given a value of y.
        """
        return (y - self.intercept) / self.slope

    def set_slope(self, value: float) -> None:
        """
        Set the slope of this orthogonal regression.

        Args:
            value (float): the new slope for the regression.
        """
        if value == 0:
            self._slope = np.finfo(type(value)).tiny
        else:
            self._slope = value

class SonarSegmentType(Enum):
    """Enum for Sonar Segment types."""
    NONE = 0
    WALL = 1
    OBJECT = 2

class SonarSegment:
    """Class to define a sonar segment segment."""
    def __init__(self, points: np.ndarray) -> None:
        """
        Construct a SonarSegment object.

        Args:
            points (ndarray): the points which make up this segment.
        """
        self.number = -1
        self.points = points
        self.ortho_regression: OrthogonalRegression
        self.wall_distance = -1
        self.nearest_object = None
        self.nearest_object_distance = max(points.shape[0], points.shape[1]) * 2
        self.type = SonarSegmentType.NONE

    def get_average_coordinate_of_points(self) -> tuple[int, int]:
        """
        Get the average (row, col) of the points in this SonarSegment.

        Returns:
            tuple(int, int): the coordinates of the average point.
        """
        coordinates = np.zeros(2)
        for point in self.points:
            coordinates = coordinates + point

        coordinates = coordinates / self.points.shape[0]

        return (np.round(coordinates[0]), np.round(coordinates[1]))

class SonarSegmentation:
    """A class which segments an sonar image, and applies regressions to each segment."""
    def __init__(
            self,
            image: np.ndarray,
            wall_object_threshold: float = 0.,
            segment_size_threshold: float = 0.,
            segment_brightness_threshold: float = 0.,
            merge_threshold: float = 1.1,
            merge_angle_limit: float = 5.,
        ) -> None:
        self.segment_size_threshold = segment_size_threshold
        self.segment_brightness_threshold = segment_brightness_threshold
        self.merge_threshold = merge_threshold
        self.merge_angle_limit = merge_angle_limit

        markers = np.zeros_like(image)
        low_threshold = np.percentile(np.percentile(image[image > 0], 30), 30)
        high_threshold = np.percentile(np.percentile(image[image > 0], 50), 50)
        markers[image < low_threshold] = 1
        markers[image > high_threshold] = 2

        segmented = watershed(sobel(image), markers.astype(int))

        self.labeled_points, self.num_segments = label(segmented, background=0, return_num=True, connectivity=2)

        self.filter_segments()
        self.separate_walls_objects(wall_object_threshold)
        self.calculate_nearest_object()

    def filter_segments(self) -> None:
        """
        Regress each segment and filter out invalid segments.

        Through each segment, apply an Orthogonal regression to it. Also, filter out segments which have some kind
        of issue, either being too small or too dim.
        """
        self.side_length = self.labeled_points.shape[0]

        segments = []
        for num in range(self.num_segments + 1):
            if num in {0, 1}:
                segments.append(None)
                continue

            # linear regression on each object
            segment_points = np.argwhere(self.labeled_points == num)
            if segment_points.shape[0] != 0:
                segment = SonarSegment(segment_points)
                segment.number = num
                segment.ortho_regression = OrthogonalRegression(segment.points)
                segments.append(segment)
            else:
                segments.append(None)

        for num, segment in enumerate(segments):
            if segment is None:
                continue
            segments = self.merge_segments(segments, num, segment)
            if segment.points.shape[0] < self.segment_size_threshold:
                segments[num] = None

        self.raw_segments = segments
        self.segments = [segment for segment in segments if segment is not None]

    def merge_segments(self, segments: list, current_segment_num: int, current_segment: SonarSegment) -> list:
        """Combine the SonarSegments in a list of segments if they are not significantly different."""
        changed = False

        for num, segment in enumerate(segments):
            if (num == current_segment_num or segment is None):
                continue
            concatenated = SonarSegment(np.concatenate((current_segment.points, segment.points)))
            concatenated.number = current_segment.number
            concatenated.ortho_regression = OrthogonalRegression(concatenated.points)
            if ((concatenated.ortho_regression.mse < current_segment.ortho_regression.mse * self.merge_threshold
                or concatenated.ortho_regression.mse < segment.ortho_regression.mse * self.merge_threshold)
                and
                (np.abs(np.dot(concatenated.ortho_regression.unit_normal, current_segment.ortho_regression.unit_normal))
                 > np.cos(np.radians(self.merge_angle_limit))
                 or
                 np.abs(np.dot(concatenated.ortho_regression.unit_normal, segment.ortho_regression.unit_normal))
                 > np.cos(np.radians(self.merge_angle_limit)))):

                current_segment = concatenated
                segments[current_segment_num] = concatenated
                segments[num] = None
                changed = True

        if changed:
            segments = self.merge_segments(segments, current_segment_num, current_segment)

        return segments

    def separate_walls_objects(self, threshold: float) -> None:
        """
        Iterate through the segments and select whether they are a Wall or an Objects.

        Args:
            threshold (float): The MSE threshold which will determine whether a segment is a wall or object.
        """
        self.walls = []
        self.objects = []

        for segment in self.segments:
            if segment.ortho_regression.mse <= threshold:
                segment.type = SonarSegmentType.WALL
                self.walls.append(segment)
            else:
                segment.type = SonarSegmentType.OBJECT
                self.objects.append(segment)

    def calculate_nearest_object(self) -> None:
        """For this object's list of segments, find the nearest segment."""
        for segment in self.segments:
            for point in segment.points:
                distance = np.linalg.norm(point)
                if distance < segment.nearest_object_distance:
                    segment.nearest_object = point
                    segment.nearest_object_distance = distance

    def get_nearest_segment(self) -> 'SonarSegment':
        """
        Get the nearest segment.

        Returns:
            SonarSegment: the nearest segment.
        """
        nearest_segment = None
        min_distance = 1000000
        for segment in self.segments:
            if(segment.nearest_object_distance < min_distance):
                nearest_segment = segment
                min_distance = segment.nearest_object_distance

        return nearest_segment # type: ignore

    def plot_segments_together(self, plot, walls, objects) -> None:
        for wall in walls:
            self.plot_segment(plot, wall, SonarSegmentType.WALL)
        for object in objects:
            self.plot_segment(plot, object, SonarSegmentType.OBJECT)

    def plot_segment(self, plot, segment, segment_type=SonarSegmentType.NONE):
        plot.scatter(segment.points[:,1], segment.points[:,0], s=1, label=f"{segment.number}: {segment.ortho_regression.r2}")
        plot.legend()

        if segment_type != SonarSegmentType.WALL:
            plot.scatter(segment.nearest_object[1], segment.nearest_object[0], s=5, c="red")

        if segment_type != SonarSegmentType.OBJECT:
            X_fit = np.linspace(0, self.side_length)
            y_fit = segment.ortho_regression.y_given_x(X_fit)

            # plot line wrt X. only keep parts where y is in frame
            X_fit_1 = X_fit[y_fit<=self.side_length]
            y_fit_1 = y_fit[y_fit<=self.side_length]
            X_fit_1 = X_fit_1[y_fit_1>=0]
            y_fit_1 = y_fit_1[y_fit_1>=0]
            plot.plot(X_fit_1, y_fit_1)

            # plot wrt y in case line is too vertical => no y values is in frame due to sampling gaps
            y_fit = np.linspace(0, self.side_length)
            X_fit = segment.ortho_regression.x_given_y(y_fit)
            y_fit_2 = y_fit[X_fit<=self.side_length]
            X_fit_2 = X_fit[X_fit<=self.side_length]
            y_fit_2 = y_fit_2[X_fit_2>=0]
            X_fit_2 = X_fit_2[X_fit_2>=0]
            plot.plot(X_fit_2, y_fit_2)