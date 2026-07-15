from enum import Enum

import numpy as np
from scipy.signal import convolve2d
from sklearn.cluster import DBSCAN

NUM_DIMENSIONS_FOR_REPEAT = 3


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

        # Reshape data
        processed_data = np.zeros(
            shape=(100, np.floor(self.shape_radius * 1.41421356).astype(int)),
        )
        processed_data[: self.shape_theta, : self.shape_radius] = self.data[: self.shape_theta]

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
                if self.data[theta][r] > max_along_theta * threshold:
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
        nonzero_data = self.data[self.data > 0]
        if nonzero_data.size == 0:
            return self

        threshold = float(np.percentile(np.percentile(nonzero_data, threshold), threshold))
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

        # Applies the Radial Mask
        radius = np.sqrt(xv**2 + yv**2)
        mask = (radius < outer_radius) & (radius >= inner_radius)
        mask = mask.astype(np.float32)
        if self.data.ndim == NUM_DIMENSIONS_FOR_REPEAT and self.data.shape[2] == NUM_DIMENSIONS_FOR_REPEAT:
            mask = np.repeat(mask[:, :, np.newaxis], 3, axis=2)
        fimg = np.fft.fftshift(np.fft.fft2(self.data, axes=(0, 1))) * mask

        # Filter
        self.data = np.fft.ifft2(np.fft.ifftshift(fimg))
        self.data = np.abs(self.data)
        self.data[self.data < threshold] = 0

        # Return self
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
        theta[:, 0] = 89  # x=0
        theta[:, 1:] = np.arctan(y[:, 1:] / x[:, 1:]) / np.pi * 180
        theta_gradians = (theta / 90 * 100).astype(int)
        r = (np.floor(np.sqrt(x**2 + y**2))).astype(int)

        self.cartesian = self.data[theta_gradians, r]
        return self

    def normalize(self) -> 'SonarDenoiser':
        """
        Normalize the cartesian image.

        Returns:
            SonarDenoiser: returns itself to allow for method chaining.
        """
        self.cartesian = self.cartesian - np.min(self.cartesian)
        max_value = np.max(self.cartesian)
        if np.isclose(max_value, 0):
            return self
        self.cartesian = self.cartesian / max_value

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
        self.cartesian = convolve2d(self.cartesian, blur_kernel, mode='same', boundary='symm')

        self.normalize()

        self.cartesian = np.where(self.cartesian > 1 / 5, self.cartesian, 0)

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
        e_val = e_val.real
        e_vect = e_vect.real

        # The eigenvector of the largest eigenvalue points along the direction of maximum
        # variance, i.e. the tangent of the best-fit line. The eigenvector of the smallest
        # eigenvalue is the direction of minimum variance, i.e. the normal to the line.
        self.unit_tangent = e_vect[:, np.argmax(e_val)]
        self.unit_tangent[1] *= -1
        self.unit_normal = np.array([-self.unit_tangent[1], self.unit_tangent[0]])

        # Ratio of variance along the line to variance across it. Close to 1 for a round/blob
        # shaped cluster of points, much greater than 1 for a long, thin, wall-like cluster.
        min_eig = max(float(np.min(e_val)), 1e-9)
        self.elongation = float(np.max(e_val)) / min_eig

        if self.unit_tangent[0] == 0:
            self.slope = (2**31) - 1
        else:
            self.slope = self.unit_tangent[1] / self.unit_tangent[0]

        self.intercept = self.points[:, 0].mean() - self.slope * self.points[:, 1].mean()

        self.orthogonal_projections = np.matmul(
            np.dot(self.points - np.array([self.intercept, 0]), self.unit_tangent[::-1])[:, np.newaxis],
            self.unit_tangent[np.newaxis, ::-1],
        )
        self.residual_vectors = self.points - np.array([self.intercept, 0]) - self.orthogonal_projections

        residuals = np.linalg.norm(self.residual_vectors, axis=1)

        self.mse = np.sum(np.square(residuals)) / residuals.shape[0]
        self.r2 = 1 - np.sum(np.square(residuals)) / (np.sum(np.square(self.points[:, 0] - np.mean(self.points[:, 0]))))

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

        return (int(np.round(coordinates[0])), int(np.round(coordinates[1])))

    def get_average_distance_to_origin(self) -> float:
        """
        Get the average Euclidean distance of this segment's points to the sonar origin.

        The sonar origin (0, 0) corresponds to the robot's own position in the denoised
        cartesian image, so this is a proxy for how close this segment is to the robot.

        Returns:
            float: the average distance, in pixels, of this segment's points to the origin.
        """
        return float(np.mean(np.linalg.norm(self.points, axis=1)))


class GlobalSonarSegmentation:
    """A class which treats all non-zero sonar data as a single segment."""

    def __init__(
        self,
        image: np.ndarray,
    ) -> None:

        # Store image
        self.image = image
        self.side_length = image.shape[0]

        # Get ALL non-zero pixels
        points = np.argwhere(image > 0)

        if points.shape[0] == 0:
            self.raw_segments = []
            self.segments = []
            self.walls = []
            self.objects = []
            return

        # Create single segment
        segment = SonarSegment(points)
        segment.number = 1
        segment.ortho_regression = OrthogonalRegression(segment.points)

        # Everything is now one segment
        self.raw_segments = [segment]
        self.segments = [segment]

    def get_nearest_segment(self) -> 'SonarSegment':
        """
        Get the nearest segment.

        Returns:
            SonarSegment: the nearest segment.
        """
        return self.segments[0]


class ClusteredSonarSegmentation:
    """
    Segments sonar data into distinct objects using density-based clustering.

    Unlike GlobalSonarSegmentation, this treats spatially separate reflectors (e.g. a wall,
    a diver, a pipe, another robot) as distinct segments rather than lumping every non-zero
    pixel into a single point cloud and fitting one line through all of them.
    """

    MIN_POINTS_FOR_REGRESSION = 2

    def __init__(
        self,
        image: np.ndarray,
        eps: float,
        min_samples: int,
    ) -> None:
        """
        Construct a ClusteredSonarSegmentation object.

        Args:
            image (ndarray): the denoised cartesian sonar image to segment.
            eps (float): DBSCAN neighborhood radius, in pixels, for two points to be
                considered connected.
            min_samples (int): DBSCAN minimum number of neighbors, within eps, for a point
                to be treated as a core (non-noise) point.
        """
        self.image = image
        self.side_length = image.shape[0]

        self.raw_segments: list[SonarSegment] = []
        self.segments: list[SonarSegment] = []

        points = np.argwhere(image > 0)
        if points.shape[0] == 0:
            return

        labels = DBSCAN(eps=eps, min_samples=min_samples).fit_predict(points)

        for label in sorted(set(labels)):
            if label == -1:
                continue  # DBSCAN noise label; not dense enough to be a real object

            cluster_points = points[labels == label]
            if cluster_points.shape[0] < self.MIN_POINTS_FOR_REGRESSION:
                continue

            segment = SonarSegment(cluster_points)
            segment.number = int(label)
            try:
                segment.ortho_regression = OrthogonalRegression(cluster_points)
            except np.linalg.LinAlgError:
                # Degenerate cluster (e.g. singular covariance matrix); skip it
                continue

            self.raw_segments.append(segment)
            self.segments.append(segment)

    def get_nearest_segment(self) -> 'SonarSegment | None':
        """
        Get the segment which is, on average, closest to the sonar origin (the robot).

        Returns:
            SonarSegment | None: the nearest segment, or None if no segments were found.
        """
        if not self.segments:
            return None

        return min(self.segments, key=lambda segment: segment.get_average_distance_to_origin())

    def get_most_wall_like_segment(self, min_elongation: float) -> 'SonarSegment | None':
        """
        Get the nearest segment that is shaped like a flat wall rather than a compact object.

        A segment is considered wall-like if its points are much more spread out along its
        fitted line than across it (see OrthogonalRegression.elongation), which distinguishes
        long flat surfaces (walls) from compact/round reflectors (divers, pipes, other robots).
        Among wall-like segments, the one nearest to the sonar origin is returned, since that
        is the most likely reflector to be usable for wall-relative navigation.

        Args:
            min_elongation (float): the minimum elongation ratio for a segment to be
                considered wall-like.

        Returns:
            SonarSegment | None: the nearest wall-like segment, or None if no segment qualifies.
        """
        wall_like_segments = [
            segment for segment in self.segments if segment.ortho_regression.elongation >= min_elongation
        ]
        if not wall_like_segments:
            return None

        return min(wall_like_segments, key=lambda segment: segment.get_average_distance_to_origin())
