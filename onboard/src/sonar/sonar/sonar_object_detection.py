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

    def blur(self, factor: int = 16, cutoff: float = 1 / 5) -> 'SonarDenoiser':
        """
        Apply box blur onto cartesian image.

        Args:
            factor (int): the size of the box for the box blur. Larger values dilute thin,
                sparse reflections (like a wall's 1-2 pixel wide line) more heavily, since
                more of the averaging window ends up covering empty space around them.
            cutoff (float): after blurring and renormalizing to [0, 1], pixels at or below
                this fraction of the new max are zeroed out. Higher values discard more.

        Returns:
            SonarDenoiser: returns itself to allow for method chaining.
        """
        blur_kernel = np.ones((factor, factor), np.float32) / (factor**2)
        self.cartesian = convolve2d(self.cartesian, blur_kernel, mode='same', boundary='symm')

        self.normalize()

        self.cartesian = np.where(self.cartesian > cutoff, self.cartesian, 0)

        return self


class OrthogonalRegression:
    """A class representing the Orthogonal Regression of a group of points."""

    MIN_ROBUST_FIT_POINTS = 5

    def __init__(
        self,
        points: np.ndarray,
        inlier_threshold: float | None = None,
        ransac_iterations: int = 50,
    ) -> None:
        """
        Construct a OrthogonalRegression object given a group of points.

        Args:
            points (ndarray): the points within this orthogonal regression. Points should be (y, x)
            inlier_threshold (float | None): if set, robustly fit the line using RANSAC instead
                of fitting every point directly. A segment's points are rarely a perfectly thin
                line — beam width, multipath, and loose denoising thresholds can all merge a
                real wall's tight core together with a smaller, off-to-one-side contaminating
                population (a side-lobe, a secondary echo, a bit of an adjacent surface) into
                the same DBSCAN cluster. A direct least-squares/PCA fit is skewed by that
                contamination, and — critically — so is naively trimming by residual against
                that same biased fit, since the fit is already pulled toward the contamination
                before residuals are even measured. RANSAC instead searches for the largest
                subset of points consistent with a single line (repeatedly hypothesizing a line
                through 2 random points and counting how many other points fall within
                inlier_threshold pixels of it), then fits only on that subset, which is robust
                even when the contaminating population is a large minority of the segment. If
                None (default) or the segment is too small, every point is fit directly with no
                robustness step. This never affects self.all_points, so callers that need the
                full, untrimmed point cloud (e.g. to measure physical size) still can.
            ransac_iterations (int): number of random 2-point line hypotheses to try.
        """
        self.all_points = points

        fit_points = points
        if inlier_threshold is not None and points.shape[0] > self.MIN_ROBUST_FIT_POINTS:
            inliers = self._find_ransac_inliers(points, inlier_threshold, ransac_iterations)
            if inliers is not None and int(np.count_nonzero(inliers)) >= self.MIN_ROBUST_FIT_POINTS:
                fit_points = points[inliers]

        self._fit(fit_points)
        self.points = fit_points

    @staticmethod
    def _find_ransac_inliers(points: np.ndarray, inlier_threshold: float, iterations: int) -> np.ndarray | None:
        """
        Find the largest subset of points consistent with a single line via random sampling.

        Args:
            points (ndarray): the points to search, as (row, col).
            inlier_threshold (float): the max perpendicular distance, in pixels, from a
                candidate line for a point to count as an inlier to it.
            iterations (int): number of random 2-point line hypotheses to try.

        Returns:
            ndarray | None: a boolean mask over points marking the largest inlier set found,
                or None if no valid line hypothesis could be formed.
        """
        rng = np.random.default_rng()
        num_points = points.shape[0]
        best_inliers = None
        best_count = -1

        for _ in range(max(iterations, 1)):
            i, j = rng.choice(num_points, size=2, replace=False)
            direction = points[j] - points[i]
            norm = np.linalg.norm(direction)
            if norm < 1e-9:
                continue  # degenerate hypothesis (duplicate points); skip

            unit_normal = np.array([-direction[1], direction[0]]) / norm
            distances = np.abs((points - points[i]) @ unit_normal)
            inliers = distances <= inlier_threshold

            count = int(np.count_nonzero(inliers))
            if count > best_count:
                best_count = count
                best_inliers = inliers

        return best_inliers

    def _fit(self, points: np.ndarray) -> None:
        """
        Fit this orthogonal regression to the given points, overwriting all fit attributes.

        Args:
            points (ndarray): the points to fit. Points should be (y, x).
        """
        e_val, e_vect = np.linalg.eig(np.cov(points, rowvar=False))
        e_val = e_val.real
        e_vect = e_vect.real

        # The eigenvector of the largest eigenvalue points along the direction of maximum
        # variance, i.e. the tangent of the best-fit line. The eigenvector of the smallest
        # eigenvalue is the direction of minimum variance, i.e. the normal to the line.
        # points are (row, col); an eigenvector's sign is arbitrary (v and -v are equally
        # valid tangent directions for an undirected line), but flipping only ONE of its two
        # components (as opposed to negating the whole vector) reflects it across an axis,
        # producing a direction that is neither the true tangent nor its negation. That used
        # to happen here and corrupted both unit_normal (and therefore normal_angle, used to
        # rotate the sub) and slope (used for angle_of_wall) — do not reintroduce it.
        self.unit_tangent = e_vect[:, np.argmax(e_val)]
        self.unit_normal = np.array([-self.unit_tangent[1], self.unit_tangent[0]])

        # Ratio of variance along the line to variance across it. Close to 1 for a round/blob
        # shaped cluster of points, much greater than 1 for a long, thin, wall-like cluster.
        min_eig = max(float(np.min(e_val)), 1e-9)
        self.elongation = float(np.max(e_val)) / min_eig

        # slope/intercept fit row as a function of col (see self.intercept below), so slope is
        # d(row)/d(col) = tangent's row-component / tangent's col-component.
        if self.unit_tangent[1] == 0:
            self.slope = (2**31) - 1
        else:
            self.slope = self.unit_tangent[0] / self.unit_tangent[1]

        self.intercept = points[:, 0].mean() - self.slope * points[:, 1].mean()

        self.orthogonal_projections = np.matmul(
            np.dot(points - np.array([self.intercept, 0]), self.unit_tangent[::-1])[:, np.newaxis],
            self.unit_tangent[np.newaxis, ::-1],
        )
        self.residual_vectors = points - np.array([self.intercept, 0]) - self.orthogonal_projections

        residuals = np.linalg.norm(self.residual_vectors, axis=1)

        self.mse = np.sum(np.square(residuals)) / residuals.shape[0]
        self.r2 = 1 - np.sum(np.square(residuals)) / (np.sum(np.square(points[:, 0] - np.mean(points[:, 0]))))

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

    def get_span(self) -> float:
        """
        Get how far this segment's points spread out along its own fitted line.

        This is a proxy for the reflector's physical size along its length, which helps tell
        apart a true wall (which spans a large distance) from a small but coincidentally
        elongated reflector, e.g. a diver, pipe, or another robot's frame/tether, which can
        have a high elongation ratio despite being physically small.

        Returns:
            float: the distance, in pixels, between the two most extreme points of this
                segment when projected onto its fitted tangent direction.
        """
        projections = self.points @ self.ortho_regression.unit_tangent
        return float(np.max(projections) - np.min(projections))


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
        wall_fit_inlier_threshold: float | None = None,
        wall_fit_ransac_iterations: int = 50,
    ) -> None:
        """
        Construct a ClusteredSonarSegmentation object.

        Args:
            image (ndarray): the denoised cartesian sonar image to segment.
            eps (float): DBSCAN neighborhood radius, in pixels, for two points to be
                considered connected.
            min_samples (int): DBSCAN minimum number of neighbors, within eps, for a point
                to be treated as a core (non-noise) point.
            wall_fit_inlier_threshold (float | None): if set, each segment's line/angle is
                fit robustly via RANSAC using this inlier distance in pixels, instead of
                fitting every point in the segment directly (see
                OrthogonalRegression.__init__). This does not affect which points make up a
                segment (e.g. for span/elongation purposes) — only which of those points the
                final fitted angle is based on.
            wall_fit_ransac_iterations (int): number of RANSAC iterations per segment when
                wall_fit_inlier_threshold is set.
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
                segment.ortho_regression = OrthogonalRegression(
                    cluster_points, wall_fit_inlier_threshold, wall_fit_ransac_iterations,
                )
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

    def get_most_wall_like_segment(
        self,
        min_elongation: float,
        min_span: float = 0.0,
        min_range_pixels: float | None = None,
        max_range_pixels: float | None = None,
    ) -> 'SonarSegment | None':
        """
        Get the largest segment that is shaped like a flat wall rather than a compact object.

        A segment is considered wall-like if its points are much more spread out along its
        fitted line than across it (see OrthogonalRegression.elongation) AND it spans at
        least min_span pixels along that line (see SonarSegment.get_span).

        Both checks have real-world limitations: elongation alone can be fooled by a small,
        physically compact reflector (a diver, pipe, or another robot) that happens to have a
        high elongation ratio from just a handful of points, and it can also work against a
        real wall, since actual pool walls are rarely perfectly straight/flat in the denoised
        image (mounting seams, curvature in the cartesian remap, etc. add spread across the
        fitted line and lower its elongation relative to a small but very straight object).
        min_elongation is therefore intentionally lenient, mainly to rule out round/blob-shaped
        clusters (elongation close to 1), and min_span is what should really be relied on to
        rule out small objects.

        Optional min/max range (average distance to the sonar origin, in pixels) further
        rejects wall-like clutter outside the expected standoff band.

        Among segments that pass the checks, the LARGEST one (by span) is returned rather
        than the nearest one: a true wall is almost always the biggest reflective structure in
        the scene by a wide margin, so preferring size over proximity is more robust against
        latching onto a smaller-but-closer object than preferring proximity is.

        Args:
            min_elongation (float): the minimum elongation ratio for a segment to be
                considered wall-like.
            min_span (float): the minimum distance, in pixels, a segment must span along its
                fitted line to be considered wall-like. Defaults to 0.0 (no minimum).
            min_range_pixels (float | None): if set, require average range >= this (pixels).
            max_range_pixels (float | None): if set, require average range <= this (pixels).

        Returns:
            SonarSegment | None: the largest wall-like segment, or None if no segment qualifies.
        """
        wall_like_segments = []
        for segment in self.segments:
            if segment.ortho_regression.elongation < min_elongation or segment.get_span() < min_span:
                continue
            if min_range_pixels is not None or max_range_pixels is not None:
                range_px = segment.get_average_distance_to_origin()
                if min_range_pixels is not None and range_px < min_range_pixels:
                    continue
                if max_range_pixels is not None and range_px > max_range_pixels:
                    continue
            wall_like_segments.append(segment)

        if not wall_like_segments:
            return None

        return max(wall_like_segments, key=lambda segment: segment.get_span())
