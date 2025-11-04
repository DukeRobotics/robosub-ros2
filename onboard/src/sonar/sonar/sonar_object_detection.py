import numpy as np
from scipy.signal import convolve2d


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
