from __future__ import annotations

from dataclasses import dataclass
from typing import TYPE_CHECKING

import numpy as np
from scipy.ndimage import gaussian_filter, gaussian_filter1d, label, map_coordinates, median_filter

if TYPE_CHECKING:
    from numpy.typing import ArrayLike, NDArray

R0: int = 100  # first valid range column (blind zone before this)

@dataclass(frozen=True, slots=True)
class Geo:
    """
    Sonar geometry needed to turn (row, column) indices into meters.

    Attributes:
        rb: Meters per range bin.
        start: Gradian of row 0.
        step: Gradians per row, so any row count works.
        cw: True if increasing gradians sweep clockwise.
        off: Range of column 0 in meters.
    """

    rb: float = 0.01
    start: float = 0.0
    step: float = 100 / 98
    cw: bool = True
    off: float = 0.0


@dataclass(slots=True)
class SonarObject:
    """One detected object, as returned by `objects`."""

    id: int
    conf: float
    xy: NDArray[np.float64]  # (N, 2) pixel positions in meters
    centroid: tuple[float, float]  # (x, y) in meters
    n_px: int
    peak: float  # maximum z-score inside the object
    mean: float  # mean z-score inside the object
    rows: int  # bearing extent in rows
    cols: int  # range extent in columns

    def __post_init__(self) -> None:
        """Check values after SonarObject is initialized."""
        if self.xy.ndim != 2 or self.xy.shape[1] != 2:  # noqa: PLR2004
            msg = f'xy must have shape (N, 2), got {self.xy.shape}'
            raise ValueError(msg)


def preprocess(raw: NDArray, vmed: int = 5, rs: float = 3, bs: float = 1.5, wide: int = 60) -> NDArray[np.float32]:
    """
    Despeckle a sonar sweep and convert it to robust z-scores.

    Args:
        raw: Polar sweep with shape (n_bearing, n_range).
        vmed: Median window along bearing in rows. Streaks thinner than this are removed.
        rs: Gaussian sigma along range in columns.
        bs: Gaussian sigma along bearing in rows.
        wide: Gaussian sigma in columns of the slow range background that is subtracted.

    Returns:
        np.ndarray: Z-score image with the same shape as `raw`. The blind zone (columns before `R0`) is 0.
    """
    x = raw.astype(np.float32)[:, R0:]
    x = median_filter(x, (vmed, 1), mode='nearest')  # streaks are thin in bearing and long in range: gone
    x = gaussian_filter(x, (bs, rs), mode='nearest')  # light speckle smoothing
    d = x - gaussian_filter1d(x, wide, axis=1, mode='nearest')  # drop slow range falloff and reverberation
    z = d / (1.4826 * np.median(np.abs(d - np.median(d))) + 1e-6)  # robust noise units, targets are a tiny fraction
    out = np.zeros(raw.shape, np.float32)
    out[:, R0:] = z
    return out


def detect(z: NDArray, lo: float = 3.0, hi: float = 9.0, min_area: int = 300) -> NDArray[np.int_]:
    """
    Find objects with a hysteresis threshold and a size filter.

    Args:
        z: Z-score image from `preprocess`, in noise units.
        lo: Low threshold. Regions grow out to every connected pixel above this.
        hi: High threshold. A region must contain at least one pixel above this.
        min_area: Minimum region size in pixels.

    Returns:
        Label image with the same shape as `Z`. 0 is background, objects are numbered 1..k.
    """
    S = z[:, R0:]
    lab, n = label(lo < S)
    ok = (np.bincount(lab[hi < S], minlength=n + 1) > 0) & (np.bincount(lab.ravel(), minlength=n + 1) >= min_area)
    ok[0] = False
    out = np.zeros(z.shape, int)
    out[:, R0:] = (np.cumsum(ok) * ok)[lab]
    return out


def to_xy(b: ArrayLike, r: ArrayLike, g: Geo) -> NDArray[np.float64]:
    """
    Convert row and column indices to meters. 0 gradians is +y and x points to the right.

    Args:
        b: Row (bearing) indices. Fractional values are fine.
        r: Column (range) indices. Fractional values are fine.
        g: Sonar geometry.

    Returns:
        Array of shape (N, 2) with columns (x, y) in meters.
    """
    a = (g.start + np.asarray(b, float) * g.step) * np.pi / 200
    r = np.asarray(r, float) * g.rb + g.off
    return np.c_[(1 if g.cw else -1) * r * np.sin(a), r * np.cos(a)]


def confidence(
    peak: float,
    n_px: int,
    n_rows: int,
    noise_peak: float = 9.0,
    width: float = 1.5,
    area_per_row: float = 6,
) -> float:
    """
    Score how likely an object is real, from 0 to 1. This is a heuristic, not a calibrated probability.

    Args:
        peak: Maximum z-score inside the object.
        n_px: Object size in pixels.
        n_rows: Number of bearing rows in the sweep.
        noise_peak: Peak z-score where the strength term reaches 0.5.
        width: Steepness of the strength logistic, in z-score units.
        area_per_row: Pixels per bearing row at which the size term reaches about 0.63.

    Returns:
        Confidence between 0 and 1.
    """
    return float(1 / (1 + np.exp(-(peak - noise_peak) / width)) * (1 - np.exp(-n_px / (area_per_row * n_rows))))


def objects(lab: NDArray[np.int_], z: NDArray, g: Geo) -> list[SonarObject]:
    """
    Summarize each labeled object.

    Args:
        lab: Label image from `detect`.
        z: Z-score image from `preprocess`.
        g: Sonar geometry.

    Returns:
        One `SonarObject` per label, in label order. Centroids are averaged in Cartesian
        coordinates to handle objects spanning the angular seam.
    """
    out: list[SonarObject] = []
    for i in range(1, lab.max() + 1):
        b, r = np.nonzero(lab == i)
        pk = float(z[b, r].max())
        out.append(
            SonarObject(
                id=i,
                conf=confidence(pk, len(b), len(lab)),
                xy=to_xy(b, r, g),
                centroid=tuple(to_xy(b, r, g).mean(axis=0)),
                n_px=len(b),
                peak=pk,
                mean=float(z[b, r].mean()),
                rows=int(np.ptp(b)) + 1,
                cols=int(np.ptp(r)) + 1,
            ),
        )
    return out


def run(raw: NDArray, g: Geo) -> tuple[NDArray[np.float32], NDArray[np.int_], list[SonarObject]]:
    """
    Run the full pipeline on one sweep with default settings.

    Args:
        raw: Polar sweep with shape (n_bearing, n_range).
        g: Sonar geometry.

    Returns:
        tuple: Tuple of (z-score image, label image, list of detected objects).
    """
    Z = preprocess(raw)
    lab = detect(Z)
    return Z, lab, objects(lab, Z, g)


def to_cart(
    img: NDArray, g: Geo, res: float | None = None, order: int = 1,
) -> tuple[NDArray[np.float32], tuple[float, float, float, float]]:
    """
    Resample a polar image onto a Cartesian grid by inverse mapping, so there are no holes.

    Args:
        img: Polar image with shape (n_bearing, n_range).
        g: Sonar geometry.
        res: Output pixel size in meters. Defaults to the range bin size.
        order: Interpolation order; use 0 for discrete labels and 1 for intensity images.

    Returns:
        Tuple of (Cartesian image, extent). The image is NaN outside the scanned area and inside the
        blind zone. The extent is (xmin, xmax, ymin, ymax) in meters, for `plt.imshow(..., extent=...)`.
    """
    n, m = img.shape
    full = n * g.step >= 400 - 1e-6
    res = res or g.rb
    rmax = m * g.rb + g.off
    ax = np.arange(-rmax, rmax + res, res)
    X, Y = np.meshgrid(ax, ax)
    rel = (np.arctan2((1 if g.cw else -1) * X, Y) * 200 / np.pi - g.start) % 400
    rows, cols = rel / g.step, (np.hypot(X, Y) - g.off) / g.rb
    src = np.vstack([img, img[:1]]) if full else img  # wrap the seam on full sweeps
    ok = (cols >= R0) & (cols <= m - 1) & (rows <= (n if full else n - 1))
    c = map_coordinates(src.astype(np.float32), [rows, cols], order=order, mode='nearest')
    return np.where(ok, c, np.nan), (ax[0], ax[-1], ax[0], ax[-1])
