import cv2
import numpy as np
import rclpy

import cv.config as cv_constants
from cv import hsv_filter
from scipy.spatial.distance import cdist

class HSVTorpedoTargets(hsv_filter.HSVFilter):
    def __init__(self) -> None:
        super().__init__(
            name='bin_torpedo_targets',
            camera='front',
            mask_ranges=[
                [cv_constants.Torpedo.LOW_BOT, cv_constants.Torpedo.LOW_TOP],
                [cv_constants.Torpedo.HIGH_BOT, cv_constants.Torpedo.HIGH_TOP]
            ],
            width=cv_constants.Torpedo.WIDTH,
        )

def main(args: list[str] | None = None) -> None:
    """Run the node."""
    rclpy.init(args=args)
    hsv_filter = HSVTorpedoTargets()

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