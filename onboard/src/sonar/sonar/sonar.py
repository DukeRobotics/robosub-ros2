import io
import os
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
import rclpy
import resource_retriever as rr
import tf2_ros
import yaml
from brping import Ping360
from custom_msgs.srv import SonarSweepRequest
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from serial.tools import list_ports
from std_msgs.msg import String

from sonar import sonar_image_processing, sonar_object_detection, sonar_utils


class Sonar(Node):
    """Class to interface with the Sonar device."""
    CONFIG_FILE_PATH = f'package://sonar/config/{os.getenv("ROBOT_NAME")}.yaml'

    BAUD_RATE = 2000000  # hz
    SAMPLE_PERIOD_TICK_DURATION = 25e-9  # s
    SPEED_OF_SOUND_IN_WATER = 1482  # m/s

    FILTER_INDEX = 100  # First x values are filtered out
    DEFAULT_RANGE = 5  # m
    prev_range = DEFAULT_RANGE
    DEFAULT_NUMER_OF_SAMPLES = 1200  # 1200 is max resolution

    _serial_port = None
    CONNECTION_RETRY_PERIOD = 5  # s
    LOOP_RATE = 10  # Hz
    STATUS_LOOP_RATE = 5 # Hz

    SONAR_STATUS_TOPIC = 'sonar/status'
    SONAR_REQUEST_TOPIC = 'sonar/request'
    SONAR_IMAGE_TOPIC = 'sonar/image/compressed'
    SONAR_DENOISED_IMAGE_TOPIC = 'sonar/image/denoised'

    NODE_NAME = 'sonar'

    CONSTANT_SWEEP_START = 100
    CONSTANT_SWEEP_END = 300

    VALUE_THRESHOLD = 95  # Sonar intensity threshold
    DBSCAN_EPS = 3  # DBSCAN epsilon
    DBSCAN_MIN_SAMPLES = 10  # DBSCAN min samples

    NUM_RETRIES = 10

    NEGATE_POSE = False

    def __init__(self) -> None:
        super().__init__(self.NODE_NAME)
        self.get_logger().info('Sonar planning node initialized')

        self.stream = self.declare_parameter('stream', True).value
        self.debug = self.declare_parameter('debug', False).value

        with Path(rr.get_filename(self.CONFIG_FILE_PATH, use_protocol=False)).open() as f:
            self._config_data = yaml.safe_load(f)

        self.ftdi = self._config_data['ftdi']
        self.center_gradians = self._config_data['center_gradians']
        self.increase_ccw = self._config_data['increase_ccw']

        self.status_publisher = self.create_publisher(String, self.SONAR_STATUS_TOPIC, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Enamble streaming
        self.cv_bridge = CvBridge()
        if self.stream:
            self.sonar_image_publisher = self.create_publisher(CompressedImage, self.SONAR_IMAGE_TOPIC, 10)
            self.denoised_image_publisher = self.create_publisher(CompressedImage, self.SONAR_DENOISED_IMAGE_TOPIC, 10)

        self.current_scan = (-1, -1, -1)  # (start_angle, end_angle, distance_of_scan)

        # Create ping360 instance
        self.ping360 = Ping360()

        # Find sonar port
        try:
            self._serial_port = next(list_ports.grep(self.ftdi)).device
        except StopIteration:
            self.get_logger().error('Sonar not found.')
            rclpy.shutdown()

        self.connect_timer = self.create_timer(1.0 / self.CONNECTION_RETRY_PERIOD, self.connect)
        self.status_publisher_timer = self.create_timer(1.0 / self.STATUS_LOOP_RATE, self.publish_status)

    def connect(self) -> None:
        """Read FTDI strings of all ports in list_ports.grep."""
        try:
            self.ping360.connect_serial(str(self._serial_port), self.BAUD_RATE)
            self.ping360.initialize()
            self.connect_timer.cancel()
            self.get_logger().info('Connected to sonar.')
            self.init_sonar()
            self.run()
        except StopIteration:
            self.get_logger().error(f'Error in connecting to sonar, trying again in '
                                    f'{self.CONNECTION_RETRY_PERIOD} seconds.')

    def init_sonar(self) -> None:
        """Set up default parameters for the sonar device."""
        self.ping360.set_number_of_samples(self.DEFAULT_NUMER_OF_SAMPLES)

        self.sample_period = sonar_utils.range_to_period(self.DEFAULT_RANGE, self.DEFAULT_NUMER_OF_SAMPLES)
        self.ping360.set_sample_period(self.sample_period)

        self.transmit_duration = sonar_utils.range_to_transmit(self.DEFAULT_RANGE, self.DEFAULT_NUMER_OF_SAMPLES)
        self.ping360.set_transmit_duration(self.transmit_duration)

    def publish_status(self) -> None:
        """Publish the status of the sonar device."""
        # make string type std_msgs String
        self.status_publisher.publish(String(data='Sonar Running'))

    def set_new_range(self, sonar_range: int) -> None:
        """
        Set a new sample_period and transmit_duration.

        Args:
            sonar_range (int): max range in meters of the sonar scan.
        """
        self.prev_range = sonar_range
        self.sample_period = sonar_utils.range_to_period(self.DEFAULT_RANGE, self.DEFAULT_NUMER_OF_SAMPLES)
        self.ping360.set_sample_period(self.sample_period)

        self.transmit_duration = sonar_utils.range_to_transmit(sonar_range, self.DEFAULT_NUMER_OF_SAMPLES)
        self.ping360.set_transmit_duration(self.transmit_duration)

    def request_data_at_angle(self, angle_in_gradians: float) -> list:
        """
        Set sonar device to provided angle and retrieve data.

        Args:
            angle_in_gradians (float): Angle to set the sonar device to.

        Returns:
            dictionary: Response from the device. See
            https://docs.bluerobotics.com/ping-protocol/pingmessage-ping360/#get
            for information about the dictionary response.

            Basically, an array of number_of_samples size with each index having
            a distance of MAX_RANGE/number_of_samples * [index] meters from the
            sonar device. The value is the intensity of the ping at that point.
        """
        for _ in range(self.NUM_RETRIES):
            response = self.ping360.transmitAngle(angle_in_gradians)
            if response is not None:
                break
            self.get_logger().error(f'Error in getting data at angle {angle_in_gradians}')

        response_to_int_array = [int(item) for item in response.data]
        return [0] * self.FILTER_INDEX + response_to_int_array[self.FILTER_INDEX:]

    def get_sweep(self, range_start: int = 100, range_end: int = 300) -> np.ndarray:
        """
        Get data along a range of angles.

        Args:
            range_start (int, optional): Angle to start sweep in gradians.
                Defaults to 100. Min is 0.
            range_end (int, optional): Angle to end sweep in gradians.
                Defaults to 300. Max is 399.

        Returns:
            List: List of data messages from the Sonar device.
        """
        sonar_sweep_data = []
        # Sweep from end to start so that the top edge of ths sweep corresponds to the left side of the robot
        # and the right edge of the sweep corresponds to the right side of the robot
        for i in range(range_end, range_start + 1, -1 if self.increase_ccw else 1):
            sonar_scan = self.request_data_at_angle(i)
            sonar_sweep_data.append(sonar_scan)

        return np.vstack(sonar_sweep_data)

    def get_xy_of_object_in_sweep(self, start_angle: int, end_angle: int) -> \
            tuple[Pose | None, np.ndarray, float | None]:
        """
        Get the depth of the sweep of a detected object. For now uses mean value.

        Args:
            start_angle (int): Angle to start sweep in gradians.
            end_angle (int): Angle to end sweep in gradians.

        Returns:
            (Pose, List, float): Pose of the object in robot reference frame, sonar sweep array, and normal angle.
        """
        sweep = self.get_sweep(start_angle, end_angle)

        denoiser = sonar_object_detection.SonarDenoiser(sweep)
        denoiser.wall_block().percentile_filter().fourier_signal_processing().init_cartesian().normalize().blur()
        self.denoised_image_publisher.publish(
            sonar_utils.convert_to_ros_compressed_img(denoiser.cartesian, self.cv_bridge),
            )
        color_image = sonar_image_processing.build_color_sonar_image_from_int_array(denoiser.cartesian)

        segmentation = sonar_object_detection.SonarSegmentation(
            denoiser.cartesian,
            wall_object_threshold=0,
            segment_size_threshold=2000/90*denoiser.shape_theta,
            segment_brightness_threshold=60,
            merge_threshold=1.5,
            merge_angle_limit=6,
        )

        nearest_segment = segmentation.get_nearest_segment()

        if nearest_segment is None:
            return (None, color_image, None)

        _, sonar_index = nearest_segment.get_average_coordinate_of_points()
        normal_angle = np.arctan2(
            nearest_segment.ortho_regression.unit_normal[1],
            nearest_segment.ortho_regression.unit_normal[0]
            )

        sonar_angle = (start_angle + end_angle) / 2  # Take the middle of the sweep

        return (sonar_utils.to_robot_position(sonar_angle, sonar_index, self.sample_period,
                                              self.center_gradians, self.NEGATE_POSE),
                                              color_image,
                                              normal_angle)

    def constant_sweep(self) -> None:
        """
        In debug mode, scan indefinitely and publish images.

        Args:
            start_angle (int): Angle to start sweep in gradians.
            end_angle (int): Angle to end sweep in gradians.
            distance_of_scan (int): Distance in meters to scan.
        """
        self.get_logger().info('Starting constant sweep')
        self.set_new_range(self.DEFAULT_RANGE)

        # Perform a scan
        try:
            self.get_logger().info(f'Starting sweep from {self.CONSTANT_SWEEP_START} to {self.CONSTANT_SWEEP_END}')
            sonar_sweep = self.get_sweep(self.CONSTANT_SWEEP_START, self.CONSTANT_SWEEP_END)
            self.get_logger().info('Finishng sweep')
            if self.stream:
                compressed_image = sonar_utils.convert_to_ros_compressed_img(sonar_sweep, self.cv_bridge)
                self.sonar_image_publisher.publish(compressed_image)
        except (RuntimeError, ValueError) as e:
            self.get_logger().error(f'Error during constant sweep: {e}')
            rclpy.shutdown()

    def perform_sonar_request(self, request: SonarSweepRequest.Request, response: SonarSweepRequest.Response) -> \
            SonarSweepRequest.Response:
        """
        Perform a sonar request.

        Takes in a SonarSweepRequest service and performs a scan between two angles.
        It returns the position of the largest object detected in the scan. Since the
        scan should be a small angle, the returned pose should be the physical position
        of the object in the world.

        Args:
            request (SonarSweepRequest.Request): Request containing the start angle,
                end angle, and distance of scan.
            response (SonarSweepRequest.Response): Response containing the pose of
                the object, the normal angle, and if an object was detected.

        Returns:
            SonarSweepRequest.Response: Response generated from the sonar request.
        """
        response.pose = PoseStamped()
        response.normal_angle = 0.0
        response.is_object = False

        # Get request details
        start_degrees = request.start_angle
        end_degrees = request.end_angle
        new_range = request.distance_of_scan

        if start_degrees > end_degrees:
            response.success = False
            response.message = f'Start angle {start_degrees} must be less than or equal to end angle {end_degrees}.'
            return response

        left_gradians = sonar_utils.degrees_to_centered_gradians(start_degrees, self.center_gradians, self.increase_ccw)
        right_gradians = sonar_utils.degrees_to_centered_gradians(end_degrees, self.center_gradians,
                                                                  self.increase_ccw)

        self.get_logger().info(f'Recieved Sonar request: {left_gradians}, {right_gradians}, {new_range}')

        # Angle must be between 0 and 400 and range must be positive
        if left_gradians < 0 or right_gradians < 0 or right_gradians > 400 or new_range < 0: # noqa: PLR2004
            self.get_logger().error('Bad sonar request')
            return response

        if new_range != self.prev_range:
            self.set_new_range(new_range)

        try:
            object_pose, plot, normal_angle = self.get_xy_of_object_in_sweep(
                left_gradians,
                right_gradians,
                )
            self.get_logger().debug('Finished xy_of_object')
        except RuntimeError as e:
            response.success = False
            response.message = str(e)
            return response

        if object_pose is not None:
            response.pose = object_pose
            response.normal_angle = normal_angle
            response.is_object = True

        if object_pose is None:
            self.get_logger().error('No object found')
            response.message = 'No object found.'
        else:
            response.message = 'Found object.'

        if self.stream:
            sonar_image = sonar_utils.convert_to_ros_compressed_img(plot, self.cv_bridge, is_color=True)
            self.sonar_image_publisher.publish(sonar_image)

        return response

    def run(self) -> None:
        """Run the main loop of the node."""
        if self.debug:
            self.create_timer(1.0 / self.LOOP_RATE, self.constant_sweep)
        else:
            self.create_service(SonarSweepRequest, 'sonar/request', self.perform_sonar_request)

def main(args: list[str] | None = None) -> None:
    """Initialize and run the Sonar node."""
    rclpy.init(args=args)
    sonar = Sonar()

    try:
        rclpy.spin(sonar)
    except KeyboardInterrupt:
        pass
    finally:
        sonar.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
