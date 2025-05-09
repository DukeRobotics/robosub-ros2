import os
from pathlib import Path

import numpy as np
import rclpy
import resource_retriever as rr
import tf2_ros
import yaml
from brping import Ping360
from custom_msgs.srv import SonarSweepRequest
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from serial.tools import list_ports
from std_msgs.msg import String

from sonar import sonar_image_processing, sonar_utils


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
    STATUS_LOOP_RATE = 1 # Hz

    SONAR_STATUS_TOPIC = 'sonar/status'
    SONAR_REQUEST_TOPIC = 'sonar/request'
    SONAR_IMAGE_TOPIC = 'sonar/image/compressed'

    NODE_NAME = 'sonar'

    CONSTANT_SWEEP_START = 100
    CONSTANT_SWEEP_END = 300

    VALUE_THRESHOLD = 95  # Sonar intensity threshold
    DBSCAN_EPS = 3  # DBSCAN epsilon
    DBSCAN_MIN_SAMPLES = 10  # DBSCAN min samples

    NUM_RETRIES = 10

    def __init__(self) -> None:
        super().__init__(self.NODE_NAME)
        self.get_logger().info('Sonar planning node initialized')

        self.stream = self.declare_parameter('stream', False).value
        self.debug = self.declare_parameter('debug', True).value

        with Path(rr.get_filename(self.CONFIG_FILE_PATH, use_protocol=False)).open() as f:
            self._config_data = yaml.safe_load(f)

        self.ftdi = self._config_data['ftdi']
        self.center_gradians = self._config_data['center_gradians']
        self.negate = self._config_data['negate']

        self.status_publisher = self.create_publisher(String, self.SONAR_STATUS_TOPIC, 10)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Enamble streaming
        self.cv_bridge = CvBridge()
        if self.stream:
            self.sonar_image_publisher = self.create_publisher(CompressedImage, self.SONAR_IMAGE_TOPIC, 10)

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
        self.number_of_samples = self.DEFAULT_NUMER_OF_SAMPLES
        self.ping360.set_number_of_samples(self.number_of_samples)

        self.sample_period = self.range_to_period(self.DEFAULT_RANGE)
        self.ping360.set_sample_period(self.sample_period)

        self.transmit_duration = self.range_to_transmit(self.DEFAULT_RANGE)
        self.ping360.set_transmit_duration(self.transmit_duration)

    def publish_status(self) -> None:
        """Publish the status of the sonar device."""
        # make string type std_msgs String
        self.status_publisher.publish(String(data='Sonar Running'))

    def range_to_period(self, sonar_range: int) -> int:
        """
        From a given range determines the sample_period.

        Sample_period is the time between each sample. Given a distance we can calculate the sample period using the
        formula: 2 * range / (number_of_samples * speed_of_sound_in_water * 25e-9) where number of samples is the
        number of samples taken between 0m and the set range, speed of sound in water is 1480m/s and 25e-9 is the
        sample period tick duration.

        https://discuss.bluerobotics.com/t/please-provide-some-answer-regards-ping360/6393/3

        Args:
            sonar_range (int): max range in meters of the sonar scan.

        Returns:
            sample_period (int): sample period in ms.
        """
        period = 2 * sonar_range / (self.number_of_samples * self.SPEED_OF_SOUND_IN_WATER
                                    * self.SAMPLE_PERIOD_TICK_DURATION)
        return round(period)

    def range_to_transmit(self, sonar_range: int) -> int:
        """
        From a given range determines the transmit_duration.

        Per firmware engineer:
        1. Starting point is TxPulse in usec = ((one-way range in metres) * 8000) /
        (Velocity of sound in metres per second)
        2. Then check that TxPulse is wide enough for currently selected sample interval in usec,
        i.e., if TxPulse < (2.5 * sample interval) then TxPulse = (2.5 * sample interval)
        3. Perform limit checking
        https://discuss.bluerobotics.com/t/please-provide-some-answer-regards-ping360/6393/3

        Args:
            sonar_range (int): max range in meters of the sonar scan.

        Returns:
            transmit_duration (int): max transmit duration in ms.
        """
        # 1
        transmit_duration = 8000 * sonar_range / self.SPEED_OF_SOUND_IN_WATER
        # 2 (transmit duration is microseconds, samplePeriod() is nanoseconds)
        transmit_duration = max(self.range_to_period(sonar_range) / 40, transmit_duration)
        # 3 min_transmit is 5 and max_transmit is 500
        return round(max(5, min(500, transmit_duration)))

    def set_new_range(self, sonar_range: int) -> None:
        """
        Set a new sample_period and transmit_duration.

        Args:
            sonar_range (int): max range in meters of the sonar scan.
        """
        self.prev_range = sonar_range
        self.sample_period = self.range_to_period(sonar_range)
        self.ping360.set_sample_period(self.sample_period)

        self.transmit_duration = self.range_to_transmit(sonar_range)
        self.ping360.set_transmit_duration(self.transmit_duration)

    def meters_per_sample(self) -> float:
        """
        Return the target distance per sample, in meters.

        https://discuss.bluerobotics.com/t/access-ping360-data-for-post-processing-python/10416/2

        Returns:
            float: Distance per sample.
        """
        # sample_period is in 25ns increments
        # time of flight includes there and back, so divide by 2
        return self.SPEED_OF_SOUND_IN_WATER * self.sample_period * self.SAMPLE_PERIOD_TICK_DURATION / 2.0

    def get_distance_of_sample(self, sample_index: int) -> float:
        """
        Get the distance in meters of a sample given its index in the data array returned from the device.

        Computes distance using formula from
        https://bluerobotics.com/learn/understanding-and-using-scanning-sonars/.

        Args:
            sample_index (int | float): Index of the sample in the data array, from 0 to N-1,
            where N = number of samples.

        Returns:
            float: Distance in meters of the sample from the sonar device.
        """
        # 0.5 for the average distance of sample
        return (sample_index + 0.5) * self.meters_per_sample()

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

        response_to_int_array = [int(item) for item in response.data]  # converts bytestring to int array
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
        for i in range(range_start, range_end + 1):  # inclusive ends
            sonar_scan = self.request_data_at_angle(i)
            sonar_sweep_data.append(sonar_scan)
        return np.vstack(sonar_sweep_data)

    def to_robot_position(self, angle: float, index: int, target_frame_id: str) -> Pose:
        """
        Convert a point in sonar space to a robot global position.

        Args:
            angle (float): Angle in gradians of the point relative to in front
                of the sonar device.
            index (int | float): Index of the data in the sonar response.
            target_frame_id (str): The target frame to transform the pose to.

        Returns:
            Pose: Pose in target_frame_id containing x and y position of angle/index item.
        """
        x_pos = self.get_distance_of_sample(index)*np.cos(
            sonar_utils.centered_gradians_to_radians(angle, self.center_gradians, self.negate))
        y_pos = -1 * self.get_distance_of_sample(index)*np.sin(
            sonar_utils.centered_gradians_to_radians(angle, self.center_gradians, self.negate))
        pos_of_point = Pose()
        pos_of_point.position.x = x_pos
        pos_of_point.position.y = y_pos
        pos_of_point.position.z = 0  # z cord is not really 0 but we don't care
        pos_of_point.orientation.x = 0
        pos_of_point.orientation.y = 0
        pos_of_point.orientation.z = 0
        pos_of_point.orientation.w = 1

        return sonar_utils.transform_pose(self.tf_buffer, pos_of_point, 'sonar_ping_360', target_frame_id)

    def get_xy_of_object_in_sweep(self, start_angle: int, end_angle: int, target_frame_id: str) -> \
            tuple[Pose, np.ndarray, float]:
        """
        Get the depth of the sweep of a detected object. For now uses mean value.

        Args:
            start_angle (int): Angle to start sweep in gradians.
            end_angle (int): Angle to end sweep in gradians.
            target_frame_id (str): The target frame to transform the object pose to.

        Returns:
            (Pose, List): Pose of the object in robot reference frame and sonar sweep array.
        """
        sonar_sweep_array = self.get_sweep(start_angle, end_angle)
        sonar_index, normal_angle, plot = sonar_image_processing.find_center_point_and_angle(
            sonar_sweep_array, self.VALUE_THRESHOLD, self.DBSCAN_EPS,
            self.DBSCAN_MIN_SAMPLES, True)

        color_image = sonar_image_processing.build_color_sonar_image_from_int_array(sonar_sweep_array)

        if sonar_index is None:
            return (None, color_image, None)

        sonar_angle = (start_angle + end_angle) / 2  # Take the middle of the sweep

        return (self.to_robot_position(sonar_angle, sonar_index, target_frame_id), plot, normal_angle)

    def convert_to_ros_compressed_img(self, sonar_sweep: np.ndarray, compressed_format: str = 'jpg',
                                      is_color: bool = False) -> CompressedImage:
        """
        Convert any kind of image to ROS Compressed Image.

        Args:
            sonar_sweep (int): numpy array of int values representing the sonar image.
            compressed_format (string): format to compress the image to.
            is_color (bool): Whether the image is color or not.

        Returns:
            CompressedImage: ROS Compressed Image message.
        """
        if not is_color:
            sonar_sweep = sonar_image_processing.build_color_sonar_image_from_int_array(sonar_sweep)
        return self.cv_bridge.cv2_to_compressed_imgmsg(sonar_sweep, dst_format=compressed_format)

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
                compressed_image = self.convert_to_ros_compressed_img(sonar_sweep)
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
        response.pose = Pose()
        response.normal_angle = 0.0
        response.is_object = False

        # Get request details
        left_degrees = request.start_angle
        right_degrees = request.end_angle
        new_range = request.distance_of_scan

        left_gradians = sonar_utils.degrees_to_centered_gradians(left_degrees, self.center_gradians, self.negate)
        right_gradians = sonar_utils.degrees_to_centered_gradians(right_degrees, self.center_gradians, self.negate)

        if self.negate:
            left_gradians, right_gradians = right_gradians, left_gradians

        self.get_logger().info(f'Recieved Sonar request: {left_gradians}, {right_gradians}, {new_range}')

        # Angle must be between 0 and 400 and range must be positive
        if left_gradians < 0 or right_gradians < 0 or right_gradians > 400 or new_range < 0: # noqa: PLR2004
            self.get_logger().error('Bad sonar request')
            return response

        if new_range != self.prev_range:
            self.set_new_range(new_range)

        object_pose, plot, normal_angle = self.get_xy_of_object_in_sweep(left_gradians, right_gradians,
            request.target_frame_id)

        if object_pose is not None:
            response.pose = object_pose
            response.normal_angle = normal_angle
            response.is_object = True

        if object_pose is None:
            self.get_logger().error('No object found')

        if self.stream:
            sonar_image = self.convert_to_ros_compressed_img(plot, is_color=True)
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
