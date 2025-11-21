import numpy as np
import rclpy
import sonar_image_processing
import tf2_geometry_msgs
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose
from sensor_msgs.msg import CompressedImage

RADIANS_PER_GRADIAN = np.pi / 200
GRADIANS_PER_DEGREE = 400 / 360
SPEED_OF_SOUND_IN_WATER = 1482 # m/s
SAMPLE_PERIOD_TICK_DURATION = 25e-9  # s

def transform_pose(buffer: tf2_ros.Buffer, pose: tf2_geometry_msgs.PoseStamped,
                   source_frame_id: str, target_frame_id: str) -> tf2_geometry_msgs.PoseStamped:
    """
    Transform pose from source reference frame to target reference frame.

    Args:
        buffer (tf2_ros.Buffer): The tf2 buffer containing transform data.
        pose (Pose): Pose in the source reference frame.
        source_frame_id (str): Name of the source frame (e.g., 'sonar_ping_360').
        target_frame_id (str): Name of the target frame (e.g., 'camera_depthai_front').

    Returns:
        Pose: Pose in the target reference frame.
    """
    try:
        # Wait for transform to be available
        transform = buffer.lookup_transform(
            target_frame_id, source_frame_id, rclpy.time.Time(),
        )

        # Transform the pose
        return tf2_geometry_msgs.do_transform_pose(pose, transform)

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException,
            tf2_ros.InvalidArgumentException) as e:
        error_message = f'Failed to transform pose: {e}'
        raise RuntimeError(error_message) from e


def centered_gradians_to_radians(angle_gradians: float, center_gradians: float, increase_ccw: bool) -> float:
    """
    Convert gradians centered at center_gradians to radians centered at 0.

    Args:
        angle_gradians (float): Angle in gradians where center_gradians is forward.
        center_gradians (float): Center gradians to use for conversion.
        increase_ccw (bool): If True, sonar angles increase counter-clockwise when viewed from above the robot.

    Returns:
        float: Angle in radians where 0 is forward.
    """
    return (angle_gradians - center_gradians) * RADIANS_PER_GRADIAN * (1 if increase_ccw else -1)


def degrees_to_centered_gradians(angle_degrees: float, center_gradians: float, increase_ccw: bool) -> int:
    """
    Convert degrees centered at 0 to gradians centered at center_gradians.

    Args:
        angle_degrees (float): Angle in degrees where 0 is forward.
        center_gradians (float): Center gradians to use for conversion.
        increase_ccw (bool): If True, sonar angles increase counter-clockwise when viewed from above the robot.

    Returns:
        int: Angle in gradians where center_gradians is forward.
    """
    angle_gradians = angle_degrees * GRADIANS_PER_DEGREE * (1 if increase_ccw else -1)
    angle_gradians_centered = angle_gradians + center_gradians
    return int(angle_gradians_centered)

def range_to_period(sonar_range: int, number_of_samples : int) -> int:
    """
    From a given range determines the sample_period.

    Sample_period is the time between each sample. Given a distance we can calculate the sample period using the
    formula: 2 * range / (number_of_samples * speed_of_sound_in_water * 25e-9) where number of samples is the
    number of samples taken between 0m and the set range, speed of sound in water is 1480m/s and 25e-9 is the
    sample period tick duration.

    https://discuss.bluerobotics.com/t/please-provide-some-answer-regards-ping360/6393/3

    Args:
        sonar_range (int): max range in meters of the sonar scan.
        number_of_samples (int): number of samples taken between 0m and the set range

    Returns:
        sample_period (int): sample period in ms.
    """
    period = 2 * sonar_range / (number_of_samples * SPEED_OF_SOUND_IN_WATER
                                * SAMPLE_PERIOD_TICK_DURATION)
    return round(period)

def range_to_transmit(sonar_range: int, number_of_samples : int) -> int:
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
        number_of_samples (int): number of samples taken between 0m and the set range

    Returns:
        transmit_duration (int): max transmit duration in ms.
    """
    # 1
    transmit_duration = 8000 * sonar_range / SPEED_OF_SOUND_IN_WATER
    # 2 (transmit duration is microseconds, samplePeriod() is nanoseconds)
    transmit_duration = max(range_to_period(sonar_range, number_of_samples) / 40,
                            transmit_duration)
    # 3 min_transmit is 5 and max_transmit is 500
    return round(max(5, min(500, transmit_duration)))

def meters_per_sample(sample_period : int) -> float:
    """
    Return the target distance per sample, in meters.

    https://discuss.bluerobotics.com/t/access-ping360-data-for-post-processing-python/10416/2

    Args:
        sample_period (int): the sample period of the ping360

    Returns:
        float: Distance per sample.
    """
    # sample_period is in 25ns increments
    # time of flight includes there and back, so divide by 2
    return SPEED_OF_SOUND_IN_WATER * sample_period * SAMPLE_PERIOD_TICK_DURATION / 2.0

def get_distance_of_sample(sample_period: int, sample_index: int) -> float:
    """
    Get the distance in meters of a sample given its index in the data array returned from the device.

    Computes distance using formula from
    https://bluerobotics.com/learn/understanding-and-using-scanning-sonars/.

    Args:
        sample_period: the sample period of the Ping360
        sample_index (int | float): Index of the sample in the data array, from 0 to N-1,
        where N = number of samples.

    Returns:
        float: Distance in meters of the sample from the sonar device.
    """
    # 0.5 for the average distance of sample
    return (sample_index + 0.5) * meters_per_sample(sample_period)

def to_robot_position(angle: float, index: int, target_frame_id: str, sample_period: int,
                      center_gradians: int, negate: bool, tf_buffer: tf2_ros.Buffer) -> Pose:
    """
    Convert a point in sonar space to a robot global position.

    Args:
        angle (float): Angle in gradians of the point relative to in front
            of the sonar device.
        index (int | float): Index of the data in the sonar response.
        target_frame_id (str): The target frame to transform the pose to.
        sample_period (int): the sample period of the ping360
        center_gradians (int): the gradian value of the center of the scan
        negate (bool): whether to negate the pose or not
        tf_buffer: tf2 buffer containing transform data

    Returns:
        Pose: Pose in target_frame_id containing x and y position of angle/index item.
    """
    x_pos = get_distance_of_sample(
            sample_period, index,
        )*np.cos(
            centered_gradians_to_radians(angle, center_gradians, negate),
        )
    y_pos = -1 * get_distance_of_sample(
            sample_period, index,
        )*np.sin(
            centered_gradians_to_radians(angle, center_gradians, negate),
        )
    pos_of_point = Pose()
    pos_of_point.position.x = x_pos
    pos_of_point.position.y = y_pos
    pos_of_point.position.z = 0  # z cord is not really 0 but we don't care
    pos_of_point.orientation.x = 0
    pos_of_point.orientation.y = 0
    pos_of_point.orientation.z = 0
    pos_of_point.orientation.w = 1

    return transform_pose(tf_buffer, pos_of_point, 'sonar_ping_360', target_frame_id)

def convert_to_ros_compressed_img(sonar_sweep: np.ndarray, cv_bridge: CvBridge,
                                  compressed_format: str = 'jpg', is_color: bool = False) -> CompressedImage:
    """
    Convert any kind of image to ROS Compressed Image.

    Args:
        sonar_sweep (int): numpy array of int values representing the sonar image.
        cv_bridge: the CV Bridge to use to convert to CompressedImage.
        compressed_format (string): format to compress the image to.
        is_color (bool): Whether the image is color or not.

    Returns:
        CompressedImage: ROS Compressed Image message.
    """
    if not is_color:
        sonar_sweep = sonar_image_processing.build_color_sonar_image_from_int_array(sonar_sweep)
    return cv_bridge.cv2_to_compressed_imgmsg(sonar_sweep, dst_format=compressed_format)
