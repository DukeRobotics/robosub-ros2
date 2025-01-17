from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.logging import get_logger
from rclpy.node import Node
from robot_localization.srv import SetPose
from sensor_msgs.msg import Imu
from task_planning.utils.other_utils import singleton
from tf2_ros.buffer import Buffer

logger = get_logger('state')

@singleton
class State:
    """
    Interface for the state of the robot.

    Attributes:
        _instance: The singleton instance of this class. Is a static attribute.
        state: The current state of the robot.
        tfBuffer: The transform buffer for the robot.
        _reset_pose: The service proxy for resetting the pose
    """

    # ROS topics for the state and resetting the pose
    STATE_TOPIC = 'state'
    DEPTH_TOPIC = '/sensors/depth'
    IMU_TOPIC = '/vectornav/IMU'
    RESET_POSE_SERVICE = '/set_pose'

    def __init__(self, node: Node, bypass: bool = False, tf_buffer: Buffer = None) -> None:
        """
        Initialize the state.

        Args:
            node: The ROS node used for communication and state management.
            bypass: If True, bypass certain checks. Defaults to False.
            tf_buffer: The transform buffer for the robot. Defaults to None.
        """
        self.bypass = bypass
        if tf_buffer:
            self._tf_buffer = tf_buffer

        self.received_state = False
        self.received_depth = False
        self.received_imu = False

        node.create_subscription(Odometry, self.STATE_TOPIC, self._on_receive_state, 10)
        self._state = None

        self._reset_pose = node.create_client(SetPose, self.RESET_POSE_SERVICE)
        if not bypass:
            while not self._reset_pose.wait_for_service(timeout_sec=1.0):
                logger.info('%s not ready, waiting...', self.RESET_POSE_SERVICE)

        node.create_subscription(PoseWithCovarianceStamped, self.DEPTH_TOPIC, self._on_receive_depth, 10)

        node.create_subscription(Imu, self.IMU_TOPIC, self._on_receive_imu, 10)

    @property
    def state(self) -> Odometry:
        """The state."""
        return self._state

    @property
    def orig_state(self) -> Odometry:
        """The first state message received."""
        return self._orig_state

    @property
    def depth(self) -> float:
        """The depth from the pressure sensor."""
        return self._depth

    @property
    def orig_depth(self) -> float:
        """The depth from the pressure sensor."""
        return self._orig_depth

    @property
    def imu(self) -> Imu:
        """The IMU data."""
        return self._imu

    @property
    def orig_imu(self) -> Imu:
        """The first IMU message received."""
        return self._orig_imu

    @property
    def tf_buffer(self) -> Buffer:
        """The transform buffer."""
        return self._tf_buffer

    def _on_receive_state(self, state : Odometry) -> None:
        self._state = state

        if not self.received_state:
            self._orig_state = state
            self.received_state = True

    def _on_receive_depth(self, depth_msg : PoseWithCovarianceStamped) -> None:
        self._depth = depth_msg.pose.pose.position.z

        if not self.received_depth:
            self._orig_depth = depth_msg.pose.pose.position.z
            self.received_depth = True

    def _on_receive_imu(self, imu_msg : Imu) -> None:
        self._imu = imu_msg

        if not self.received_imu:
            self._orig_imu = imu_msg
            self.received_imu = True

    def reset_pose(self) -> None:
        """Reset the pose."""
        posecov = PoseWithCovarianceStamped()
        posecov.pose.pose.orientation.w = 1
        if not self.bypass:
            self._reset_pose(posecov)
