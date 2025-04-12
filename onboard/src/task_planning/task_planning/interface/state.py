from geometry_msgs.msg import PoseWithCovarianceStamped, Vector3
from task_planning.utils import geometry_utils
from nav_msgs.msg import Odometry
from rclpy.logging import get_logger
from rclpy.node import Node
from robot_localization.srv import SetPose
from sensor_msgs.msg import Imu
from task_planning.utils.other_utils import singleton
from tf2_ros.buffer import Buffer

logger = get_logger('state_interface')

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
    IMU_TOPIC = '/vectornav/imu'
    RESET_POSE_SERVICE = '/set_pose'

    def __init__(self, node: Node, bypass: bool = False, tf_buffer: Buffer | None = None) -> None:
        """
        Initialize the state.

        Args:
            node: The ROS node used for communication and state management.
            bypass: If True, bypass certain checks. Defaults to False.
            tf_buffer: The transform buffer for the robot. Defaults to None.
        """
        self.bypass = bypass
        self._tf_buffer = tf_buffer if tf_buffer else Buffer()

        self._received_state = False
        self._received_depth = False
        self._received_imu = False

        self._state = None
        self._orig_state = None
        self._depth = 0
        self._orig_depth = 0
        self._imu = Imu()
        self._orig_imu = Imu()

        node.create_subscription(Odometry, self.STATE_TOPIC, self._on_receive_state, 10)

        self._reset_pose = node.create_client(SetPose, self.RESET_POSE_SERVICE)
        if not bypass:
            while not self._reset_pose.wait_for_service(timeout_sec=1.0):
                logger.info(f'{self.RESET_POSE_SERVICE} not ready, waiting...')

        node.create_subscription(PoseWithCovarianceStamped, self.DEPTH_TOPIC, self._on_receive_depth, 10)

        node.create_subscription(Imu, self.IMU_TOPIC, self._on_receive_imu, 10)

    @property
    def received_state(self) -> bool:
        """Whether the state has been received."""
        return self._received_state

    @property
    def state(self) -> Odometry | None:
        """The state."""
        return self._state

    @property
    def orig_state(self) -> Odometry | None:
        """The first state message received."""
        return self._orig_state

    @property
    def depth(self) -> float:
        """The depth from the pressure sensor."""
        return self._depth

    @property
    def orig_depth(self) -> float:
        """The first depth message received from the pressure sensor."""
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
    def imu_euler_angles(self) -> Vector3:
        """Euler angles of IMU orientation in radians."""
        return self._imu_euler_angles

    @property
    def tf_buffer(self) -> Buffer:
        """The transform buffer."""
        return self._tf_buffer

    def _on_receive_state(self, state: Odometry) -> None:
        self._state = state

        if not self._received_state:
            self._orig_state = state
            self._received_state = True

    def _on_receive_depth(self, depth_msg: PoseWithCovarianceStamped) -> None:
        self._depth = depth_msg.pose.pose.position.z

        if not self._received_depth:
            self._orig_depth = depth_msg.pose.pose.position.z
            self._received_depth = True

    def _on_receive_imu(self, imu_msg: Imu) -> None:
        self._imu = imu_msg

        if not self._received_imu:
            self._orig_imu = imu_msg
            self._received_imu = True
        
        self._imu_euler_angles = geometry_utils.geometry_quat_to_euler_angles(self._imu.orientation)

    def reset_pose(self) -> None:
        """Reset the pose."""
        request = SetPose.Request()
        request.pose = PoseWithCovarianceStamped()
        request.pose.pose.pose.orientation.w = 1.0

        if not self.bypass:
            self._reset_pose.call_async(request)
