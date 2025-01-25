import math
from collections.abc import Callable
from typing import ClassVar, TypeVar

import rclpy
from geometry_msgs.msg import Pose, Twist, Vector3
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu
from transforms3d.euler import quat2euler


# Generic type for ROS messages
RosMessage = TypeVar('RosMessage')


class TopicTransformData:
    """
    Class to store data for topic transformations.

    Attributes:
        input_topic (str): Topic name for input data.
        input_type (type): ROS message type for input data.
        input_type_conversion (Callable): Function to convert input data.
        output_topic (str): Topic name for output data.
        output_type (type): ROS message type for output data.
        output_type_conversion (Callable): Function to convert output data.
        subscriber (Optional[Callable]): Subscriber for the input topic.
        publisher (Optional[Callable]): Publisher for the output topic.
        publisher_queue_size (int): Queue size for the publisher.
        subscriber_queue_size (int): Queue size for the subscriber.
    """
    def __init__(
        self,
        input_topic: str,
        input_type: type[RosMessage],
        input_type_conversion: Callable[[RosMessage], RosMessage],
        output_topic: str,
        output_type: type[RosMessage],
        output_type_conversion: Callable[[RosMessage], RosMessage],
        subscriber: Callable | None = None,
        publisher: Callable | None = None,
        publisher_queue_size: int = 1,
        subscriber_queue_size: int = 1,
    ) -> None:
        self.input_topic = input_topic
        self.input_type = input_type
        self.input_type_conversion = input_type_conversion
        self.output_topic = output_topic
        self.output_type = output_type
        self.output_type_conversion = output_type_conversion
        self.subscriber = subscriber
        self.publisher = publisher
        self.publisher_queue_size = publisher_queue_size
        self.subscriber_queue_size = subscriber_queue_size

class Conversions:
    """Class to store conversion functions."""

    @staticmethod
    def quat_to_vector(quat_msg: Imu) -> Vector3:
        """
        Convert a quaternion to a Vector3 representing Euler angles in degrees.

        Args:
            quat_msg (Imu): Quaternion message.

        Returns:
            Vector3: Converted Euler angles as a Vector3.
        """
        euler_angles = quat2euler([quat_msg.w, quat_msg.x, quat_msg.y, quat_msg.z])
        euler_angles = [math.degrees(angle) for angle in euler_angles]

        vector_msg = Vector3()
        vector_msg.x = euler_angles[0]
        vector_msg.y = euler_angles[1]
        vector_msg.z = euler_angles[2]

        return vector_msg

    @staticmethod
    def pose_to_twist(pose_msg: Pose) -> Twist:
        """
        Convert a Pose message to a Twist message.

        Args:
            pose_msg (Pose): Pose message.

        Returns:
            Twist: Twist message with linear and angular components.
        """
        twist_msg = Twist()
        twist_msg.linear.x = pose_msg.position.x
        twist_msg.linear.y = pose_msg.position.y
        twist_msg.linear.z = pose_msg.position.z

        twist_msg.angular = Conversions.quat_to_vector(pose_msg.orientation)

        return twist_msg

class TopicTransforms(Node):
    """Node to perform topic transformations."""
    NODE_NAME: ClassVar[str] = 'topic_transforms'

    TOPIC_TRANSFORM_DATA: ClassVar[list[TopicTransformData]] = [
        TopicTransformData(
            '/state',
            Odometry,
            lambda x: x.pose.pose,
            '/transforms/state/pose',
            Twist,
            Conversions.pose_to_twist,
        ),
        TopicTransformData(
            '/vectornav/IMU',
            Imu,
            lambda x: x.orientation,
            '/transforms/vectornav/IMU/orientation',
            Vector3,
            Conversions.quat_to_vector,
        ),
        TopicTransformData(
            '/controls/desired_position',
            Pose,
            lambda x: x,
            '/transforms/controls/desired_position',
            Twist,
            Conversions.pose_to_twist,
        ),
    ]

    def __init__(self) -> None:
        """Initialize the TopicTransforms node."""
        super().__init__(self.NODE_NAME)

        for data in self.TOPIC_TRANSFORM_DATA:
            data.subscriber = self.create_subscription(
                data.input_type,
                data.input_topic,
                lambda msg, data=data: self.callback(data, msg),
                data.subscriber_queue_size,
            )
            data.publisher = self.create_publisher(
                data.output_type,
                data.output_topic,
                data.publisher_queue_size,
            )

        self.get_logger().info('Topic transforms node started.')

    def callback(self, data: TopicTransformData, msg: RosMessage) -> None:
        """
        Execute callback function to transform input message and publish output message.

        Args:
            data (TopicTransformData): Data for the transformation.
            msg (RosMessage): Input message.
        """
        converted_input = data.input_type_conversion(msg)
        output_msg = data.output_type_conversion(converted_input)
        data.publisher.publish(output_msg)

def main(args: list[str] | None) -> None:
    """
    Run main entry point for the TopicTransforms node.

    Args:
        args (Optional): Command-line arguments.
    """
    rclpy.init(args=args)
    topic_transforms = TopicTransforms()

    try:
        rclpy.spin(topic_transforms)
    except KeyboardInterrupt:
        pass
    finally:
        topic_transforms.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
