import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_msgs.msg import TFMessage


class DummyStatePublisher(Node):
    """Node that publishes dummy transform and odometry messages between the 'odom' and 'base_link' frames."""

    PUBLISH_RATE = 20.0  # Hz

    def __init__(self) -> None:
        super().__init__('dummy_state_publisher')

        # Odometry publisher
        self.odom_publisher = self.create_publisher(Odometry, '/state', 10)

        # TF publisher
        self.tf_publisher = self.create_publisher(TFMessage, '/tf', 10)

        # Timer to publish messages
        self.timer = self.create_timer(1.0 / self.PUBLISH_RATE, self.publish_messages)

        # Dummy odometry message
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = 'odom'
        self.odom_msg.child_frame_id = 'base_link'

        # Dummy position
        self.odom_msg.pose.pose.position.x = 0.0
        self.odom_msg.pose.pose.position.y = 0.0
        self.odom_msg.pose.pose.position.z = 0.0

        # Dummy orientation (identity quaternion)
        self.odom_msg.pose.pose.orientation.x = 0.0
        self.odom_msg.pose.pose.orientation.y = 0.0
        self.odom_msg.pose.pose.orientation.z = 0.0
        self.odom_msg.pose.pose.orientation.w = 1.0

        # Dummy velocity
        self.odom_msg.twist.twist.linear.x = 0.0
        self.odom_msg.twist.twist.angular.z = 0.0

        # Dummy transform
        self.transform = TransformStamped()
        self.transform.header.frame_id = 'odom'
        self.transform.child_frame_id = 'base_link'

        # Set translation (position)
        self.transform.transform.translation.x = 0.0
        self.transform.transform.translation.y = 0.0
        self.transform.transform.translation.z = 0.0

        # Set rotation (identity quaternion)
        self.transform.transform.rotation.x = 0.0
        self.transform.transform.rotation.y = 0.0
        self.transform.transform.rotation.z = 0.0
        self.transform.transform.rotation.w = 1.0

        # Dummy transform message
        self.tf_message = TFMessage()
        self.tf_message.transforms.append(self.transform)

    def publish_messages(self) -> None:
        """Publish the dummy transform and odometry messages."""
        self.get_logger().info('Publishing dummy messages')
        self.publish_transform()
        self.publish_odom()

    def publish_transform(self) -> None:
        """Publish a dummy transform message."""
        # Set the current time
        self.tf_message.transforms[0].header.stamp = self.get_clock().now().to_msg()

        # Publish the message
        self.tf_publisher.publish(self.tf_message)

    def publish_odom(self) -> None:
        """Publish a dummy odometry message."""
        # Set the current time
        self.odom_msg.header.stamp = self.get_clock().now().to_msg()

        # Publish the message
        self.odom_publisher.publish(self.odom_msg)

def main(args: list[str] | None = None) -> None:
    """Create and run the dummy state publisher node."""
    rclpy.init(args=args)
    node = DummyStatePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
