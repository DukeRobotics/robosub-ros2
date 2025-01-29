import rclpy
from custom_msgs.msg import ServoAngleArray
from custom_msgs.srv import SetServo
from rclpy.node import Node


class ServoWrapperPublisher(Node):
    """Node that publishes servo angles at a fixed rate and offers a service to set a given servo's angle."""

    NUM_SERVOS = 8
    PUBLISHER_TOPIC = 'offboard/servo_angles'
    SERVICE_TOPIC = 'offboard/set_servo_angle'

    def __init__(self) -> None:
        super().__init__('servo_wrapper')

        # Create the publisher
        self.publisher_ = self.create_publisher(ServoAngleArray, self.PUBLISHER_TOPIC, 10)

        # Create the service
        self.srv_ = self.create_service(SetServo, self.SERVICE_TOPIC, self.set_servo_angle_callback)

        # Internal storage for servo angles
        self.angles = [0] * self.NUM_SERVOS

        # Create a timer to publish servo angles at 15 Hz
        timer_period = 1.0 / 15.0  # seconds
        self.timer_ = self.create_timer(timer_period, self.publish_servo_angles)

    def set_servo_angle_callback(self, request, response):
        """
        Callback for the SetServo service. Validates the servo number and angle,
        updates internal angles, and returns success/failure.
        """
        if 0 <= request.num < self.NUM_SERVOS and 0 <= request.angle <= 180:
            self.angles[request.num] = request.angle
            response.success = True
        else:
            response.success = False

        return response

    def publish_servo_angles(self) -> None:
        """Timer callback that periodically publishes the servo angles."""
        msg = ServoAngleArray()
        # Assuming ServoAngleArray contains a field like `angles` that is a list
        msg.angles = self.angles
        self.publisher_.publish(msg)


def main(args: list[str] | None = None) -> None:
    """Execute main entry point for the servo_wrapper node."""
    rclpy.init(args=args)

    servo_wrapper_publisher = ServoWrapperPublisher()

    try:
        rclpy.spin(servo_wrapper_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        servo_wrapper_publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
