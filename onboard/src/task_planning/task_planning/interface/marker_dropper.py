import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy

@singleton
class MarkerDropper(Node):
    def __init__(self, bypass: bool= False):
        super().__init__('marker_dropper')

        # Create a callback group that allows concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()

        # Create a client for the servo_control service
        self.drop_marker_client = self.create_client(
            SetBool,
            'servo_control',
            callback_group=self.callback_group,
            qos_profile=QoSProfile(reliability=ReliabilityPolicy.RELIABLE)
        )

        if not bypass:
            while not self.drop_marker_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info('servo_control service not available, waiting again...')

    def drop_marker(self, data: bool):
        # Create a request
        request = SetBool.Request()
        request.data = data  # Assuming True means "drop the marker"

        # Call the service
        future = self.drop_marker_client.call_async(request)
        return future