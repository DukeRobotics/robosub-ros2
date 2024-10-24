import rclpy
from rclpy.node import Node
from example_interfaces.srv import SetBool
from rclpy.callback_groups import ReentrantCallbackGroup
from task_planning.utils.other_utils import singleton

"""
TODO:ros2

Look at the other interfaces. MarkDropper does *not* inherit from rclypy.node since it should receive a node
object in its constructor (note that MarkerDropper is a @singleton).
"""

@singleton
class MarkerDropper:

    # ROS service names
    SERVO_CONTROL_SERVICE = 'marker_dropper/servo_control'

    def __init__(self, node: Node, bypass: bool = False):
        self.node = node

        # Create a callback group that allows concurrent callbacks
        self.callback_group = ReentrantCallbackGroup()

        # Create a client for the servo_control service
        self.drop_marker_client = node.create_client(SetBool, self.SERVO_CONTROL_SERVICE)

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
