import rclpy
from custom_msgs.srv import SonarSweepRequest
from rclpy.node import Node


class SonarClient(Node):
    """Client node for sending sonar sweep requests."""
    def __init__(self) -> None:
        super().__init__('sonar_client')
        self.cli = self.create_client(SonarSweepRequest, 'sonar/request')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')

    def request_sweep(self, start_angle: float, end_angle: float, distance_of_scan: float) -> None:
        """Send a sonar sweep request."""
        req = SonarSweepRequest.Request()
        req.start_angle = start_angle
        req.end_angle = end_angle
        req.distance_of_scan = distance_of_scan
        self.future = self.cli.call_async(req)

def main(args: list | None = None) -> None:
    """Send a sonar sweep request."""
    rclpy.init(args=args)
    client_node = SonarClient()

    # Send the request
    client_node.request_sweep(-90, 90, 5.0)

    # Spin until the future is complete
    rclpy.spin_until_future_complete(client_node, client_node.future)

    if client_node.future.result() is not None:
        response = client_node.future.result()
        print(f'Sonar sweep response: {response.pose}, {response.normal_angle}, {response.is_object}')
    else:
        print('Service call failed.')

    client_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
