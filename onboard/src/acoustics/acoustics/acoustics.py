import rclpy
from rclpy.node import Node
from custom_msgs.srv import AcousticsRequest


class Acoustics(Node):
    def __init__(self):
        super().__init__('acoustics')
        self.get_logger().info('Acoustics node initialized')

        # TODO: add additional setup code

    def run(self) -> None:
        """Run the main loop of the node."""
        self.create_service(AcousticsRequest, 'acoustics/request', self.perform_acoustics_request)

        # Create publishers for status, data and response

    def perform_acoustics_request(
            self, request: AcousticsRequest.Request, response: AcousticsRequest.Response
    ) -> AcousticsRequest.Response:
        """
        Perform an acoustics request.

        TODO: Insert some documentation, and process the request

        General steps
        1. Validate the request, if any args are needed
        2. Call whatever method in Acoustics needs to be called, and get the answer
        3. Populate the response, and return the response.
        """
        pass


def main(args=None):
    rclpy.init(args=args)
    node = Acoustics()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()