import rclpy
from custom_msgs.srv import AcousticsRequest
from rclpy.node import Node

from acoustics.acoustics_v3.scripts import controller


class Acoustics(Node):
    """Acoustics node to interface with the Logic2 software."""
    def __init__(self) -> None:
        super().__init__('acoustics')
        self.get_logger().info('Acoustics node initialized')

        self.create_service(AcousticsRequest, 'acoustics/request', self.perform_acoustics_request)
        self.get_logger().info('Service initialized')
        # TODO: add additional setup code

    def perform_acoustics_request(
            self, request: AcousticsRequest.Request, response: AcousticsRequest.Response,
    ) -> AcousticsRequest.Response:
        """
        Perform an acoustics request.

        TODO: Insert some documentation, and process the request

        General steps
        1. Validate the request, if any args are needed
        2. Call whatever method in Acoustics needs to be called, and get the answer
        3. Populate the response, and return the response.
        """
        self.get_logger().info('Receieved acoustics request')

        # Parse request details, if any

        # Call controller, should intialize logic and do all of the work
        closest, nearby, valid = controller.main()

        # Post processing
        response.closest = closest
        response.nearby = nearby

        return response


def main(args: list[str] | None = None) -> None:
    """Initialize and run the acoustics node."""
    rclpy.init(args=args)
    node = Acoustics()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
