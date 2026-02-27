import rclpy
from rclpy.node import Node
from custom_msgs.srv import AcousticsRequest
from acoustics.acoustics_v3 import controller

class Acoustics(Node):
    def __init__(self):
        super().__init__('acoustics')
        self.get_logger().info('Acoustics node initialized')

        self.create_service(AcousticsRequest, 'acoustics/request', self.perform_acoustics_request)
        # TODO: add additional setup code

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
        self.get_logger.info(f'Receieved acoustics request')

        # Parse request details, if any

        # Call controller, should intialize logic and do all of the work
        closest, nearby = controller.main()

        # Post processing
        response.closest = closest
        response.nearby = nearby

        return response


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