import os
from pathlib import Path

import depthai as dai
import nmap
import rclpy
from rclpy.node import Node
import resource_retriever as rr
import yaml


def connect(node: Node, camera: str, pipeline: dai.Pipeline) -> dai.Device:
    """
    Connect to the DepthAI camera and upload the pipeline.

    It tries to connect five times, waiting 2 seconds before trying again (so the script is successful if the camera is
    just temporarily unavailable).

    Args:
        node (Node): The ROS2 node.
        camera (str): The name of the camera to connect to.
        pipeline (depthai.Pipeline): The DepthAI pipeline to upload to the camera.

    Returns:
        depthai.Device: DepthAI.Device object.

    Raises:
        RuntimeError: If a successful connection to the camera could not be made.
    """
    # Load the configuration file
    config_file_path = f'package://cv/config/{os.getenv('ROBOT_NAME')}.yaml'
    with Path(rr.get_filename(config_file_path, use_protocol=False)).open() as f:
        config = yaml.safe_load(f)

    # Check if the camera is valid
    if camera not in config["depthai"]["devices"]:
        error_msg = f'Invalid camera: "{camera}". Valid cameras are: {list(config["depthai"]["devices"].keys())}'
        raise ValueError(error_msg)

    # Number of attempts that will be made to connect to the camera
    total_tries = 5

    for i in range(total_tries):
        if not rclpy.ok():
            break

        node.get_logger().info(f'Attempting to connect to DepthAI camera {camera}, try {i + 1}...')

        try:
            # Scan for camera IP address using custom autodiscovery
            ip = custom_autodiscovery(
                config["depthai"]["devices"][camera]["mac"],
                config["depthai"]["ip_range"]
            )

            # Try connecting with the discovered IP address
            # If the execution reaches the following return statement, the lines above did not raise an exception, so a
            # successful camera connection was made, and device should be returned
            device = dai.Device(pipeline, dai.DeviceInfo(ip))
            node.get_logger().info(f'Connected to DepthAI camera {camera} at IP {ip}')
            return device

        except RuntimeError:
            pass

        # Wait two seconds before trying again
        # This ensures the script does not terminate if the camera is just temporarily unavailable
        rclpy.spin_once(node, timeout_sec=2)

    error_message = f'{total_tries} attempts were made to connect to the DepthAI camera using autodiscovery and static \
          IP address specification. All attempts failed.'

    raise RuntimeError(error_message)


def custom_autodiscovery(mac_address: str, ip_range: str) -> str:
    """
    Scan specified IP range looking for the DepthAI camera's MAC address.

    Args:
        mac_address (str): DepthAI camera MAC address.
        ip_range (str): IP range to scan. In Classless Inter-Domain Routing (CIDR) format.

    Returns:
        str: DepthAI IP address.

    Raises:
        RuntimeError: If the camera's IP address could not be found.
    """
    nm = nmap.PortScanner()
    scan = nm.scan(hosts=ip_range, arguments='-sn', sudo=True)
    scan = scan['scan']

    for ip, info in scan.items():
        if info['status']['state'] == 'up' and info['addresses'].get('mac') == mac_address:
            return ip

    error_message = 'Custom autodiscovery failed to find camera.'
    raise RuntimeError(error_message)


def main(args: None = None) -> None:
    """Run the node."""
    rclpy.init(args=args)
    node = Node('depthai_camera_connect')
    camera = node.declare_parameter('camera', 'front').get_parameter_value().string_value

    try:
        connect(node, camera, dai.Pipeline())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
