import depthai as dai
import nmap
import rclpy
from rclpy.node import Node


class DepthAiCameraConnect(Node):
    """Temporary node declared to make use of rclpy.spin_once."""

    def __init__(self) -> None:
        super().__init__('depthai_camera_connect')

def connect(pipeline: dai.Pipeline) -> dai.Device:
    """
    Connect to the DepthAI camera and upload the pipeline.

    It tries to connect five times, waiting 2 seconds before trying again (so the script is successful if the camera is
    just temporarily unavailable). In each try, it attempts to connect first using custom autodiscovery, then using
    DepthAI autodiscovery, then finally using static IP address specification.

    Args:
        pipeline (depthai.Pipeline): The DepthAI pipeline to upload to the camera.

    Returns:
        depthai.Device: DepthAI.Device object.

    Raises:
        RuntimeError: If a successful connection to the camera could not be made.
    """
    # Number of attempts that will be made to connect to the camera
    total_tries = 5

    # A node

    node = DepthAiCameraConnect()

    for _ in range(total_tries):
        if not rclpy.ok():
            break

        try:
            # Scan for camera IP address using custom autodiscovery
            ip = custom_autodiscovery()

            # Try connecting with the discovered IP address
            # If the execution reaches the following return statement, the lines above did not raise an exception, so a
            # successful camera connection was made, and device should be returned
            return dai.Device(pipeline, dai.DeviceInfo(ip))
        except RuntimeError:
            pass

        if not rclpy.ok():
            break

        # Wait two seconds before trying again
        # This ensures the script does not terminate if the camera is just temporarily unavailable
        rclpy.spin_once(node, timeout_sec=2)
    error_message = f'{total_tries} attempts were made to connect to the DepthAI camera using autodiscovery and static \
          IP address specification. All attempts failed.'
    raise RuntimeError(error_message)


def custom_autodiscovery() -> str:
    """
    Scan all IP addresses from 192.168.1.0 to 192.168.1.255 looking for the DepthAI camera's MAC address.

    Returns:
        str: DepthAI IP address.

    Raises:
        RuntimeError: If the camera's IP address could not be found.
    """
    mac_address = '44:A9:2C:3C:0A:90'  # DepthAI camera MAC address
    ip_range = '192.168.1.0/24'  # 192.168.1.0 to 192.168.1.255

    nm = nmap.PortScanner()
    scan = nm.scan(hosts=ip_range, arguments='-sP')
    scan = scan['scan']

    for ip, info in scan.items():
        print(f"Searching,... {ip} {info['addresses']}")
        if info['status']['state'] == 'up' and info['addresses'].get('mac') == mac_address:
            print('DepthAI Camera Found')
            return ip

    error_message = 'Custom autodiscovery failed to find camera.'
    raise RuntimeError(error_message)
