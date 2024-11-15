#!/usr/bin/env python3

import depthai as dai
import rclpy
from rclpy.node import Node
import nmap


class DepthAiCameraConnect(Node):
    def __init__(self):
        super().__init__("depthai_camera_connect")

def connect(pipeline):
    """
    Connects to the DepthAI camera and uploads the pipeline.

    It tries to connect five times, waiting 2 seconds before trying again (so the script is successful if the camera is
    just temporarily unavailable). In each try, it attempts to connect first using custom autodiscovery, then using
    DepthAI autodiscovery, then finally using static IP address specification.

    :param pipeline: The DepthAI pipeline to upload to the camera.
    :return: depthai.Device object
    :raises RuntimeError: if a successful connection to the camera could not be made
    """

    # The DepthAI device that we hope to obtain
    # If device is not None, a successful connection to the camera was made
    device = None

    # Number of attempts that will be made to connect to the camera
    total_tries = 5

    # A node
    node = DepthAiCameraConnect()

    for _ in range(total_tries):
        if not rclpy.ok():
            break

        try:
            # Scan for camera IP address using custom autodiscovery
            ip = custom_autodiscovery()  # TODO: use MAC address from camera config file

            # Try connecting with the discovered IP address
            device = dai.Device(pipeline, dai.DeviceInfo(ip))

            # If the execution reaches the following return statement, the lines above did not raise an exception, so a
            # successful camera connection was made, and device should be returned
            return device

        except RuntimeError:
            pass

        if not rclpy.ok():
            break

        # Wait two seconds before trying again
        # This ensures the script does not terminate if the camera is just temporarily unavailable
        rclpy.spin_once(node, timeout_sec=2)

    raise RuntimeError(f"{total_tries} attempts were made to connect to the DepthAI camera using "
                    f"autodiscovery and static IP address specification. All attempts failed.")


def custom_autodiscovery():
    """
    Scans all IP addresses from 192.168.1.0 to 192.168.1.255 looking for the DepthAI camera's MAC address.

    :return: DepthAI IP address string
    :raises RuntimeError: if the camera's IP address could not be found
    """

    MAC_address = "44:A9:2C:3C:0A:90"  # DepthAI camera MAC address
    IP_range = "169.254.1.222"  # 192.168.1.0 to 192.168.1.255

    nm = nmap.PortScanner()
    scan = nm.scan(hosts=IP_range, arguments='-sP')['scan']

    for ip, info in scan.items():
        print(f"Searching,... {ip}")
        if info['status']['state'] == 'up': #and info['addresses'].get('mac') == MAC_address:
            print("FOUND!!")
            return ip

    raise RuntimeError("Custom autodiscovery failed to find camera.")

'''
If needed for testing please port this over too thanks!
if __name__ == "__main__":
    rospy.init_node("depthai_camera_connect")
    if connect(dai.Pipeline()):
        rospy.loginfo("Connected to DepthAI device successfully.")

'''