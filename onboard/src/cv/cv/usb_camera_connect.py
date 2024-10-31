#!/usr/bin/env python3

import rospy
import roslaunch
import yaml

import rclpy

import resource_retriever as rr

class USBCameraConnect():
    CAMERA_CONFIG_PATH = f'package://cv/configs/usb_cameras.yaml'

    def connect_all(self):
        # get camera specs
        with open(rr.get_filename(self.CAMERA_CONFIG_PATH, use_protocol=False)) as f:
            cameras = yaml.safe_load(f)

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)

        roslaunch_files = []
        for camera_name in cameras:
            # add cli args for each camera to run upon init
            camera = cameras[camera_name]
            device_path = camera["device_path"]
            topic = camera["topic"]

            cli_args = ["cv", "usb_camera.launch", f"topic:={topic}", f"device_path:={device_path}", "framerate:=10"]
            roslaunch_file = roslaunch.rlutil.resolve_launch_arguments(cli_args)[0]
            roslaunch_files.append((roslaunch_file, cli_args[2:]))

        # actually init
        parent = roslaunch.parent.ROSLaunchParent(uuid, roslaunch_files)
        parent.start()

        while not rospy.is_shutdown():
            rospy.spin()


def main(args=None):
    rclpy.init(args=args)
    camera_connect = USBCameraConnect()

    try:
        rclpy.spin(camera_connect)
    except KeyboardInterrupt:
        pass
    finally:
        camera_connect.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
