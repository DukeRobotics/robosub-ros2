#!/usr/bin/env python3

import os
import subprocess

import cv2
import depthai as dai
import depthai_camera_connect
import numpy as np
import rclpy
import utils
from image_tools import ImageTools
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

LOOP_RATE = 10


class DepthAIStreamsPublisherAndSaver(Node):
    """
    Class to publish the RGB video, preview, mono left, mono right, disparity, and depth streams to respective topics.
    Also saves the RGB video, preview, mono left, mono right, and disparity streams to respective files.
    """

    # Topic names for publishing the streams
    CAMERA = 'front'
    STREAM_TOPIC_RGB_VIDEO = f'/camera/{CAMERA}/rgb/video/compressed'
    STREAM_TOPIC_RGB_PREVIEW = f'/camera/{CAMERA}/rgb/preview/compressed'
    STREAM_TOPIC_LEFT = f'/camera/{CAMERA}/left/compressed'
    STREAM_TOPIC_RIGHT = f'/camera/{CAMERA}/right/compressed'
    STREAM_TOPIC_DISPARITY = f'/camera/{CAMERA}/disparity/compressed'
    STREAM_TOPIC_DEPTH = f'/camera/{CAMERA}/depth/compressed'

    # Base path for saving the streams to files
    BASE_PATH = '/root/dev/robosub-ros/'

    def __init__(self):
        """
        Set up publisher and camera node pipeline.
        """
        super().__init__('depthai_publish_save_streams')

        # Framerate of the streams
        self.framerate = self.declare_parameter('~framerate', 30).value

        # Whether to publish the streams to topics
        self.publish_rgb_video = self.declare_parameter('~rgb_video', False).value
        self.publish_rgb_preview = self.declare_parameter('~rgb_preview', False).value
        self.publish_left = self.declare_parameter('~left', False).value
        self.publish_right = self.declare_parameter('~right', False).value
        self.publish_disparity = self.declare_parameter('~disparity', False).value
        self.publish_depth = self.declare_parameter('~depth', False).value

        # File paths to save the streams to. If param is empty, the stream will not be saved.
        self.rgb_video_file_path = os.path.join(self.BASE_PATH, self.declare_parameter('~rgb_video_file_path', '').value)
        self.rgb_preview_file_path = os.path.join(self.BASE_PATH, self.declare_parameter('~rgb_preview_file_path', '').value)
        self.left_file_path = os.path.join(self.BASE_PATH, self.declare_parameter('~left_file_path', '').value)
        self.right_file_path = os.path.join(self.BASE_PATH, self.declare_parameter('~right_file_path', '').value)
        self.disparity_file_path = os.path.join(self.BASE_PATH, self.declare_parameter('~disparity_file_path', '').value)

        # Whether to convert the saved encoded streams into a video
        self.convert_to_video = self.declare_parameter('~convert_to_video', False).value
        # Whether to convert video to QuickTime compatible format
        self.qt_compatible = self.declare_parameter('~qt_compatible', False).value

        # Whether to save the streams to files
        self.save_rgb_video = self.rgb_video_file_path != ''
        self.save_rgb_preview = self.rgb_preview_file_path != ''
        self.save_left = self.left_file_path != ''
        self.save_right = self.right_file_path != ''
        self.save_disparity = self.disparity_file_path != ''

        num_saved_streams = (int(self.save_rgb_video) + int(self.save_rgb_preview) + int(self.save_left) +
                             int(self.save_right) + int(self.save_disparity))

        self.rgb_video_resolution = (1920, 1080)
        self.rgb_preview_resolution = (416, 416)
        self.left_resolution = (640, 400)
        self.right_resolution = (640, 400)
        self.disparity_resolution = (640, 400)

        # Sum of the number of pixels in each frame of the streams that are being saved
        encoded_pixels_per_frame = 0
        if self.save_rgb_video:
            encoded_pixels_per_frame += self.rgb_video_resolution[0] * self.rgb_video_resolution[1]
        if self.save_rgb_preview:
            encoded_pixels_per_frame += self.rgb_preview_resolution[0] * self.rgb_preview_resolution[1]
        if self.save_left:
            encoded_pixels_per_frame += self.left_resolution[0] * self.left_resolution[1]
        if self.save_right:
            encoded_pixels_per_frame += self.right_resolution[0] * self.right_resolution[1]
        if self.save_disparity:
            encoded_pixels_per_frame += self.disparity_resolution[0] * self.disparity_resolution[1]

        # Check if the framerate goes over the encoding limit
        if encoded_pixels_per_frame > 0:
            max_encoded_pixels_per_second = 3840 * 2160 * 30
            max_framerate = int(max_encoded_pixels_per_second / encoded_pixels_per_frame)
            if self.framerate > max_framerate:
                self.get_logger().warn(f'Framerate {self.framerate} goes over the encoding limit. '
                                       f'Using maximum possible framerate: {max_framerate}')
                self.framerate = max_framerate

        if num_saved_streams > 3:
            raise ValueError('Cannot save more than 3 streams at once')

        if self.framerate < 1 or self.framerate > 60:
            raise ValueError(f'Framerate {self.framerate} is not in range [1, 60]')

        # Check if the file paths are valid
        if self.save_rgb_video and not utils.check_file_writable(self.rgb_video_file_path):
            raise ValueError(f'RGB video file path {self.rgb_video_file_path} is not writable')

        if self.save_rgb_preview and not utils.check_file_writable(self.rgb_preview_file_path):
            raise ValueError(f'RGB preview file path {self.rgb_preview_file_path} is not writable')

        if self.save_left and not utils.check_file_writable(self.left_file_path):
            raise ValueError(f'Left file path {self.left_file_path} is not writable')

        if self.save_right and not utils.check_file_writable(self.right_file_path):
            raise ValueError(f'Right file path {self.right_file_path} is not writable')

        if self.save_disparity and not utils.check_file_writable(self.disparity_file_path):
            raise ValueError(f'Disparity file path {self.disparity_file_path} is not writable')

        # Set up publishers
        if self.publish_rgb_video:
            self.stream_publisher_rgb_video = self.create_publisher(CompressedImage, self.STREAM_TOPIC_RGB_VIDEO, 10)

        if self.publish_rgb_preview:
            self.stream_publisher_rgb_preview = self.create_publisher(CompressedImage, self.STREAM_TOPIC_RGB_PREVIEW, 10)

        if self.publish_left:
            self.stream_publisher_left = self.create_publisher(CompressedImage, self.STREAM_TOPIC_LEFT, 10)

        if self.publish_right:
            self.stream_publisher_right = self.create_publisher(CompressedImage, self.STREAM_TOPIC_RIGHT, 10)

        if self.publish_disparity:
            self.stream_publisher_disparity = self.create_publisher(CompressedImage, self.STREAM_TOPIC_DISPARITY, 10)

        if self.publish_depth:
            self.stream_publisher_depth = self.create_publisher(CompressedImage, self.STREAM_TOPIC_DEPTH, 10)

        self.image_tools = ImageTools()
        self.pipeline = dai.Pipeline()
        self.build_pipeline()

        self.run()

    def build_pipeline(self):
        """
        Build the DepthAI.Pipeline, which takes the RGB camera feed and retrieves it using an XLinkOut.
        """
        # Setup ColorCamera node for RGB video/preview
        if self.publish_rgb_video or self.publish_rgb_preview or self.save_rgb_video or self.save_rgb_preview:
            camRgb = self.pipeline.create(dai.node.ColorCamera)
            camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
            camRgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)

            if self.publish_rgb_video or self.save_rgb_video:
                camRgb.setVideoSize(self.rgb_video_resolution)

            if self.publish_rgb_preview or self.save_rgb_preview:
                camRgb.setPreviewSize(self.rgb_preview_resolution)

            camRgb.setInterleaved(False)
            camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
            camRgb.setFps(self.framerate)

        # Setup VideoEncoder and XLinkOut nodes for saving RGB video to files
        if self.save_rgb_video:
            veRgbVideo = self.pipeline.create(dai.node.VideoEncoder)
            veRgbVideo.setDefaultProfilePreset(self.framerate, dai.VideoEncoderProperties.Profile.H265_MAIN)

            xoutVeRgbVideo = self.pipeline.create(dai.node.XLinkOut)
            xoutVeRgbVideo.setStreamName('veRgbVideo')

        # Setup VideoEncoder and XLinkOut nodes for saving RGB preview to files
        if self.save_rgb_preview:
            # Must convert rgb preview to NV12 for video encoder
            manipRgbPreview = self.pipeline.create(dai.node.ImageManip)
            manipRgbPreview.initialConfig.setFrameType(dai.ImgFrame.Type.NV12)

            veRgbPreview = self.pipeline.create(dai.node.VideoEncoder)
            veRgbPreview.setDefaultProfilePreset(self.framerate, dai.VideoEncoderProperties.Profile.H265_MAIN)

            xoutVeRgbPreview = self.pipeline.create(dai.node.XLinkOut)
            xoutVeRgbPreview.setStreamName('veRgbPreview')

        # Setup MonoCamera node for left camera
        if self.publish_left or self.publish_disparity or self.publish_depth or self.save_left or self.save_disparity:
            camLeft = self.pipeline.create(dai.node.MonoCamera)
            camLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)  # Resolution: 640 x 400
            camLeft.setBoardSocket(dai.CameraBoardSocket.CAM_B)
            camLeft.setFps(self.framerate)

        # Setup VideoEncoder and XLinkOut nodes for saving left camera to files
        if self.save_left:
            veLeft = self.pipeline.create(dai.node.VideoEncoder)
            veLeft.setDefaultProfilePreset(self.framerate, dai.VideoEncoderProperties.Profile.H264_MAIN)

            xoutVeLeft = self.pipeline.create(dai.node.XLinkOut)
            xoutVeLeft.setStreamName('veLeft')

        # Setup MonoCamera node for right camera
        if self.publish_right or self.publish_disparity or self.publish_depth or self.save_right or self.save_disparity:
            camRight = self.pipeline.create(dai.node.MonoCamera)
            camRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)  # Resolution: 640 x 400
            camRight.setBoardSocket(dai.CameraBoardSocket.CAM_C)
            camRight.setFps(self.framerate)

        # Setup VideoEncoder and XLinkOut nodes for saving right camera to files
        if self.save_right:
            veRight = self.pipeline.create(dai.node.VideoEncoder)
            veRight.setDefaultProfilePreset(self.framerate, dai.VideoEncoderProperties.Profile.H264_MAIN)

            xoutVeRight = self.pipeline.create(dai.node.XLinkOut)
            xoutVeRight.setStreamName('veRight')

        # Setup StereoDepth node for disparity/depth
        if self.publish_disparity or self.publish_depth or self.save_disparity:
            stereo = self.pipeline.create(dai.node.StereoDepth)
            stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
            self.stereoMaxDisparity = stereo.initialConfig.getMaxDisparity()

        # Setup VideoEncoder and XLinkOut nodes for saving disparity to files
        if self.save_disparity:
            veDisparity = self.pipeline.create(dai.node.VideoEncoder)
            veDisparity.setDefaultProfilePreset(self.framerate, dai.VideoEncoderProperties.Profile.H265_MAIN)

            xoutVeDisparity = self.pipeline.create(dai.node.XLinkOut)
            xoutVeDisparity.setStreamName('veDisparity')

        # Setup XLinkOut nodes for publishing streams to topics
        if self.publish_rgb_video:
            xoutRgbVideo = self.pipeline.create(dai.node.XLinkOut)
            xoutRgbVideo.setStreamName('rgbVideo')
            xoutRgbVideo.input.setBlocking(False)
            xoutRgbVideo.input.setQueueSize(1)

        if self.publish_rgb_preview:
            xoutRgbPreview = self.pipeline.create(dai.node.XLinkOut)
            xoutRgbPreview.setStreamName('rgbPreview')
            xoutRgbPreview.input.setBlocking(False)
            xoutRgbPreview.input.setQueueSize(1)

        if self.publish_left:
            xoutLeft = self.pipeline.create(dai.node.XLinkOut)
            xoutLeft.setStreamName('left')
            xoutLeft.input.setBlocking(False)
            xoutLeft.input.setQueueSize(1)

        if self.publish_right:
            xoutRight = self.pipeline.create(dai.node.XLinkOut)
            xoutRight.setStreamName('right')
            xoutRight.input.setBlocking(False)
            xoutRight.input.setQueueSize(1)

        if self.publish_disparity:
            xoutDisparity = self.pipeline.create(dai.node.XLinkOut)
            xoutDisparity.setStreamName('disparity')
            xoutDisparity.input.setBlocking(False)
            xoutDisparity.input.setQueueSize(1)

        if self.publish_depth:
            xoutDepth = self.pipeline.create(dai.node.XLinkOut)
            xoutDepth.setStreamName('depth')
            xoutDepth.input.setBlocking(False)
            xoutDepth.input.setQueueSize(1)

        # Link nodes
        if self.publish_disparity or self.publish_depth or self.save_disparity:
            camLeft.out.link(stereo.left)
            camRight.out.link(stereo.right)

        if self.publish_rgb_video:
            camRgb.video.link(xoutRgbVideo.input)

        if self.publish_rgb_preview:
            camRgb.preview.link(xoutRgbPreview.input)

        if self.publish_left:
            camLeft.out.link(xoutLeft.input)

        if self.publish_right:
            camRight.out.link(xoutRight.input)

        if self.publish_disparity:
            stereo.disparity.link(xoutDisparity.input)

        if self.publish_depth:
            stereo.depth.link(xoutDepth.input)

        if self.save_rgb_video:
            camRgb.video.link(veRgbVideo.input)
            veRgbVideo.bitstream.link(xoutVeRgbVideo.input)

        if self.save_rgb_preview:
            camRgb.preview.link(manipRgbPreview.inputImage)
            manipRgbPreview.out.link(veRgbPreview.input)
            veRgbPreview.bitstream.link(xoutVeRgbPreview.input)

        if self.save_left:
            camLeft.out.link(veLeft.input)
            veLeft.bitstream.link(xoutVeLeft.input)

        if self.save_right:
            camRight.out.link(veRight.input)
            veRight.bitstream.link(xoutVeRight.input)

        if self.save_disparity:
            stereo.disparity.link(veDisparity.input)
            veDisparity.bitstream.link(xoutVeDisparity.input)

    # Publish newest image off queue to topic every few seconds
    def run(self):
        """
        Get rgb images from the camera and publish them to STREAM_TOPIC.
        """
        with depthai_camera_connect.connect(self.pipeline) as device:

            # Output queue, to receive message on the host from the device (you can send the message
            # on the device with XLinkOut)

            if self.publish_rgb_video:
                rgbVideoQueue = device.getOutputQueue(name='rgbVideo', maxSize=4, blocking=False)

            if self.publish_rgb_preview:
                rgbPreviewQueue = device.getOutputQueue(name='rgbPreview', maxSize=4, blocking=False)

            if self.publish_left:
                leftQueue = device.getOutputQueue(name='left', maxSize=4, blocking=False)

            if self.publish_right:
                rightQueue = device.getOutputQueue(name='right', maxSize=4, blocking=False)

            if self.publish_disparity:
                disparityQueue = device.getOutputQueue(name='disparity', maxSize=4, blocking=False)

            if self.publish_depth:
                depthQueue = device.getOutputQueue(name='depth', maxSize=4, blocking=False)

            if self.save_rgb_video:
                veRgbVideoQueue = device.getOutputQueue(name='veRgbVideo', maxSize=4, blocking=False)
                rgb_video_file = open(self.rgb_video_file_path + '.h265', 'wb')

            if self.save_rgb_preview:
                veRgbPreviewQueue = device.getOutputQueue(name='veRgbPreview', maxSize=4, blocking=False)
                rgb_preview_file = open(self.rgb_preview_file_path + '.h265', 'wb')

            if self.save_left:
                veLeftQueue = device.getOutputQueue(name='veLeft', maxSize=4, blocking=False)
                left_file = open(self.left_file_path + '.h264', 'wb')

            if self.save_right:
                veRightQueue = device.getOutputQueue(name='veRight', maxSize=4, blocking=False)
                right_file = open(self.right_file_path + '.h264', 'wb')

            if self.save_disparity:
                veDisparityQueue = device.getOutputQueue(name='veDisparity', maxSize=4, blocking=False)
                disparity_file = open(self.disparity_file_path + '.h265', 'wb')

            self.publish_and_save_timer = self.create_timer(1 / LOOP_RATE, self.publish_and_save)

            # Close files
            if self.save_rgb_video:
                rgb_video_file.close()

            if self.save_rgb_preview:
                rgb_preview_file.close()

            if self.save_left:
                left_file.close()

            if self.save_right:
                right_file.close()

            if self.save_disparity:
                disparity_file.close()

        # Convert encoded video files to playable videos
        h265_convert_options = '-vcodec libx264 -pix_fmt yuv420p' if self.qt_compatible else '-c copy'

        rgb_video_command = (f'ffmpeg -framerate {self.framerate} -i {self.rgb_video_file_path}.h265 ' +
                             f'{h265_convert_options} {self.rgb_video_file_path}.mp4')

        rgb_preview_command = (f'ffmpeg -framerate {self.framerate} -i {self.rgb_preview_file_path}.h265 ' +
                               f'{h265_convert_options} {self.rgb_preview_file_path}.mp4')

        left_command = (f'ffmpeg -framerate {self.framerate} -i {self.left_file_path}.h264 -c copy ' +
                        f'{self.left_file_path}.mp4')

        right_command = (f'ffmpeg -framerate {self.framerate} -i {self.right_file_path}.h264 -c copy ' +
                         f'{self.right_file_path}.mp4')

        disparity_command = (f'ffmpeg -framerate {self.framerate} -i {self.disparity_file_path}.h265 ' +
                             f'{h265_convert_options} {self.disparity_file_path}.mp4')

        if self.convert_to_video:
            self.get_logger().info('Converting encoded video files to playable videos.')
            if self.save_rgb_video:
                subprocess.Popen(rgb_video_command.split(' '))
            if self.save_rgb_preview:
                subprocess.Popen(rgb_preview_command.split(' '))
            if self.save_left:
                subprocess.Popen(left_command.split(' '))
            if self.save_right:
                subprocess.Popen(right_command.split(' '))
            if self.save_disparity:
                subprocess.Popen(disparity_command.split(' '))

        self.get_logger().info('To convert the encoded video files to a playable videos, run the following commands:')
        if self.save_rgb_video:
            self.get_logger().info(rgb_video_command)
        if self.save_rgb_preview:
            self.get_logger().info(rgb_preview_command)
        if self.save_left:
            self.get_logger().info(left_command)
        if self.save_right:
            self.get_logger().info(right_command)
        if self.save_disparity:
            self.get_logger().info(disparity_command)

    def publish_and_save(self):
        # Get messages that came from the queue
        if self.publish_rgb_video:
            raw_img_rgb_video = rgbVideoQueue.get()
            img_rgb_video = raw_img_rgb_video.getCvFrame()
            image_msg_rgb_video = self.image_tools.convert_to_ros_compressed_msg(img_rgb_video)
            self.stream_publisher_rgb_video.publish(image_msg_rgb_video)

        if self.publish_rgb_preview:
            raw_img_rgb_preview = rgbPreviewQueue.get()
            img_rgb_preview = raw_img_rgb_preview.getCvFrame()
            image_msg_rgb_preview = self.image_tools.convert_to_ros_compressed_msg(img_rgb_preview)
            self.stream_publisher_rgb_preview.publish(image_msg_rgb_preview)

        if self.publish_left:
            raw_img_left = leftQueue.get()
            img_left = raw_img_left.getCvFrame()
            image_msg_left = self.image_tools.convert_depth_to_ros_compressed_msg(img_left, 'mono8')
            self.stream_publisher_left.publish(image_msg_left)

        if self.publish_right:
            raw_img_right = rightQueue.get()
            img_right = raw_img_right.getCvFrame()
            image_msg_right = self.image_tools.convert_depth_to_ros_compressed_msg(img_right, 'mono8')
            self.stream_publisher_right.publish(image_msg_right)

        if self.publish_disparity:
            # Normalize and apply color map to disparity image
            raw_img_disparity = disparityQueue.get()
            img_disparity = raw_img_disparity.getFrame()
            img_disparity = (img_disparity * (255 / self.stereoMaxDisparity)).astype(np.uint8)
            img_disparity = cv2.applyColorMap(img_disparity, cv2.COLORMAP_AUTUMN)
            image_msg_disparity = self.image_tools.convert_to_ros_compressed_msg(img_disparity)
            self.stream_publisher_disparity.publish(image_msg_disparity)

        if self.publish_depth:
            raw_img_depth = depthQueue.get()
            img_depth = raw_img_depth.getCvFrame()
            image_msg_depth = self.image_tools.convert_depth_to_ros_compressed_msg(img_depth, 'mono16')
            self.stream_publisher_depth.publish(image_msg_depth)

        # Save messages to files
        while self.save_rgb_video and veRgbVideoQueue.has():
            veRgbVideoQueue.get().getData().tofile(rgb_video_file)

        while self.save_rgb_preview and veRgbPreviewQueue.has():
            veRgbPreviewQueue.get().getData().tofile(rgb_preview_file)

        while self.save_left and veLeftQueue.has():
            veLeftQueue.get().getData().tofile(left_file)

        while self.save_right and veRightQueue.has():
            veRightQueue.get().getData().tofile(right_file)

        while self.save_disparity and veDisparityQueue.has():
            veDisparityQueue.get().getData().tofile(disparity_file)


def main(args=None):
    rclpy.init(args=args)
    depthai_streams_publisher_and_saver = DepthAIStreamsPublisherAndSaver()

    try:
        rclpy.spin(depthai_streams_publisher_and_saver)
    except KeyboardInterrupt:
        pass
    finally:
        depthai_streams_publisher_and_saver.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()