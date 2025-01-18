

import os
import subprocess
from pathlib import Path

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

    def __init__(self) -> None:
        """Set up publisher and camera node pipeline."""
        super().__init__('depthai_publish_save_streams')

        self.set_parameters(self)

        num_saved_streams = (int(self.save_rgb_video) + int(self.save_rgb_preview) + int(self.save_left) +
                             int(self.save_right) + int(self.save_disparity))

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

        self.start_publishers(self)

        MIN_NUM_SAVED_STREAMS = 3  # noqa: N806

        if num_saved_streams > MIN_NUM_SAVED_STREAMS:
            msg = 'Cannot save more than 3 streams at once'
            raise ValueError(msg)

        self.check_file_paths(self)

        self.image_tools = ImageTools()
        self.pipeline = dai.Pipeline()
        self.build_pipeline()

        self.run()

    def set_parameters(self) -> None:
        """Set parameters for self."""
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
        self.rgb_video_file_path = os.path.join(self.BASE_PATH, # noqa: PTH118
                                                self.declare_parameter('~rgb_video_file_path', '').value)
        self.rgb_preview_file_path = os.path.join(self.BASE_PATH,  # noqa: PTH118
                                               self.declare_parameter('~rgb_preview_file_path', '').value)
        self.left_file_path = os.path.join(self.BASE_PATH, self.declare_parameter('~left_file_path', '').value) # noqa: PTH118
        self.right_file_path = os.path.join(self.BASE_PATH, self.declare_parameter('~right_file_path', '').value) # noqa: PTH118
        self.disparity_file_path = os.path.join(self.BASE_PATH, # noqa: PTH118
                                                self.declare_parameter('~disparity_file_path', '').value)

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

        self.rgb_video_resolution = (1920, 1080)
        self.rgb_preview_resolution = (416, 416)
        self.left_resolution = (640, 400)
        self.right_resolution = (640, 400)
        self.disparity_resolution = (640, 400)

    def start_publishers(self) -> None:
        """Start publishers."""
         # Set up publishers
        if self.publish_rgb_video:
            self.stream_publisher_rgb_video = self.create_publisher(CompressedImage, self.STREAM_TOPIC_RGB_VIDEO, 10)

        if self.publish_rgb_preview:
            self.stream_publisher_rgb_preview = self.create_publisher(CompressedImage, self.STREAM_TOPIC_RGB_PREVIEW,
                                                                      10)

        if self.publish_left:
            self.stream_publisher_left = self.create_publisher(CompressedImage, self.STREAM_TOPIC_LEFT, 10)

        if self.publish_right:
            self.stream_publisher_right = self.create_publisher(CompressedImage, self.STREAM_TOPIC_RIGHT, 10)

        if self.publish_disparity:
            self.stream_publisher_disparity = self.create_publisher(CompressedImage, self.STREAM_TOPIC_DISPARITY, 10)

        if self.publish_depth:
            self.stream_publisher_depth = self.create_publisher(CompressedImage, self.STREAM_TOPIC_DEPTH, 10)

    def check_file_paths(self) -> None:
        """Check file paths."""
        FRAMERATE_MIN = 1  # noqa: N806
        FRAMERATE_MAX = 60  # noqa: N806

        if FRAMERATE_MIN < 1 or self.framerate > FRAMERATE_MAX:
            msg = f'Framerate {self.framerate} is not in range [1, 60]'
            raise ValueError(msg)

        # Check if the file paths are valid
        if self.save_rgb_video and not utils.check_file_writable(self.rgb_video_file_path):
            msg = f'RGB video file path {self.rgb_video_file_path} is not writable'
            raise ValueError(msg)

        if self.save_rgb_preview and not utils.check_file_writable(self.rgb_preview_file_path):
            msg = f'RGB preview file path {self.rgb_preview_file_path} is not writable'
            raise ValueError(msg)

        if self.save_left and not utils.check_file_writable(self.left_file_path):
            msg = f'Left file path {self.left_file_path} is not writable'
            raise ValueError(msg)

        if self.save_right and not utils.check_file_writable(self.right_file_path):
            msg = f'Right file path {self.right_file_path} is not writable'
            raise ValueError(msg)

        if self.save_disparity and not utils.check_file_writable(self.disparity_file_path):
            msg = f'Disparity file path {self.disparity_file_path} is not writable'
            raise ValueError(msg)

    def build_pipeline(self) -> None:
        """Build the DepthAI. Pipeline, which takes the RGB camera feed and retrieves it using an XLinkOut."""
        self.build_pipeline_cam_LR(self)

        # Setup VideoEncoder and XLinkOut nodes for saving disparity to files
        if self.save_disparity:
            ve_disparity = self.pipeline.create(dai.node.VideoEncoder)
            ve_disparity.setDefaultProfilePreset(self.framerate, dai.VideoEncoderProperties.Profile.H265_MAIN)

            xout_ve_disparity = self.pipeline.create(dai.node.XLinkOut)
            xout_ve_disparity.setStreamName('veDisparity')

        if self.publish_disparity:
            xout_disparity = self.pipeline.create(dai.node.XLinkOut)
            xout_disparity.setStreamName('disparity')
            xout_disparity.input.setBlocking(False)
            xout_disparity.input.setQueueSize(1)

        if self.publish_depth:
            xout_depth = self.pipeline.create(dai.node.XLinkOut)
            xout_depth.setStreamName('depth')
            xout_depth.input.setBlocking(False)
            xout_depth.input.setQueueSize(1)

        # Setup StereoDepth node for disparity/depth
        if self.publish_disparity or self.publish_depth or self.save_disparity:
            stereo = self.pipeline.create(dai.node.StereoDepth)
            stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
            self.stereoMaxDisparity = stereo.initialConfig.getMaxDisparity()

        # Link nodes
        if self.publish_disparity:
            stereo.disparity.link(xout_disparity.input)

        if self.publish_depth:
            stereo.depth.link(xout_depth.input)

        if self.save_disparity:
            stereo.disparity.link(ve_disparity.input)
            ve_disparity.bitstream.link(xout_ve_disparity.input)

    def build_pipeline_cam_LR(self) -> None:
        """Build pipeline for left and right cams, ve left and rought, xout left and right."""
        # Setup MonoCamera node for left camera
        if self.publish_left or self.publish_disparity or self.publish_depth or self.save_left or self.save_disparity:
            cam_left = self.pipeline.create(dai.node.MonoCamera)
            cam_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)  # Resolution: 640 x 400
            cam_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
            cam_left.setFps(self.framerate)

        # Setup VideoEncoder and XLinkOut nodes for saving left camera to files
        if self.save_left:
            ve_left = self.pipeline.create(dai.node.VideoEncoder)
            ve_left.setDefaultProfilePreset(self.framerate, dai.VideoEncoderProperties.Profile.H264_MAIN)

            xout_ve_left = self.pipeline.create(dai.node.XLinkOut)
            xout_ve_left.setStreamName('veLeft')

        # Setup MonoCamera node for right camera
        if self.publish_right or self.publish_disparity or self.publish_depth or self.save_right or self.save_disparity:
            cam_right = self.pipeline.create(dai.node.MonoCamera)
            cam_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)  # Resolution: 640 x 400
            cam_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
            cam_right.setFps(self.framerate)

        # Setup StereoDepth node for disparity/depth
        if self.publish_disparity or self.publish_depth or self.save_disparity:
            stereo = self.pipeline.create(dai.node.StereoDepth)
            stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
            self.stereoMaxDisparity = stereo.initialConfig.getMaxDisparity()

        # Setup VideoEncoder and XLinkOut nodes for saving right camera to files
        if self.save_right:
            ve_right = self.pipeline.create(dai.node.VideoEncoder)
            ve_right.setDefaultProfilePreset(self.framerate, dai.VideoEncoderProperties.Profile.H264_MAIN)

            xout_ve_right = self.pipeline.create(dai.node.XLinkOut)
            xout_ve_right.setStreamName('veRight')

        if self.publish_left:
            xout_left = self.pipeline.create(dai.node.XLinkOut)
            xout_left.setStreamName('left')
            xout_left.input.setBlocking(False)
            xout_left.input.setQueueSize(1)

        if self.publish_right:
            xout_right = self.pipeline.create(dai.node.XLinkOut)
            xout_right.setStreamName('right')
            xout_right.input.setBlocking(False)
            xout_right.input.setQueueSize(1)

        # Link nodes
        if self.publish_disparity or self.publish_depth or self.save_disparity:
            cam_left.out.link(stereo.left)
            cam_right.out.link(stereo.right)

        if self.publish_left:
            cam_left.out.link(xout_left.input)

        if self.publish_right:
            cam_right.out.link(xout_right.input)

        if self.save_left:
            cam_left.out.link(ve_left.input)
            ve_left.bitstream.link(xout_ve_left.input)

        if self.save_right:
            cam_right.out.link(ve_right.input)
            ve_right.bitstream.link(xout_ve_right.input)

    def build_pipeline_cam_RGB(self) -> None:
        """Build pipeline for all things RGB."""
        # Setup XLinkOut nodes for publishing streams to topics
        if self.publish_rgb_video:
            xout_rgb_video = self.pipeline.create(dai.node.XLinkOut)
            xout_rgb_video.setStreamName('rgbVideo')
            xout_rgb_video.input.setBlocking(False)
            xout_rgb_video.input.setQueueSize(1)

        # Setup StereoDepth node for disparity/depth
        if self.publish_disparity or self.publish_depth or self.save_disparity:
            stereo = self.pipeline.create(dai.node.StereoDepth)
            stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
            self.stereoMaxDisparity = stereo.initialConfig.getMaxDisparity()

        # Setup ColorCamera node for RGB video/preview
        if self.publish_rgb_video or self.publish_rgb_preview or self.save_rgb_video or self.save_rgb_preview:
            cam_rgb = self.pipeline.create(dai.node.ColorCamera)
            cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
            cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)

            if self.publish_rgb_video or self.save_rgb_video:
                cam_rgb.setVideoSize(self.rgb_video_resolution)

            if self.publish_rgb_preview or self.save_rgb_preview:
                cam_rgb.setPreviewSize(self.rgb_preview_resolution)

            cam_rgb.setInterleaved(False)
            cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
            cam_rgb.setFps(self.framerate)

        # Setup VideoEncoder and XLinkOut nodes for saving RGB video to files
        if self.save_rgb_video:
            ve_rgb_video = self.pipeline.create(dai.node.VideoEncoder)
            ve_rgb_video.setDefaultProfilePreset(self.framerate, dai.VideoEncoderProperties.Profile.H265_MAIN)

            xout_ve_rgb_video = self.pipeline.create(dai.node.XLinkOut)
            xout_ve_rgb_video.setStreamName('veRgbVideo')

        # Setup VideoEncoder and XLinkOut nodes for saving RGB preview to files
        if self.save_rgb_preview:
            # Must convert rgb preview to NV12 for video encoder
            manip_rgb_preview = self.pipeline.create(dai.node.ImageManip)
            manip_rgb_preview.initialConfig.setFrameType(dai.ImgFrame.Type.NV12)

            ve_rgb_preview = self.pipeline.create(dai.node.VideoEncoder)
            ve_rgb_preview.setDefaultProfilePreset(self.framerate, dai.VideoEncoderProperties.Profile.H265_MAIN)

            xout_ve_rgb_preview = self.pipeline.create(dai.node.XLinkOut)
            xout_ve_rgb_preview.setStreamName('veRgbPreview')

        if self.publish_rgb_preview:
            xout_rgb_preview = self.pipeline.create(dai.node.XLinkOut)
            xout_rgb_preview.setStreamName('rgbPreview')
            xout_rgb_preview.input.setBlocking(False)
            xout_rgb_preview.input.setQueueSize(1)

        # Link nodes
        if self.publish_rgb_video:
            cam_rgb.video.link(xout_rgb_video.input)

        if self.publish_rgb_preview:
            cam_rgb.preview.link(xout_rgb_preview.input)

        if self.save_rgb_video:
            cam_rgb.video.link(ve_rgb_video.input)
            ve_rgb_video.bitstream.link(manip_rgb_preview.input)

        if self.save_rgb_preview:
            cam_rgb.preview.link(manip_rgb_preview.inputImage)
            manip_rgb_preview.out.link(ve_rgb_preview.input)
            ve_rgb_preview.bitstream.link(xout_ve_rgb_preview.input)

    # Publish newest image off queue to topic every few seconds
    def run(self) -> None:
        """Get rgb images from the camera and publish them to STREAM_TOPIC."""
        with depthai_camera_connect.connect(self.pipeline) as device:
            self.setup_output_queue_in_run(self, device)
            self.save_videos_in_run(self, device)

        # Convert encoded video files to playable videos
        h265_convert_options = '-vcodec libx264 -pix_fmt yuv420p' if self.qt_compatible else '-c copy'

        rgb_video_command = (f"""ffmpeg -framerate {self.framerate} -i {self.rgb_video_file_path}.h265
                             {h265_convert_options} {self.rgb_video_file_path}.mp4""")

        rgb_preview_command = (f"""ffmpeg -framerate {self.framerate} -i {self.rgb_preview_file_path}.h265
                               {h265_convert_options} {self.rgb_preview_file_path}.mp4""")

        left_command = (f"""ffmpeg -framerate {self.framerate} -i {self.left_file_path}.h264 -c copy
                        {self.left_file_path}.mp4""")

        right_command = (f"""ffmpeg -framerate {self.framerate} -i {self.right_file_path}.h264 -c copy
                         {self.right_file_path}.mp4""")

        disparity_command = (f"""ffmpeg -framerate {self.framerate} -i {self.disparity_file_path}.h265
                             {h265_convert_options} {self.disparity_file_path}.mp4""")

        if self.convert_to_video:
            self.get_logger().info('Converting encoded video files to playable videos.')
            if self.save_rgb_video:
                subprocess.Popen(rgb_video_command.replace('\n', '').split(' '))  # noqa: S603
            if self.save_rgb_preview:
                subprocess.Popen(rgb_preview_command.replace('\n', '').split(' '))  # noqa: S603
            if self.save_left:
                subprocess.Popen(left_command.replace('\n', '').split(' '))  # noqa: S603
            if self.save_right:
                subprocess.Popen(right_command.replace('\n', '').split(' '))  # noqa: S603
            if self.save_disparity:
                subprocess.Popen(disparity_command.replace('\n', '').split(' '))  # noqa: S603

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

    def setup_output_queue_in_run(self, device: dai.device) -> None:
        """Set up output queue for run."""
        # Output queue, to receive message on the host from the device (you can send the message
            # on the device with XLinkOut)

        if self.publish_rgb_video:
            self.rgbVideoQueue = device.getOutputQueue(name='rgbVideo', maxSize=4, blocking=False)

        if self.publish_rgb_preview:
            self.rgbPreviewQueue = device.getOutputQueue(name='rgbPreview', maxSize=4, blocking=False)

        if self.publish_left:
            self.leftQueue = device.getOutputQueue(name='left', maxSize=4, blocking=False)

        if self.publish_right:
            self.rightQueue = device.getOutputQueue(name='right', maxSize=4, blocking=False)

        if self.publish_disparity:
            self.disparityQueue = device.getOutputQueue(name='disparity', maxSize=4, blocking=False)

        if self.publish_depth:
            self.depthQueue = device.getOutputQueue(name='depth', maxSize=4, blocking=False)


    def save_videos_in_run(self, device: dai.device) -> None:
        """Save videos."""
        if self.save_rgb_video:
                self.veRgbVideoQueue = device.getOutputQueue(name='veRgbVideo', maxSize=4, blocking=False)
                self.rgb_video_file = Path.open(self.rgb_video_file_path + '.h265', 'wb')

        if self.save_rgb_preview:
            self.veRgbPreviewQueue = device.getOutputQueue(name='veRgbPreview', maxSize=4, blocking=False)
            self.rgb_preview_file = Path.open(self.rgb_preview_file_path + '.h265', 'wb')

        if self.save_left:
            self.veLeftQueue = device.getOutputQueue(name='veLeft', maxSize=4, blocking=False)
            self.left_file = Path.open(self.left_file_path + '.h264', 'wb')

        if self.save_right:
            self.veRightQueue = device.getOutputQueue(name='veRight', maxSize=4, blocking=False)
            self.right_file = Path.open(self.right_file_path + '.h264', 'wb')

        if self.save_disparity:
            self.veDisparityQueue = device.getOutputQueue(name='veDisparity', maxSize=4, blocking=False)
            self.disparity_file = Path.open(self.disparity_file_path + '.h265', 'wb')

        self.publish_and_save_timer = self.create_timer(1 / LOOP_RATE, self.publish_and_save)

        # Close files
        if self.save_rgb_video:
            self.rgb_video_file.close()

        if self.save_rgb_preview:
            self.rgb_preview_file.close()

        if self.save_left:
            self.left_file.close()

        if self.save_right:
            self.right_file.close()

        if self.save_disparity:
            self.disparity_file.close()

    def publish_and_save(self) -> None:
        """Publish and saves."""
        # Get messages that came from the queue
        if self.publish_rgb_video:
            raw_img_rgb_video = self.rgbVideoQueue.get()
            img_rgb_video = raw_img_rgb_video.getCvFrame()
            image_msg_rgb_video = self.image_tools.convert_to_ros_compressed_msg(img_rgb_video)
            self.stream_publisher_rgb_video.publish(image_msg_rgb_video)

        if self.publish_rgb_preview:
            raw_img_rgb_preview = self.rgbPreviewQueue.get()
            img_rgb_preview = raw_img_rgb_preview.getCvFrame()
            image_msg_rgb_preview = self.image_tools.convert_to_ros_compressed_msg(img_rgb_preview)
            self.stream_publisher_rgb_preview.publish(image_msg_rgb_preview)

        if self.publish_left:
            raw_img_left = self.leftQueue.get()
            img_left = raw_img_left.getCvFrame()
            image_msg_left = self.image_tools.convert_depth_to_ros_compressed_msg(img_left, 'mono8')
            self.stream_publisher_left.publish(image_msg_left)

        if self.publish_right:
            raw_img_right = self.rightQueue.get()
            img_right = raw_img_right.getCvFrame()
            image_msg_right = self.image_tools.convert_depth_to_ros_compressed_msg(img_right, 'mono8')
            self.stream_publisher_right.publish(image_msg_right)

        if self.publish_disparity:
            # Normalize and apply color map to disparity image
            raw_img_disparity = self.disparityQueue.get()
            img_disparity = raw_img_disparity.getFrame()
            img_disparity = (img_disparity * (255 / self.stereoMaxDisparity)).astype(np.uint8)
            img_disparity = cv2.applyColorMap(img_disparity, cv2.COLORMAP_AUTUMN)
            image_msg_disparity = self.image_tools.convert_to_ros_compressed_msg(img_disparity)
            self.stream_publisher_disparity.publish(image_msg_disparity)

        if self.publish_depth:
            raw_img_depth = self.depthQueue.get()
            img_depth = raw_img_depth.getCvFrame()
            image_msg_depth = self.image_tools.convert_depth_to_ros_compressed_msg(img_depth, 'mono16')
            self.stream_publisher_depth.publish(image_msg_depth)

        # Save messages to files
        while self.save_rgb_video and self.veRgbVideoQueue.has():
            self.veRgbVideoQueue.get().getData().tofile(self.rgb_video_file)

        while self.save_rgb_preview and self.veRgbPreviewQueue.has():
            self.veRgbPreviewQueue.get().getData().tofile(self.rgb_preview_file)

        while self.save_left and self.veLeftQueue.has():
            self.veLeftQueue.get().getData().tofile(self.left_file)

        while self.save_right and self.veRightQueue.has():
            self.veRightQueue.get().getData().tofile(self.right_file)

        while self.save_disparity and self.veDisparityQueue.has():
            self.veDisparityQueue.get().getData().tofile(self.disparity_file)


def main(args:None=None) -> None:
    """Start node."""
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
