from pathlib import Path
from typing import Any, ClassVar

import depthai as dai
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

from cv import depthai_camera_connect, utils
from cv.image_tools import ImageTools


class DepthAIStreamsPublisherAndSaver(Node):
    """
    Class to publish the RGB video, preview, mono left, mono right, disparity, and depth streams to respective topics.

    Also saves the RGB video, preview, mono left, mono right, and disparity streams to respective files.
    """

    # Topic names for publishing the streams
    STREAM_TOPIC_RGB_VIDEO: ClassVar[str] = '/camera/{camera}/rgb/video/compressed'
    STREAM_TOPIC_RGB_PREVIEW: ClassVar[str] = '/camera/{camera}/rgb/preview/compressed'
    STREAM_TOPIC_LEFT: ClassVar[str] = '/camera/{camera}/left/compressed'
    STREAM_TOPIC_RIGHT: ClassVar[str] = '/camera/{camera}/right/compressed'
    STREAM_TOPIC_DISPARITY: ClassVar[str] = '/camera/{camera}/disparity/compressed'
    STREAM_TOPIC_DEPTH: ClassVar[str] = '/camera/{camera}/depth/compressed'

    RGB_VIDEO_RESOLUTION: ClassVar[tuple[int, int]] = (1920, 1080)
    RGB_PREVIEW_RESOLUTION: ClassVar[tuple[int, int]] = (416, 416)
    LEFT_RESOLUTION: ClassVar[tuple[int, int]] = (640, 400)
    RIGHT_RESOLUTION: ClassVar[tuple[int, int]] = (640, 400)
    DISPARITY_RESOLUTION: ClassVar[tuple[int, int]] = (640, 400)

    FRAMERATE_MIN: ClassVar[int] = 1
    FRAMERATE_MAX: ClassVar[int] = 60

    # Maximum number of pixels that can be encoded per second in H.264/H.265 format, as provided here:
    # https://docs.luxonis.com/software/depthai-components/nodes/video_encoder
    MAX_ENCODED_PIXELS_PER_SECOND: ClassVar[int] = 3840 * 2160 * 30  # 4K resolution at 30 fps

    # Base path for saving the streams to files
    BASE_PATH: ClassVar[Path] = Path('/home/ubuntu/robosub-ros2/')

    PARAMETERS: ClassVar[list[tuple[str, Any]]] = [
        ('camera', 'front'),
        ('framerate', 30),
        ('rgb_video', False),
        ('rgb_video_file_path', ''),
        ('rgb_preview', False),
        ('rgb_preview_file_path', ''),
        ('left', False),
        ('left_file_path', ''),
        ('right', False),
        ('right_file_path', ''),
        ('disparity', False),
        ('disparity_file_path', ''),
        ('depth', False),
    ]

    MAX_NUM_SAVED_STREAMS: ClassVar[int] = 3

    RGB_VIDEO_SUFFIX: ClassVar[str] = '.h265'
    RGB_PREVIEW_SUFFIX: ClassVar[str] = '.h265'
    LEFT_SUFFIX: ClassVar[str] = '.h264'
    RIGHT_SUFFIX: ClassVar[str] = '.h264'
    DISPARITY_SUFFIX: ClassVar[str] = '.h265'

    PLAYABLE_VIDEO_SUFFIX: ClassVar[str] = '.mp4'

    H265_CONVERT_OPTIONS: ClassVar[str] = '-vcodec libx264 -pix_fmt yuv420p'
    H264_CONVERT_OPTIONS: ClassVar[str] = '-c copy'

    PLAYABLE_VIDEO_COMMAND_TEMPLATE: ClassVar[str] = \
        'ffmpeg -framerate {framerate} -i {input_file} {options} {output_file}'

    def __init__(self) -> None:
        """Set up publisher and camera node pipeline."""
        super().__init__('depthai_publish_save_streams')

        self.get_ros_param_values()
        self.check_framerate()
        self.determine_if_streams_should_be_saved()
        self.check_num_saved_streams()
        self.compute_stream_save_paths()
        self.check_encoded_pixels_per_frame()
        self.create_publishers()
        self.check_file_paths()
        self.build_pipeline()

        self.image_tools = ImageTools()

        self.run()

    def get_ros_param_values(self) -> None:
        """Get values of all ROS parameters."""
        # Declare all parameters
        self.declare_parameters('', self.PARAMETERS)

        # Get parameter values
        self.camera = self.get_parameter('camera').get_parameter_value().string_value
        self.framerate = self.get_parameter('framerate').get_parameter_value().integer_value

        self.publish_rgb_video = self.get_parameter('rgb_video').get_parameter_value().bool_value
        self.publish_rgb_preview = self.get_parameter('rgb_preview').get_parameter_value().bool_value
        self.publish_left = self.get_parameter('left').get_parameter_value().bool_value
        self.publish_right = self.get_parameter('right').get_parameter_value().bool_value
        self.publish_disparity = self.get_parameter('disparity').get_parameter_value().bool_value
        self.publish_depth = self.get_parameter('depth').get_parameter_value().bool_value

        self.rgb_video_file_path_param = self.get_parameter('rgb_video_file_path').get_parameter_value().string_value
        self.rgb_preview_file_path_param = \
            self.get_parameter('rgb_preview_file_path').get_parameter_value().string_value
        self.left_file_path_param = self.get_parameter('left_file_path').get_parameter_value().string_value
        self.right_file_path_param = self.get_parameter('right_file_path').get_parameter_value().string_value
        self.disparity_file_path_param = self.get_parameter('disparity_file_path').get_parameter_value().string_value

    def check_framerate(self) -> None:
        """Check if the framerate is in the valid range. If not, raise an error."""
        if self.framerate < self.FRAMERATE_MIN or self.framerate > self.FRAMERATE_MAX:
            msg = f'Framerate {self.framerate} is not in range [{self.FRAMERATE_MIN}, {self.FRAMERATE_MAX}].'
            raise ValueError(msg)

    def determine_if_streams_should_be_saved(self) -> None:
        """Determine if streams should be saved."""
        # If the file paths are empty, the streams will not be saved
        self.save_rgb_video = self.rgb_video_file_path_param != ''
        self.save_rgb_preview = self.rgb_preview_file_path_param != ''
        self.save_left = self.left_file_path_param != ''
        self.save_right = self.right_file_path_param != ''
        self.save_disparity = self.disparity_file_path_param != ''

    def check_num_saved_streams(self) -> None:
        """Check if the number of saved streams is less than the maximum. If not, raise an error."""
        num_saved_streams =  (int(self.save_rgb_video) + int(self.save_rgb_preview) + int(self.save_left) +
                              int(self.save_right) + int(self.save_disparity))

        if num_saved_streams > self.MAX_NUM_SAVED_STREAMS:
            error_msg = f'Cannot save more than {self.MAX_NUM_SAVED_STREAMS} streams at once.'
            raise ValueError(error_msg)

    def compute_stream_save_paths(self) -> None:
        """Compute file paths to save the streams to."""
        # For the streams that should be saved, compute the file paths to save the streams to.
        if self.save_rgb_video:
            self.rgb_video_file_path = (self.BASE_PATH /
                                        self.rgb_video_file_path_param).with_suffix(self.RGB_VIDEO_SUFFIX)

        if self.save_rgb_preview:
            self.rgb_preview_file_path = (self.BASE_PATH /
                                        self.rgb_preview_file_path_param).with_suffix(self.RGB_PREVIEW_SUFFIX)

        if self.save_left:
            self.left_file_path = (self.BASE_PATH / self.left_file_path_param).with_suffix(self.LEFT_SUFFIX)

        if self.save_right:
            self.right_file_path = (self.BASE_PATH / self.right_file_path_param).with_suffix(self.RIGHT_SUFFIX)

        if self.save_disparity:
            self.disparity_file_path = (self.BASE_PATH /
                                        self.disparity_file_path_param).with_suffix(self.DISPARITY_SUFFIX)

    def check_encoded_pixels_per_frame(self) -> None:
        """
        Make sure the framerate does not go over the encoding limit.

        Compute the number of pixels that are being encoded in each frame. If it is greater than the maximum number of
        pixels that can be encoded per frame, lower the framerate to the maximum possible framerate and warn the user.
        """
        encoded_pixels_per_frame = 0
        if self.save_rgb_video:
            encoded_pixels_per_frame += self.RGB_VIDEO_RESOLUTION[0] * self.RGB_VIDEO_RESOLUTION[1]
        if self.save_rgb_preview:
            encoded_pixels_per_frame += self.RGB_PREVIEW_RESOLUTION[0] * self.RGB_PREVIEW_RESOLUTION[1]
        if self.save_left:
            encoded_pixels_per_frame += self.LEFT_RESOLUTION[0] * self.LEFT_RESOLUTION[1]
        if self.save_right:
            encoded_pixels_per_frame += self.RIGHT_RESOLUTION[0] * self.RIGHT_RESOLUTION[1]
        if self.save_disparity:
            encoded_pixels_per_frame += self.DISPARITY_RESOLUTION[0] * self.DISPARITY_RESOLUTION[1]

        if encoded_pixels_per_frame > 0:
            max_framerate = int(self.MAX_ENCODED_PIXELS_PER_SECOND / encoded_pixels_per_frame)
            if self.framerate > max_framerate:
                self.get_logger().warn(f'Framerate {self.framerate} goes over the encoding limit. '
                                       f'Using maximum possible framerate: {max_framerate}')
                self.framerate = max_framerate

    def create_publishers(self) -> None:
        """Create publishers."""
        # Set up publishers
        if self.publish_rgb_video:
            self.stream_publisher_rgb_video = self.create_publisher(
                CompressedImage, self.STREAM_TOPIC_RGB_VIDEO.format(camera=self.camera), 10)

        if self.publish_rgb_preview:
            self.stream_publisher_rgb_preview = self.create_publisher(
                CompressedImage, self.STREAM_TOPIC_RGB_PREVIEW.format(camera=self.camera), 10)

        if self.publish_left:
            self.stream_publisher_left = self.create_publisher(
                CompressedImage, self.STREAM_TOPIC_LEFT.format(camera=self.camera), 10)

        if self.publish_right:
            self.stream_publisher_right = self.create_publisher(
                CompressedImage, self.STREAM_TOPIC_RIGHT.format(camera=self.camera), 10)

        if self.publish_disparity:
            self.stream_publisher_disparity = self.create_publisher(
                CompressedImage, self.STREAM_TOPIC_DISPARITY.format(camera=self.camera), 10)

        if self.publish_depth:
            self.stream_publisher_depth = self.create_publisher(
                CompressedImage, self.STREAM_TOPIC_DEPTH.format(camera=self.camera), 10)

    def check_file_paths(self) -> None:
        """Check if file paths for all saved streams exist and are writable. If not, raise an error."""
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
        """Build the DepthAI pipeline, which takes the RGB camera feed and retrieves it using an XLinkOut."""
        self.pipeline = dai.Pipeline()
        self.build_pipeline_cam_RGB()
        self.build_pipeline_cam_LR()

    def build_pipeline_cam_LR(self) -> None:  # noqa: PLR0915
        """Build pipeline for left and right cams, ve left and right, xout left and right."""
        # Setup MonoCamera node for left camera
        if self.publish_left or self.publish_disparity or self.publish_depth or self.save_left or self.save_disparity:
            cam_left = self.pipeline.create(dai.node.MonoCamera)
            cam_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)  # Resolution: 640 x 400
            cam_left.setBoardSocket(dai.CameraBoardSocket.CAM_B)
            cam_left.setFps(self.framerate)

        # Setup MonoCamera node for right camera
        if self.publish_right or self.publish_disparity or self.publish_depth or self.save_right or self.save_disparity:
            cam_right = self.pipeline.create(dai.node.MonoCamera)
            cam_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)  # Resolution: 640 x 400
            cam_right.setBoardSocket(dai.CameraBoardSocket.CAM_C)
            cam_right.setFps(self.framerate)

        # Setup StereoDepth node for disparity/depth
        if self.publish_depth or self.publish_disparity or self.save_disparity:
            stereo = self.pipeline.create(dai.node.StereoDepth)
            stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)

            # Apply a colormap to the disparity output so it can be visualized
            if self.publish_disparity or self.save_disparity:
                manip_disparity = self.pipeline.create(dai.node.ImageManip)
                manip_disparity.initialConfig.setResize(*self.DISPARITY_RESOLUTION)
                manip_disparity.initialConfig.setFrameType(dai.ImgFrame.Type.NV12)
                manip_disparity.initialConfig.setColormap(dai.Colormap.STEREO_TURBO,
                                                          stereo.initialConfig.getMaxDisparity())
                manip_disparity.setMaxOutputFrameSize(int(self.DISPARITY_RESOLUTION[0] *
                                                          self.DISPARITY_RESOLUTION[1] * 1.5))

        # Setup VideoEncoder and XLinkOut nodes
        if self.publish_left:
            xout_left = self.pipeline.create(dai.node.XLinkOut)
            xout_left.setStreamName('left')
            xout_left.input.setBlocking(False)
            xout_left.input.setQueueSize(1)

        if self.save_left:
            ve_left = self.pipeline.create(dai.node.VideoEncoder)
            ve_left.setDefaultProfilePreset(self.framerate, dai.VideoEncoderProperties.Profile.H264_MAIN)

            xout_ve_left = self.pipeline.create(dai.node.XLinkOut)
            xout_ve_left.setStreamName('veLeft')

        if self.publish_right:
            xout_right = self.pipeline.create(dai.node.XLinkOut)
            xout_right.setStreamName('right')
            xout_right.input.setBlocking(False)
            xout_right.input.setQueueSize(1)

        if self.save_right:
            ve_right = self.pipeline.create(dai.node.VideoEncoder)
            ve_right.setDefaultProfilePreset(self.framerate, dai.VideoEncoderProperties.Profile.H264_MAIN)

            xout_ve_right = self.pipeline.create(dai.node.XLinkOut)
            xout_ve_right.setStreamName('veRight')

        if self.publish_depth:
            xout_depth = self.pipeline.create(dai.node.XLinkOut)
            xout_depth.setStreamName('depth')
            xout_depth.input.setBlocking(False)
            xout_depth.input.setQueueSize(1)

        if self.publish_disparity:
            xout_disparity = self.pipeline.create(dai.node.XLinkOut)
            xout_disparity.setStreamName('disparity')
            xout_disparity.input.setBlocking(False)
            xout_disparity.input.setQueueSize(1)

        if self.save_disparity:
            ve_disparity = self.pipeline.create(dai.node.VideoEncoder)
            ve_disparity.setDefaultProfilePreset(self.framerate, dai.VideoEncoderProperties.Profile.H265_MAIN)

            xout_ve_disparity = self.pipeline.create(dai.node.XLinkOut)
            xout_ve_disparity.setStreamName('veDisparity')

        # Link nodes
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

        if self.publish_depth or self.publish_disparity or self.save_disparity:
            cam_left.out.link(stereo.left)
            cam_right.out.link(stereo.right)

        if self.publish_depth:
            stereo.depth.link(xout_depth.input)

        if self.publish_disparity or self.save_disparity:
            stereo.disparity.link(manip_disparity.inputImage)

            if self.publish_disparity:
                manip_disparity.out.link(xout_disparity.input)

            if self.save_disparity:
                manip_disparity.out.link(ve_disparity.input)
                ve_disparity.bitstream.link(xout_ve_disparity.input)

    def build_pipeline_cam_RGB(self) -> None:
        """Build pipeline for all things RGB."""
        # Setup ColorCamera node for RGB video/preview
        if self.publish_rgb_video or self.publish_rgb_preview or self.save_rgb_video or self.save_rgb_preview:
            cam_rgb = self.pipeline.create(dai.node.ColorCamera)
            cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
            cam_rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)

            if self.publish_rgb_video or self.save_rgb_video:
                cam_rgb.setVideoSize(self.RGB_VIDEO_RESOLUTION)

            if self.publish_rgb_preview or self.save_rgb_preview:
                cam_rgb.setPreviewSize(self.RGB_PREVIEW_RESOLUTION)

            cam_rgb.setInterleaved(False)
            cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
            cam_rgb.setFps(self.framerate)

        # Setup VideoEncoder and XLinkOut nodes
        if self.publish_rgb_video:
            xout_rgb_video = self.pipeline.create(dai.node.XLinkOut)
            xout_rgb_video.setStreamName('rgbVideo')
            xout_rgb_video.input.setBlocking(False)
            xout_rgb_video.input.setQueueSize(1)

        if self.save_rgb_video:
            ve_rgb_video = self.pipeline.create(dai.node.VideoEncoder)
            ve_rgb_video.setDefaultProfilePreset(self.framerate, dai.VideoEncoderProperties.Profile.H265_MAIN)

            xout_ve_rgb_video = self.pipeline.create(dai.node.XLinkOut)
            xout_ve_rgb_video.setStreamName('veRgbVideo')

        if self.publish_rgb_preview:
            xout_rgb_preview = self.pipeline.create(dai.node.XLinkOut)
            xout_rgb_preview.setStreamName('rgbPreview')
            xout_rgb_preview.input.setBlocking(False)
            xout_rgb_preview.input.setQueueSize(1)

        if self.save_rgb_preview:
            # Must convert rgb preview to NV12 for video encoder
            manip_rgb_preview = self.pipeline.create(dai.node.ImageManip)
            manip_rgb_preview.initialConfig.setFrameType(dai.ImgFrame.Type.NV12)

            ve_rgb_preview = self.pipeline.create(dai.node.VideoEncoder)
            ve_rgb_preview.setDefaultProfilePreset(self.framerate, dai.VideoEncoderProperties.Profile.H265_MAIN)

            xout_ve_rgb_preview = self.pipeline.create(dai.node.XLinkOut)
            xout_ve_rgb_preview.setStreamName('veRgbPreview')

        # Link nodes
        if self.publish_rgb_video:
            cam_rgb.video.link(xout_rgb_video.input)

        if self.publish_rgb_preview:
            cam_rgb.preview.link(xout_rgb_preview.input)

        if self.save_rgb_video:
            cam_rgb.video.link(ve_rgb_video.input)
            ve_rgb_video.bitstream.link(xout_ve_rgb_video.input)

        if self.save_rgb_preview:
            cam_rgb.preview.link(manip_rgb_preview.inputImage)
            manip_rgb_preview.out.link(ve_rgb_preview.input)
            ve_rgb_preview.bitstream.link(xout_ve_rgb_preview.input)

    def run(self) -> None:
        """Connect to the camera, set up output queues, and start the publish and save timer."""
        self.device = depthai_camera_connect.connect(self, self.camera, self.pipeline)
        self.set_up_publish_output_queues()
        self.set_up_video_encoder_output_queues_and_files()
        self.publish_and_save_timer = self.create_timer(1 / self.framerate, self.publish_and_save)

    def set_up_publish_output_queues(self) -> None:
        """Set up output queues for publishing frames."""
        if self.publish_rgb_video:
            self.rgbVideoQueue = self.device.getOutputQueue(name='rgbVideo', maxSize=4, blocking=False)

        if self.publish_rgb_preview:
            self.rgbPreviewQueue = self.device.getOutputQueue(name='rgbPreview', maxSize=4, blocking=False)

        if self.publish_left:
            self.leftQueue = self.device.getOutputQueue(name='left', maxSize=4, blocking=False)

        if self.publish_right:
            self.rightQueue = self.device.getOutputQueue(name='right', maxSize=4, blocking=False)

        if self.publish_disparity:
            self.disparityQueue = self.device.getOutputQueue(name='disparity', maxSize=4, blocking=False)

        if self.publish_depth:
            self.depthQueue = self.device.getOutputQueue(name='depth', maxSize=4, blocking=False)


    def set_up_video_encoder_output_queues_and_files(self) -> None:
        """Set up output queus for encoded video streams and open files to save them to."""
        if self.save_rgb_video:
            self.veRgbVideoQueue = self.device.getOutputQueue(name='veRgbVideo', maxSize=4, blocking=False)
            self.rgb_video_file = self.rgb_video_file_path.open('wb')

        if self.save_rgb_preview:
            self.veRgbPreviewQueue = self.device.getOutputQueue(name='veRgbPreview', maxSize=4, blocking=False)
            self.rgb_preview_file = self.rgb_preview_file_path.open('wb')

        if self.save_left:
            self.veLeftQueue = self.device.getOutputQueue(name='veLeft', maxSize=4, blocking=False)
            self.left_file = self.left_file_path.open('wb')

        if self.save_right:
            self.veRightQueue = self.device.getOutputQueue(name='veRight', maxSize=4, blocking=False)
            self.right_file = self.right_file_path.open('wb')

        if self.save_disparity:
            self.veDisparityQueue = self.device.getOutputQueue(name='veDisparity', maxSize=4, blocking=False)
            self.disparity_file = self.disparity_file_path.open('wb')

    def publish_and_save(self) -> None:
        """Publish frames and save encoded video streams."""
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
            raw_img_disparity = self.disparityQueue.get()
            img_disparity = raw_img_disparity.getCvFrame()
            image_msg_disparity = self.image_tools.convert_to_ros_compressed_msg(img_disparity)
            self.stream_publisher_disparity.publish(image_msg_disparity)

        if self.publish_depth:
            raw_img_depth = self.depthQueue.get()
            img_depth = raw_img_depth.getCvFrame()
            image_msg_depth = self.image_tools.convert_depth_to_ros_compressed_msg(img_depth, 'mono16')
            self.stream_publisher_depth.publish(image_msg_depth)

        # Save encoded video streams to files
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

    def destroy_node(self) -> None:
        """
        Destroy node.

        Stop the publish and save timer, close the connection to the camera, close all files, and provide commands to
        convert the encoded video files to playable videos.
        """
        # Stop the timer
        if self.publish_and_save_timer is not None:
            self.publish_and_save_timer.cancel()
            self.publish_and_save_timer = None

        # Close the device
        if self.device is not None:
            self.device.close()
            self.device = None

        # Close files
        if self.save_rgb_video and self.rgb_video_file is not None:
            self.rgb_video_file.close()
        if self.save_rgb_preview and self.rgb_preview_file is not None:
            self.rgb_preview_file.close()
        if self.save_left and self.left_file is not None:
            self.left_file.close()
        if self.save_right and self.right_file is not None:
            self.right_file.close()
        if self.save_disparity and self.disparity_file is not None:
            self.disparity_file.close()

        # Provide commands to convert encoded video files to playable videos
        self.get_logger().info('To convert the encoded video files to playable videos, run the following commands:')
        if self.save_rgb_video:
            rgb_video_command = self.PLAYABLE_VIDEO_COMMAND_TEMPLATE.format(
                framerate=self.framerate,
                input_file=self.rgb_video_file_path,
                options=self.H265_CONVERT_OPTIONS,
                output_file=self.rgb_video_file_path.with_suffix(self.PLAYABLE_VIDEO_SUFFIX),
            )
            self.get_logger().info(rgb_video_command)
        if self.save_rgb_preview:
            rgb_preview_command = self.PLAYABLE_VIDEO_COMMAND_TEMPLATE.format(
                framerate=self.framerate,
                input_file=self.rgb_preview_file_path,
                options=self.H265_CONVERT_OPTIONS,
                output_file=self.rgb_preview_file_path.with_suffix(self.PLAYABLE_VIDEO_SUFFIX),
            )
            self.get_logger().info(rgb_preview_command)
        if self.save_left:
            left_command = self.PLAYABLE_VIDEO_COMMAND_TEMPLATE.format(
                framerate=self.framerate,
                input_file=self.left_file_path,
                options=self.H264_CONVERT_OPTIONS,
                output_file=self.left_file_path.with_suffix(self.PLAYABLE_VIDEO_SUFFIX),
            )
            self.get_logger().info(left_command)
        if self.save_right:
            right_command = self.PLAYABLE_VIDEO_COMMAND_TEMPLATE.format(
                framerate=self.framerate,
                input_file=self.right_file_path,
                options=self.H264_CONVERT_OPTIONS,
                output_file=self.right_file_path.with_suffix(self.PLAYABLE_VIDEO_SUFFIX),
            )
            self.get_logger().info(right_command)
        if self.save_disparity:
            disparity_command = self.PLAYABLE_VIDEO_COMMAND_TEMPLATE.format(
                framerate=self.framerate,
                input_file=self.disparity_file_path,
                options=self.H265_CONVERT_OPTIONS,
                output_file=self.disparity_file_path.with_suffix(self.PLAYABLE_VIDEO_SUFFIX),
            )
            self.get_logger().info(disparity_command)

        super().destroy_node()


def main(args: None = None) -> None:
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
