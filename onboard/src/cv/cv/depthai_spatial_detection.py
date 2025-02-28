import math
from pathlib import Path

import cv2
import depthai as dai
import numpy as np
import rclpy
import resource_retriever as rr
import yaml
from custom_msgs.msg import CVObject, SonarSweepRequest, SonarSweepResponse
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String

from cv import correct, depthai_camera_connect
from cv.image_tools import ImageTools
from cv.utils import DetectionVisualizer, calculate_relative_pose

MM_IN_METER = 1000
DEPTHAI_OBJECT_DETECTION_MODELS_FILEPATH = 'package://cv/models/depthai_models.yaml'
HORIZONTAL_FOV = 95
SONAR_DEPTH = 10
SONAR_RANGE = 1.75
SONAR_REQUESTS_PATH = 'sonar/request'
SONAR_RESPONSES_PATH = 'sonar/cv/response'
TASK_PLANNING_REQUESTS_PATH = 'controls/desired_feature'
LOOP_RATE = 10

GATE_IMAGE_WIDTH = 0.2452  # Width of gate images in meters
GATE_IMAGE_HEIGHT = 0.2921  # Height of gate images in meters
FOCAL_LENGTH = 2.75  # Focal length of camera in mm
SENSOR_SIZE = (6.2868, 4.712)  # Sensor size in mm
ISP_IMG_SHAPE = (4056, 3040)  # Size of ISP image


class DepthAISpatialDetector(Node):
    """Compute detections on live camera feed and publish spatial coordinates for detected objects."""
    def __init__(self) -> None:
        """Initialize the ROS node. Loads the yaml file at cv/models/depthai_models.yaml."""
        super().__init__('depthai_spatial_detection')
        self.running_model = self.declare_parameter('running_model', 'yolov7_tiny_2023_main').value
        self.rgb_raw = self.declare_parameter('rgb_raw', True).value
        self.rgb_detections = self.declare_parameter('rgb_detections', True).value
        self.queue_depth = self.declare_parameter('queue_depth', False).value  # Whether to output depth map
        self.sync_nn = self.declare_parameter('sync_nn', True).value
        self.using_sonar = self.declare_parameter('using_sonar', False).value
        self.show_class_name = self.declare_parameter('show_class_name', True).value
        self.show_confidence = self.declare_parameter('show_confidence', True).value
        self.correct_color = self.declare_parameter('correct_color', False).value

        with Path.open(rr.get_filename(DEPTHAI_OBJECT_DETECTION_MODELS_FILEPATH,
                                  use_protocol=False)) as f:
            self.models = yaml.safe_load(f)

        self.camera = 'front'
        self.pipeline = None
        self.publishers_dict = {}  # Keys are the class names of a given model
        self.output_queues = {}  # Keys are "rgb", "depth", and "detections"
        self.connected = False
        self.current_model_name = None
        self.classes = None
        self.camera_pixel_width = None
        self.camera_pixel_height = None
        self.detection_feed_publisher = None
        self.rgb_preview_publisher = None
        self.detection_visualizer = None

        self.image_tools = ImageTools()

        self.sonar_response = (0, 0)
        self.in_sonar_range = True

        # By default the first task is going through the gate
        self.current_priority = 'buoy_abydos_serpenscaput'

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Initialize publishers and subscribers for sonar/task planning
        self.sonar_requests_publisher = self.create_publisher(
            SonarSweepRequest, SONAR_REQUESTS_PATH, 10)
        self.sonar_response_subscriber = self.create_subscription(
            SonarSweepResponse, SONAR_RESPONSES_PATH, self.update_sonar,qos_profile)
        self.desired_detection_feature = self.create_subscription(
            String, TASK_PLANNING_REQUESTS_PATH, self.update_priority,qos_profile)

        self.run()

    def build_pipeline(self, nn_blob_path: Path, sync_nn: bool) -> dai.Pipeline:  # noqa: ARG002
        """
        Get the DepthAI Pipeline for 3D object localization.

        Inspiration taken from
        https://docs.luxonis.com/projects/api/en/latest/samples/SpatialDetection/spatial_tiny_yolo/.
        To understand the DepthAI pipeline structure, please see https://docs.luxonis.com/projects/api/en/latest/.
        This pipeline computes the depth map using the two mono cameras. This depth map and the RGB feed are fed into
        the YoloSpatialDetection Node, which detects objects and computes the average depth within the bounding box
        for any detected object. The object detection model for this node is loaded from the nnBlobPath. For info
        about the YoloSpatialDetection Node, see
        https://docs.luxonis.com/projects/api/en/latest/components/nodes/yolo_spatial_detection_network/.
        The output queues available from this pipeline are:
            - "rgb": contains the 400x400 RGB preview of the camera feed.
            - "detections": contains SpatialImgDetections messages (https://docs.luxonis.com/projects/api/en/latest/
            components/messages/spatial_img_detections/#spatialimgdetections), which includes bounding boxes for
            detections as well as XYZ coordinates of the detected objects.
            - "depth": contains ImgFrame messages with UINT16 values representing the depth in millimeters by default.
                See the property depth in https://docs.luxonis.com/projects/api/en/latest/components/nodes/stereo_depth/

        Args:
            nn_blob_path (str): Path to blob file used for object detection.
            sync_nn (bool): If True, sync the RGB output feed with the detection from the neural network. Needed if the
                RGB feed output will be used and needs to be synced with the object detections.

        Returns:
            depthai.Pipeline: Pipeline object to compute.
        """
        model = self.models[self.current_model_name]

        pipeline = dai.Pipeline()

        # Define sources and outputs
        cam_rgb = pipeline.create(dai.node.ColorCamera)
        spatial_detection_network = pipeline.create(dai.node.YoloDetectionNetwork)
        image_manip = pipeline.create(dai.node.ImageManip)

        xout_nn = pipeline.create(dai.node.XLinkOut)
        xout_nn.setStreamName('detections')

        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName('rgb')
        cam_rgb.video.link(xout_rgb.input)

        xin_nn_input = pipeline.create(dai.node.XLinkIn)
        xin_nn_input.setStreamName('nn_input')
        xin_nn_input.setNumFrames(2)
        xin_nn_input.setMaxDataSize(416*416*3)

        # Camera properties
        cam_rgb.setPreviewSize(model['input_size'])
        cam_rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_12_MP)
        cam_rgb.setInterleaved(False)
        cam_rgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam_rgb.setPreviewKeepAspectRatio(False)

        cam_rgb.setIspNumFramesPool(3)  # keep this high default
        cam_rgb.setPreviewNumFramesPool(1)  # breaks if <1
        cam_rgb.setRawNumFramesPool(2)  # breaks if <2
        cam_rgb.setStillNumFramesPool(0)
        cam_rgb.setVideoNumFramesPool(1)  # breaks if <1

        image_manip.initialConfig.setResize(model['input_size'])
        image_manip.initialConfig.setKeepAspectRatio(False)
        image_manip.setMaxOutputFrameSize(model['input_size'][0] * model['input_size'][1] * 3)
        image_manip.setNumFramesPool(1)

        # General spatial detection network parameters
        spatial_detection_network.setBlobPath(nn_blob_path)
        spatial_detection_network.setConfidenceThreshold(model['confidence_threshold'])
        spatial_detection_network.input.setBlocking(False)

        # Yolo specific parameters
        spatial_detection_network.setNumClasses(len(model['classes']))
        spatial_detection_network.setCoordinateSize(model['coordinate_size'])
        spatial_detection_network.setAnchors(np.array(model['anchors']))
        spatial_detection_network.setAnchorMasks(model['anchor_masks'])
        spatial_detection_network.setIouThreshold(model['iou_threshold'])

        xin_nn_input.out.link(spatial_detection_network.input)

        cam_rgb.isp.link(image_manip.inputImage)
        image_manip.out.link(xout_rgb.input)

        spatial_detection_network.out.link(xout_nn.input)

        return pipeline

    def init_model(self, model_name: str) -> None:
        """
        Create and assign the pipeline and set the current model name.

        Args:
            model_name (str): Name of the model. The model name should match a key in cv/models/depthai_models.yaml.
                For example, if the yaml file contains the following:

                gate:
                    classes: ['gate', 'gate_side', 'gate_tick', 'gate_top', 'start_gate']
                    topic: cv/
                    weights: yolo_v4_tiny_openvino_2021.3_6shave-2022-7-21_416_416.blob
                    input_size: [416, 416]
                    coordinate_size: 4
                    anchors: [10, 14, 23, 27, 37, 58, 81, 82, 135, 169, 344, 319]
                    anchor_masks: {"side26": [0, 1, 2], "side13": [3, 4, 5]}

                Then, a possible model name is "gate".
        """
        # If the model is already set, don't reinitialize
        if model_name == self.current_model_name:
            return

        self.current_model_name = model_name

        model = self.models[model_name]

        self.classes = model['classes']

        self.camera_pixel_width, self.camera_pixel_height = model['input_size']

        self.colors = model['colors']

        blob_path = rr.get_filename(f"package://cv/models/{model['weights']}",
                                    use_protocol=False)
        self.pipeline = self.build_pipeline(blob_path, self.sync_nn)

    def init_publishers(self, model_name: str) -> None:
        """
        Initialize the publishers for the node. A publisher is created for each class that the model predicts.

        The publishers are created in format: "model['topic']/camera/model_name".

        Args:
            model_name (str): Name of the model that is being used.
        """
        model = self.models[model_name]

        # Create a CVObject publisher for each class
        publisher_dict = {}
        for model_class in model['classes']:
            publisher_name = f'{model['topic']}/{self.camera}/{model_class}'
            publisher_dict[model_class] = self.create_publisher(CVObject,
                                                          publisher_name,
                                                          10)
        self.publishers_dict = publisher_dict

        # Create CompressedImage publishers for the raw RGB feed, detections feed, and depth feed
        if self.rgb_raw:
            self.rgb_preview_publisher = self.create_publisher(CompressedImage, 'camera/front/rgb/preview/compressed',
                                                         10)

        if self.rgb_detections:
            self.detection_feed_publisher = self.create_publisher(CompressedImage, 'cv/front/detections/compressed',
                                                            10)

    def init_queues(self, device: dai.Device) -> None:  # noqa: ARG002
        """
        Assign output queues from the pipeline to dictionary of queues.

        Args:
            device (DepthAI.Device): DepthAI.Device object for the connected device.
                See https://docs.luxonis.com/projects/api/en/latest/components/device/
        """
        # If the output queues are already set, don't reinitialize
        if self.connected:
            return

        # Assign output queues
        self.output_queues['rgb'] = self.device.getOutputQueue(name='rgb', maxSize=1, blocking=False)

        self.output_queues['detections'] = self.device.getOutputQueue(name='detections', maxSize=1, blocking=False)

        self.input_queue = self.device.getInputQueue(name='nn_input', maxSize=1, blocking=False)

        self.connected = True  # Flag that the output queues have been initialized

        self.detection_visualizer = DetectionVisualizer(self.classes, self.colors,
                                                        self.show_class_name, self.show_confidence)

    def detect(self) -> None:
        """Get current detections from output queues and publish."""
        # init_output_queues must be called before detect
        if not self.connected:
            self.get_logger().warn('Output queues are not initialized so cannot detect. Call init_output_queues first.')
            return

        # Format a cv2 image to be sent to the device
        def to_planar(arr: np.ndarray, shape: tuple) -> np.ndarray:
            return cv2.resize(arr, shape).transpose(2, 0, 1).flatten()

        inpreview = self.output_queues['rgb'].get()
        frame = inpreview.getCvFrame()

        # Publish raw RGB feed
        if self.rgb_raw:
            frame_img_msg = self.image_tools.convert_to_ros_compressed_msg(frame)
            self.rgb_preview_publisher.publish(frame_img_msg)

        # Underwater color correction
        if self.correct_color:
            mat = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame = correct.correct(mat)

        # Send a message to the ColorCamera to capture a still image
        img = dai.ImgFrame()
        img.setType(dai.ImgFrame.Type.BGR888p)
        img.setData(to_planar(frame, (416, 416)))
        img.setWidth(416)
        img.setHeight(416)
        self.input_queue.send(img)

        # Get detections from output queues
        indet = self.output_queues['detections'].tryGet()
        if not indet:
            return
        detections = indet.detections

        detections_dict = {}
        for detection in detections:
            prev_conf, _ = detections_dict.get(detection.label, (None, None))
            if (prev_conf is not None and detection.confidence > prev_conf) or prev_conf is None:
                detections_dict[detection.label] = detection.confidence, detection

        detections = [detection for _, detection in detections_dict.values()]
        model = self.models[self.current_model_name]

        # Publish detections feed
        if self.rgb_detections:
            detections_visualized = self.detection_visualizer.visualize_detections(frame, detections)
            detections_img_msg = self.image_tools.convert_to_ros_compressed_msg(detections_visualized)
            self.detection_feed_publisher.publish(detections_img_msg)

        # Process and publish detections. If using sonar, override det robot x coordinate
        for detection in detections:

            # Bounding box
            bbox = (detection.xmin, detection.ymin,
                    detection.xmax, detection.ymax)

            # Label name
            label_idx = detection.label
            label = self.classes[label_idx]

            confidence = detection.confidence

            # Calculate relative pose
            det_coords_robot_mm = calculate_relative_pose(bbox, model['input_size'], model['sizes'][label],
                                                          FOCAL_LENGTH, SENSOR_SIZE, 2)

            # Find yaw angle offset
            left_end_compute = self.compute_angle_from_x_offset(detection.xmin * self.camera_pixel_width)
            right_end_compute = self.compute_angle_from_x_offset(detection.xmax * self.camera_pixel_width)
            midpoint = (left_end_compute + right_end_compute) / 2.0
            yaw_offset = math.radians(midpoint)  # Degrees to radians

            # Create a new sonar request msg object if using sonar and the current detected
            # class is the desired class to be returned to task planning
            if self.using_sonar and label == self.current_priority:

                # Construct sonar request message
                sonar_request_msg = SonarSweepRequest()
                sonar_request_msg.start_angle = int(left_end_compute)
                sonar_request_msg.end_angle = int(right_end_compute)
                sonar_request_msg.distance_of_scan = int(SONAR_DEPTH)

                # Make a request to sonar if it is not busy
                self.sonar_requests_publisher.publish(sonar_request_msg)

                # Try calling sonar on detected bounding box
                # If sonar responds, then override existing robot-frame x info;
                # else, keep default
                if self.sonar_response != (0, 0) and self.in_sonar_range:
                    det_coords_robot_mm = (self.sonar_response[0],  # Override x
                                           det_coords_robot_mm[1],  # Maintain original y
                                           det_coords_robot_mm[2])  # Maintain original z

            self.publish_prediction(
                bbox, det_coords_robot_mm, yaw_offset, label, confidence,
                (self.camera_pixel_height, self.camera_pixel_width), self.using_sonar)

    def publish_prediction(self, bbox: tuple, det_coords: tuple, yaw: float, label: str, confidence: float,
                           shape: tuple, using_sonar: bool) -> None:
        """
        Publish predictions to label-specific topic. Publishes to /model['topic']/[camera]/[label].

        Args:
            bbox (tuple): Tuple for the bounding box. Values are from 0-1, where X increases left to right and Y
                increases top to bottom.
            det_coords (tuple): Tuple with the X, Y, and Z values in meters in the robot rotational reference frame.
            yaw (float): Yaw angle in radians.
            label (str): Predicted label for the detection.
            confidence (float): Confidence for the detection, from 0 to 1.
            shape (tuple): Tuple with the (height, width) of the image. NOTE: This is in reverse order from the model.
            using_sonar (bool): Whether or not sonar is being used.
        """
        object_msg = CVObject()

        object_msg.header.stamp.sec = self.get_clock().now().seconds_nanoseconds()[0]
        object_msg.header.stamp.nanosec = self.get_clock().now().seconds_nanoseconds()[1]

        object_msg.label = label
        object_msg.score = confidence

        object_msg.coords.x = det_coords[0]
        object_msg.coords.y = det_coords[1]
        object_msg.coords.z = det_coords[2]

        object_msg.xmin = bbox[0]
        object_msg.ymin = bbox[1]
        object_msg.xmax = bbox[2]
        object_msg.ymax = bbox[3]

        object_msg.yaw = yaw

        object_msg.height = shape[0]
        object_msg.width = shape[1]

        object_msg.sonar = using_sonar

        if self.publishers_dict:
            self.get_logger().debug('Publishing')
            self.publishers_dict[label].publish(object_msg)

    def update_sonar(self, sonar_results: object) -> None:
        """
        Listen to sonar response.

        Updates instance variable self.sonar_response based on what sonar throws back if it is in range
        (> SONAR_RANGE = 1.75m).

        Args:
            sonar_results (object): The sonar response message.
        """
        # Check to see if the sonar is in range - are results from sonar valid?
        if not sonar_results.is_object:
            return
        if sonar_results.pose.position.x > SONAR_RANGE and sonar_results.pose.position.x <= SONAR_DEPTH:
            self.in_sonar_range = True
            self.sonar_response = (sonar_results.pose.position.x, sonar_results.pose.position.y)
        else:
            self.in_sonar_range = False

    def update_priority(self, obj: str) -> None:
        """
        Update the current priority class. If the priority class is detected, sonar will be called.

        Args:
            obj (str): The current priority class.
        """
        self.current_priority = obj

    def run(self) -> bool:
        """
        Run the selected model on the connected device.

        Returns:
            bool: False if the model is not in cv/models/depthai_models.yaml. Otherwise, runs the model on the device.
        """
        # Check if model is valid
        if self.running_model not in self.models:
            return False

        # Setup pipeline and publishers
        self.init_model(self.running_model)
        self.init_publishers(self.running_model)

        self.device = depthai_camera_connect.connect(self, self.camera, self.pipeline)
        self.init_queues(self.device)

        self.detect_timer = self.create_timer(1 / LOOP_RATE, self.detect)

        return True

    def compute_angle_from_x_offset(self, x_offset: float) -> float:
        """
        Compute angle from x offset.

        See: https://math.stackexchange.com/questions/1320285/convert-a-pixel-displacement-to-angular-rotation
        for implementation details.

        Args:
            x_offset (int): x pixels from center of image.

        Returns:
            float: Angle in degrees.
        """
        image_center_x = self.camera_pixel_width / 2.0
        return math.degrees(math.atan((x_offset - image_center_x) * 0.005246675486))

    def compute_angle_from_y_offset(self, y_offset: float) -> float:
        """
        Compute angle from y offset.

        See https://math.stackexchange.com/questions/1320285/convert-a-pixel-displacement-to-angular-rotation for
        implementation details.

        Args:
            y_offset (int): y pixels from center of image.

        Returns:
            float: Angle in degrees.
        """
        image_center_y = self.camera_pixel_height / 2.0
        return math.degrees(math.atan((y_offset - image_center_y) * 0.003366382395))

    def coords_to_angle(self, min_x: float, max_x: float) -> tuple[float, float]:
        """
        Take in a detected bounding box from the camera and return the angle range to sonar sweep.

        Args:
            min_x (float): Minimum x coordinate of camera bounding box (robot y).
            max_x (float): Maximum x coordinate of camera bounding box (robot y).

        Returns:
            tuple[float, float]: Tuple containing the minimum and maximum angle to sweep sonar.
        """
        distance_to_screen = self.camera_pixel_width / 2 * \
            1/math.tan(math.radians(HORIZONTAL_FOV/2))
        min_angle = math.degrees(np.arctan(min_x/distance_to_screen))
        max_angle = math.degrees(np.arctan(max_x/distance_to_screen))
        return min_angle, max_angle


def mm_to_meters(val_mm: float) -> float:
    """
    Convert value from millimeters to meters.

    Args:
        val_mm (float): Value in millimeters.

    Returns:
        float: Input value converted to meters.
    """
    return val_mm / MM_IN_METER


def camera_frame_to_robot_frame(cam_x: float, cam_y: float, cam_z: float) -> tuple[float, float, float]:
    """
    Convert coordinates in camera reference frame to coordinates in robot reference frame.

    This ONLY ACCOUNTS FOR THE ROTATION BETWEEN COORDINATE FRAMES, and DOES NOT ACCOUNT FOR THE TRANSLATION.

    Args:
        cam_x (float): X coordinate of object in camera reference frame.
        cam_y (float): Y coordinate of object in camera reference frame.
        cam_z (float): Z coordinate of object in camera reference frame.

    Returns:
        tuple[float, float, float]: X, Y, Z coordinates of object in robot rotational reference frame.
    """
    robot_y = -cam_x
    robot_z = cam_y
    robot_x = cam_z
    return robot_x, robot_y, robot_z


def main(args: list[str] | None = None) -> None:
    """Spin DepthAISpatialDetector."""
    rclpy.init(args=args)
    depthai_spatial_detector = DepthAISpatialDetector()

    try:
        rclpy.spin(depthai_spatial_detector)
    except KeyboardInterrupt:
        depthai_spatial_detector.device.close()
    finally:
        depthai_spatial_detector.device.close()
        depthai_spatial_detector.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
