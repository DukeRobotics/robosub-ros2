from collections.abc import Sequence
from pathlib import Path

import cv2
import depthai as dai
import numpy as np
import rclpy
import resource_retriever as rr
import yaml
from sensor_msgs.msg import CompressedImage

from cv.depthai_spatial_detection import DepthAISpatialDetector

USB_CAMERA_CONFIG_PATH = 'package://cv/config/usb_cameras.yaml'

class DepthAIUSBDetector(DepthAISpatialDetector):
    """Using an external camera feed as input, compute detections on a DepthAI camera."""
    def __init__(self) -> None:
        """
        Initialize the ROS node.

        Loads the yaml file at cv/models/depthai_models.yaml.
        """
        super().__init__(run=False)
        self.usb_camera = self.declare_parameter('usb_camera', 'front').value

        with Path.open(rr.get_filename(USB_CAMERA_CONFIG_PATH, use_protocol=False)) as f:
            usb_cameras = yaml.safe_load(f)
        usb_camera_config = usb_cameras[self.usb_camera]

        # Update camera constants
        self.FOCAL_LENGTH = usb_camera_config['focal_length']
        self.SENSOR_SIZE = (usb_camera_config['sensor_size']['width'], usb_camera_config['sensor_size']['height'])

        self.create_subscription(CompressedImage, usb_camera_config['topic'], self._update_latest_img, 1)

        self.run()

    # Format a cv2 image to be sent to the device
    def to_planar(self, arr: np.ndarray, shape: Sequence) -> np.ndarray:
        """
        Resize a cv2 image to a given shape and transpose it to be in the correct format to send to the DepthAI device.

        Args:
            arr (np.ndarray): Image to resize and transpose.
            shape (Sequence): Shape to resize the image to. In order (width, height).
        """
        return cv2.resize(arr, shape).transpose(2, 0, 1).flatten()

    def _update_latest_img(self, img_msg: CompressedImage) -> None:
        """
        Send an image to the device for detection.

        Args:
            img_msg (sensor_msgs.msg.CompressedImage): Image to send to the device
        """
        model = self.models[self.current_model_name]

        latest_img = self.image_tools.convert_to_cv2(img_msg)

        # Input queue will be used to send video frames to the device.
        if self.device:
            input_queue = self.device.getInputQueue('nn_input')

            # Send a message to the ColorCamera to capture a still image
            img = dai.ImgFrame()
            img.setType(dai.ImgFrame.Type.BGR888p)

            img.setData(self.to_planar(latest_img, model['input_size']))

            img.setWidth(model['input_size'][0])
            img.setHeight(model['input_size'][1])

            input_queue.send(img)

    def build_pipeline(self, nn_blob_path: Path, sync_nn: bool) -> dai.Pipeline:
        """
        Get the DepthAI pipeline performing object detection using the USB camera feed.

        Inspiration taken from
        https://docs.luxonis.com/projects/api/en/latest/samples/SpatialDetection/spatial_tiny_yolo/.
        The output queues available from this pipeline are:
            - "rgb": Contains the images input to the neural network.
            - "detections": Contains SpatialImgDetections messages which includes bounding boxes for detections as well
                as XYZ coordinates of the detected objects.

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
        xout_rgb = pipeline.create(dai.node.XLinkOut)
        xout_rgb.setStreamName('rgb')

        xout_nn = pipeline.create(dai.node.XLinkOut)
        xout_nn.setStreamName('detections')

        xin_nn_input = pipeline.create(dai.node.XLinkIn)
        xin_nn_input.setStreamName('nn_input')
        xin_nn_input.setNumFrames(2)
        xin_nn_input.setMaxDataSize(model['input_size'][0] * model['input_size'][1] * 3)

        # General spatial detection network parameters
        spatial_detection_network = pipeline.create(dai.node.YoloDetectionNetwork)
        spatial_detection_network.setBlobPath(nn_blob_path)
        spatial_detection_network.setConfidenceThreshold(model['confidence_threshold'])
        spatial_detection_network.input.setBlocking(False)

        # Yolo specific parameters
        spatial_detection_network.setNumClasses(len(model['classes']))
        spatial_detection_network.setCoordinateSize(model['coordinate_size'])
        spatial_detection_network.setAnchors(np.array(model['anchors']))
        spatial_detection_network.setAnchorMasks(model['anchor_masks'])
        spatial_detection_network.setIouThreshold(model['iou_threshold'])

        # Linking
        xin_nn_input.out.link(spatial_detection_network.input)
        spatial_detection_network.out.link(xout_nn.input)

        if sync_nn:
            spatial_detection_network.passthrough.link(xout_rgb.input)
        else:
            xin_nn_input.out.link(xout_rgb.input)

        return pipeline

    def init_queues(self, device: dai.Device) -> None:
        """
        Assign queues from the pipeline to dictionary of queues.

        Args:
            device (DepthAI.Device): DepthAI.Device object for the connected device.
                See https://docs.luxonis.com/projects/api/en/latest/components/device/
        """
        super().init_queues(device)
        self.input_queue = device.getInputQueue(name='nn_input', maxSize=1, blocking=False)

    def detect(self) -> None:
        """Get current detections from output queues and publish."""
        super().detect()


def main(args: list[str] | None = None) -> None:
    """Define the main function that initiates the node."""
    rclpy.init(args=args)
    depthai_usb_detector = DepthAIUSBDetector()

    try:
        rclpy.spin(depthai_usb_detector)
    except KeyboardInterrupt:
        pass
    finally:
        depthai_usb_detector.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
