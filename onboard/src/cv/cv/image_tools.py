import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import CompressedImage, Image


class ImageTools:
    """This file houses several converters from ROS messages to other types, or from other types to ROS messages."""

    def __init__(self) -> None:
        self._cv_bridge = CvBridge()

    def convert_ros_msg_to_cv2(self, ros_data: CompressedImage, image_encoding: str='bgr8') -> np.ndarray:
        """Convert from a ROS Image message to a cv2 image."""
        try:
            return self._cv_bridge.imgmsg_to_cv2(ros_data, image_encoding)
        except CvBridgeError as e:
            if '[16UC1] is not a color format' in str(e) or '[8UC1] is not a color format' in str(e):
                raise CvBridgeError(
                    'You may be trying to use a Image method (Subscriber, Publisher, conversion) on a depth image '
                    'message. Original exception: ' + str(e)) from e
            raise CvBridgeError from e

    def convert_ros_compressed_to_cv2(self, compressed_msg: CompressedImage) -> np.ndarray:
        """Convert a ROS CompressedImage message to a CV2 image."""
        np_arr = np.frombuffer(compressed_msg.data, np.uint8)
        return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

    def convert_ros_compressed_msg_to_ros_msg(self, compressed_msg: CompressedImage, encoding: str = 'bgr8') -> Image:
        """Convert a ROS CompressedImage message to a ROS Image message."""
        cv2_img = self.convert_ros_compressed_to_cv2(compressed_msg)
        ros_img = self._cv_bridge.cv2_to_imgmsg(cv2_img, encoding=encoding)
        ros_img.header = compressed_msg.header
        return ros_img

    def convert_cv2_to_ros_msg(self, cv2_data: np.ndarray, image_encoding: str = 'bgr8') -> Image:
        """Convert from a cv2 image to a ROS Image message."""
        return self._cv_bridge.cv2_to_imgmsg(cv2_data, image_encoding)

    def convert_cv2_to_ros_compressed_msg(self, cv2_data: np.ndarray,
                                          compressed_format: str = 'jpg') -> CompressedImage:
        """Convert from cv2 image to ROS CompressedImage message."""
        return self._cv_bridge.cv2_to_compressed_imgmsg(cv2_data, dst_format=compressed_format)

    def convert_ros_msg_to_ros_compressed_msg(self, image: Image,
                                              image_encoding: str = 'bgr8',
                                              compressed_format: str = 'jpg') -> CompressedImage:
        """Convert from ROS Image message to ROS CompressedImage message."""
        cv2_img = self.convert_ros_msg_to_cv2(image, image_encoding)
        cimg_msg = self._cv_bridge.cv2_to_compressed_imgmsg(cv2_img, dst_format=compressed_format)
        cimg_msg.header = image.header
        return cimg_msg

    def convert_to_cv2(self, image: object, image_encoding: str = 'bgr8') -> np.ndarray:
        """Convert any kind of image to cv2."""
        cv2_img = None
        if type(image) is np.ndarray:
            cv2_img = image
        elif type(image) is Image:
            cv2_img = self.convert_ros_msg_to_cv2(image, image_encoding=image_encoding)
        elif type(image) is CompressedImage:
            cv2_img = self.convert_ros_compressed_to_cv2(image)
        else:
            raise TypeError('Cannot convert type: ' + str(type(image)))
        return cv2_img

    def convert_to_ros_msg(self, image: object, image_encoding: str = 'bgr8') -> Image:
        """Convert any kind of image to ROS Image message."""
        ros_msg = None
        if type(image) is np.ndarray:
            ros_msg = self.convert_cv2_to_ros_msg(image, encoding=image_encoding)
        elif type(image) is Image:
            ros_msg = image
        elif type(image) is CompressedImage:
            ros_msg = self.convert_ros_compressed_msg_to_ros_msg(image, encoding=image_encoding)
        else:
            raise TypeError('Cannot convert type: ' + str(type(image)))
        return ros_msg

    def convert_to_ros_compressed_msg(self, image: object, compressed_format: str = 'jpg',
                                      image_encoding: str = 'bgr8') -> CompressedImage:
        """Convert any kind of image to ROS Compressed Image message."""
        ros_cmp = None
        if type(image) is np.ndarray:
            ros_cmp = self.convert_cv2_to_ros_compressed_msg(image, compressed_format=compressed_format)
        elif type(image) is Image:
            ros_cmp = self.convert_ros_msg_to_ros_compressed_msg(image, image_encoding=image_encoding,
                                                                 compressed_format=compressed_format)
        elif type(image) is CompressedImage:
            ros_cmp = image
        else:
            raise TypeError('Cannot convert type: ' + str(type(image)))
        return ros_cmp

    def convert_cv_bridge_depth_encoding_to_encoding_string(self, cv_bridge_depth_encoding: str,
                                                            compressed: bool = False) -> str:
        """Convert CV bridge depth encoding to encoding string."""
        encoding_str = ''
        if cv_bridge_depth_encoding == 'mono16':
            encoding_str = '16UC1'
        elif cv_bridge_depth_encoding == 'mono8':
            encoding_str = '8UC1'
        else:
            raise TypeError('Cannot convert image with encoding: ' + cv_bridge_depth_encoding)

        if compressed:
            encoding_str += '; compressedDepth'

        return encoding_str

    def convert_encoding_string_to_cv_bridge_depth_encoding(self, encoding_str: str) -> str:
        """Convert encoding string to cv bridge depth encoding."""
        cv_bridge_depth_encoding = ''
        if encoding_str in ('16UC1', '16UC1; compressedDepth'):
            cv_bridge_depth_encoding = 'mono16'
        elif encoding_str in ('8UC1', '8UC1; compressedDepth'):
            cv_bridge_depth_encoding = 'mono8'
        else:
            raise TypeError('Cannot convert image with encoding: ' + encoding_str)

        return cv_bridge_depth_encoding

    def convert_depth_to_ros_msg(self, image: object, image_encoding: str) -> Image:
        """Convert image with specific image encoding to ros message."""
        ros_msg = None
        if isinstance(image, np.ndarray):
            ros_msg = self.convert_cv2_to_ros_msg(image, encoding=image_encoding)
        elif isinstance(image, Image):
            image.encoding = self.convert_cv_bridge_depth_encoding_to_encoding_string(image_encoding)
            ros_msg = image
        elif isinstance(image, CompressedImage):
            ros_msg = self.convert_compressed_depth_to_image_msg(image)
        else:
            raise TypeError('Cannot convert type: ' + str(type(image)))
        return ros_msg

    def convert_depth_to_ros_compressed_msg(self, image: object, image_encoding: str) -> CompressedImage:
        """Convert image with image encoding to ros CompressedImage message."""
        ros_cmp = None
        if isinstance(image, np.ndarray):
            ros_cmp = self.convert_cv2_to_ros_compressed_msg(image, compressed_format='png')
            ros_cmp.format = self.convert_cv_bridge_depth_encoding_to_encoding_string(image_encoding, compressed=True)

        elif isinstance(image, Image):
            image.encoding = image_encoding
            ros_cmp = self.convert_ros_msg_to_ros_compressed_msg(image, image_encoding=image_encoding,
                                                                 compressed_format='png')
            ros_cmp.format = self.convert_cv_bridge_depth_encoding_to_encoding_string(image_encoding, compressed=True)

        elif isinstance(image, CompressedImage):
            ros_cmp = image
        else:
            raise TypeError('Cannot convert type: ' + str(type(image)))
        return ros_cmp

    def convert_depth_to_cv2(self, image: object) -> np.ndarray:
        """Convert image to CV2 image."""
        cv2_img = None
        if isinstance(image, np.ndarray):
            cv2_img = image
        elif isinstance(image, Image):
            encoding = image.encoding.split(';')[0]
            cv_encoding = self.convert_encoding_string_to_cv_bridge_depth_encoding(encoding)
            cv2_img = self.convert_ros_msg_to_cv2(image, image_encoding=cv_encoding)
        elif isinstance(image, CompressedImage):
            cv2_img = self.convert_compressed_depth_to_cv2(image)
        else:
            raise TypeError('Cannot convert type: ' + str(type(image)))
        return cv2_img

    def convert_compressed_depth_to_image_msg(self, compressed_image: CompressedImage) -> Image:
        """
        Convert a compressedDepth topic image into a ROS Image message.

        compressed_image must be from a topic /bla/compressedDepth as it's encoded in PNG.
        Code from: https://answers.ros.org/question/249775/display-compresseddepth-image-python-cv2/
        """
        encoding = compressed_image.format.split(';')[0]
        cv_encoding = self.convert_encoding_string_to_cv_bridge_depth_encoding(encoding)

        depth_img_raw = self.convert_compressed_depth_to_cv2(compressed_image)
        img_msg = self._cv_bridge.cv2_to_imgmsg(depth_img_raw, cv_encoding)
        img_msg.header = compressed_image.header
        img_msg.encoding = encoding
        return img_msg

    def convert_compressed_depth_to_cv2(self, compressed_depth: CompressedImage) -> np.ndarray:
        """
        Convert a compressedDepth topic image into a cv2 image.

        compressed_depth must be from a topic /bla/compressedDepth as it's encoded in PNG.
        Code from: https://answers.ros.org/question/249775/display-compresseddepth-image-python-cv2/
        """
        depth_fmt, compr_type = compressed_depth.format.split(';')
        # remove white space
        depth_fmt = depth_fmt.strip()
        compr_type = compr_type.strip()
        if compr_type != 'compressedDepth':
            exception = "Compression type is not 'compressedDepth'. You probably subscribed to the wrong topic."
            raise Exception(exception)  # noqa: TRY002

        # If we compressed it with opencv, there is nothing to strip
        # If it comes from a robot/sensor, it has 12 useless bytes apparently
        # remove header from raw data, if necessary
        depth_header_size = 0 if 'PNG' in compressed_depth.data[:12] else 12
        raw_data = compressed_depth.data[depth_header_size:]

        depth_img_raw = cv2.imdecode(np.frombuffer(raw_data, np.uint8),
                                     # the cv2.CV_LOAD_IMAGE_UNCHANGED has been removed
                                     -1)  # cv2.CV_LOAD_IMAGE_UNCHANGED)
        if depth_img_raw is None:
            # probably wrong header size
            exception = "Could not decode compressed depth image. You may need to change 'depth_header_size'!"
            raise Exception(exception)  # noqa: TRY002
        return depth_img_raw
