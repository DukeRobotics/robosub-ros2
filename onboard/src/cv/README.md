# Computer Vision

The computer vision package listens for images/frames coming from multiple cameras. The package will then run pre-trained machine learning models or an HSV filtering algorithm on frames from each camera and output bounding boxes for the various objects in the frames. These objects could be the gate, buoys, etc. The package will publish to different topics depending on which classes are being detected and which cameras are being used.

## Structure

The following are the folders and files in the CV package:

`assets`: Additional assets (usually images) that the scripts may use.

`config`: Robot-specific configuration files for the CV package. See [config](#config) for more details.

`cv`: Contains scripts that connect to the camera and perform detection.
* `bin_detector.py`: Detects bins using HSV filtering.
* `lane_marker_detector.py`: Detects the lane marker in Taishoff Aquatics Pavilion using HSV filtering.
* `buoy_detector.py`: Detects the buoy using HSV filtering.
* `config.py`: Constants used throughout the CV package.
* `depthai_camera_connect.py`: Connects to the OAK camera and uploads the image pipeline. Used by other DepthAI scripts.
* `depthai_usb_detection.py`: Publishes detections using a specified model in `depthai_models.yaml` on a USB camera image feed.
* `depthai_publish_save_streams.py`: Publishes a preview of the image feed from the OAK camera and saves encoded streams. This can be used to verify connection to the camera and to check if there are any issues with the camera feed.
* `depthai_spatial_detection.py`: Publishes live detections using a specified model in `depthai_models.yaml`.
* `image_tools.py`: Auxiliary functions to convert ROS image messages to other types and vice versa.
* `torpedo_target_detector.py`: Detects the torpedo target using HSV filtering.
* `path_marker_detector.py`: Detects path marker using HSV filtering.
* `pink_bins_detector.py`: Detects pink bins using HSV filtering.
* `usb_camera.py`: Publishes frames coming from a USB camera.
* `utils.py`: Auxiliary functions that visualize detections on an image feed and modularize certain repeated calculations.

`launch`: Contains the various launch files for our CV package. There are specific launch files for each script in `cv` in addition to the following:
* `usb_camera_detectors.launch.py`: Connects to all USB cameras and starts all detectors that use the USB camera feed.

`models`: Contains our pre-trained models and a `.yaml` file that specifies the details of each model.
* `depthai_models.yaml`: Specifies models for object detection. A model is specified by a name, what classes it predicts, the path to a `.blob` file, as well as other configuration parameters. The blob file format is specific to the processors that the OAK cameras use. See [DepthAI Cameras](#depthai-cameras) for more details.

The CV package also has dependencies in the `core/src/custom_msgs` folder.

## Config
The `config` directory contains robot-specific `.yaml` files. The format is as follows:
```yaml
depthai:
  ip_range: IP range to scan (CIDR format)
  cameras:
    front:
      number: human-readable label
      mac: MAC address of the camera
      horizontal_fov: horizontal fov
      focal_length: focal length
      sensor_size:
        width: width
        height: height
      frame_id: transform frame id of the camera
    ...

usb_cameras:
  front:
    device_path: path to device to read the stream from (e.g., /dev/video0); can be a symlinked path
    topic: topic to publish detections to
    horizontal_fov: horizontal_fov
    focal_length: focal_length
    sensor_size:
      width: width
      height: height
    img_size:
      width: width
      height: height
    frame_id: transform frame id of the camera
  ...
```

## DepthAI Cameras
This package contains code for the Luxonis OAK-D PoE camera, which uses a python package called [depthai](https://docs.luxonis.com/en/latest/). This camera handles neural network and image processing on the camera's processor, which necessitates a different code structure. Because of this, we have DepthAI-specific scripts and launch files that can be run for any Luxonis / DepthAI camera. For running other cameras, see the section below, titled [Non-DepthAI Cameras](#non-depthai-cameras).

For a full walkthrough to training and uploading a new DepthAI model to the CV package see [Computer Vision Workflow](https://wiki.duke-robotics.com/#/computer_vision/workflow) on our wiki.

### Setup

The DepthAI camera scripts only support YOLO models. After you train a YOLO model, you will receive a `.pt` file containing the weights of the model. You will also receive other information about the model, such as the classes it predicts, the input size, the anchor masks, and the anchors, which must be included in the `depthai_models.yaml` file.

1. Convert the weights file to a `.blob` file, which is necessary to run the model on the DepthAI camera. Go to [tools.luxonis.com](https://tools.luxonis.com/) and upload your `.pt` file. This will generate a `.blob` file that you can download.
2. Place the `.blob` file in the `cv/models` folder.
3. Add an entry for your model in the `cv/models/depthai_models.yaml` file. The format is as follows:

```yaml
model_name:
  classes: ["class1", "class2", ...]
  sizes: {"class1": [width, height], "class2": [width, height], ...}
  topic: string
  weights: model_weights.blob
  input_size: [width, height]
  coordinate_size: int
  anchors: [anchor1, anchor2, ...]
  anchor_masks: {"mask1": [anchor1, anchor2, ...], "mask2": [anchor1, anchor2, ...], ...}
  iou_threshold: float
  confidence_threshold: float
  colors: ["color1", "color2", ...]
```

* `model_name`: A unique name for the model.
* `classes`: A list of the classes that the model predicts. Must be in the same order as the model's output.
* `sizes`: A dictionary mapping each class to its physical size in meters (width, height).
* `topic`: The first part of the topic to which the model's detections will be published. It should **not** include a trailing slash. For example, if the topic is `cv`, the model will publish to `cv/<camera>/<class>`.
* `weights`: The name of the `.blob` file containing the model's weights. Must be in the `cv/models` folder.
* `input_size`: The size of the input image to the model.
* `coordinate_size`: The size of the coordinates tuple used to represent the bounding boxes output by the model. This is usually 4.
* `anchors`: A list of the model's anchors. Provided by the model's output.
* `anchor_masks`: A dictionary mapping each mask to the anchors it uses. Provided by the model's output.
* `iou_threshold`: The IOU threshold used by the model for non-maximum suppression, which removes overlapping bounding boxes.
* `confidence_threshold`: The confidence threshold used by the model. The model will only publish detections with a confidence greater than this value.
* `colors`: A list of colors to use when drawing the bounding boxes on the input image to visualize the model's detections. There must be one color for each class. Each value is a string in hexadecimal format, _without the `#` symbol_.

### Running the Code
To stream the feed or perform spatial detection using the OAK camera, use `ros2 launch cv <module>` with the following files as needed.
* `depthai_publish_save_streams.xml`: Streams the live feed from the camera. You can choose what to publish from the camera (rgb video, rgb preview, left mono, right mono, disparity map, and depth map) by setting the appropriate boolean parameters. Also encodes the live feeds and saves them to files. You can choose what to save by setting the appropriate boolean parameters. You can also choose to automatically convert the encoded streams to video files.
* `depthai_spatial_detection.xml`: Connects to the camera and runs spatial detection. This requires a valid `.blob` file in `models/` and the path to this `.blob` file should be specified in the `depthai_models.yaml` file. For more information about these files, see the code structure outline above. This will publish `CVObject` messages to a topic for each class that the model detects, unaltered rgb preview frames that were input to the neural network, and rgb preview frames with bounding boxes, classes, and confidence values overlaid.

## Non-DepthAI Cameras

### USB Camera
This package also contains driver code to publish a camera stream from a USB-type camera in `usb_camera.py`. A USB camera can be located by `/dev/video*` on a linux computer, where `*` can be replaced by any number specifying a given camera channel. The default channel is `0`. Each camera may provide multiple channels; however, typically, only the first channel can be used by OpenCV to capture the camera stream. If multiple cameras are plugged in, the channels are enumerated in the order in which the cameras are plugged in. For example, channels `0-3` correspond to one camera, while channels `4-7` correspond to another camera.

The script `usb_camera.py` uses OpenCV to capture a stream frame by frame from a specified USB camera channel and publishes it to a specified ROS topic. Use `ros2 launch cv usb_camera.xml` to start a stream once a USB camera has been plugged in. You can specify the camera to connect to via:
```bash
ros2 launch cv usb_camera.xml camera:=<camera>
```

By default, `<camera>` is set to `front`. Note that the camera must be plugged in _before_ the Docker container is started.

The `udev` rules located at the repository root in the file `99-robosub-ros2.rules` are used to symlink the first channel provided by each of the front and bottom cameras to `/dev/video_front` and `/dev/video_bottom` respectively. This is done to ensure that the cameras are always accessible at the same path, regardless of the channel they are plugged into.

### Running the Code

To start up a CV node, run the following command:

```bash
ros2 launch cv <file_name>.xml
```

where `<file_name>` refers to the file name one is trying to launch (for more details about the files, see the section titled [Structure](#Structure)).

No further arguments are required in the command line -- additional parameters can be modified in the `.xml` files and `config.py`.

## Topics

### Subscribed

 * `/camera/<camera>/compressed`
   * The topic that the camera publishes each frame to
   * If no actual camera feed is available, you can use a bag file. See the Wiki for more details.
   * Type: `sensor_msgs/CompressedImage`

### Published

* `cv/<camera>/<object>/<stage>`
  * For each camera frame feed that a model processes, it will publish predictions to this topic.
  * `<camera>` corresponds to the camera being used (e.g., `bottom`)
  * `<object>` corresponds to the object being detected (e.g., `bin_blue`)
  * `<stage>` corresponds to the stage of the pipeline that is being published (`hsv_filtered`, `bounding_box`, etc.)
  * For DepthAI pipelines, for each detected object in a frame, the model will publish the `xmin`, `ymin`, `xmax`, and `ymax`
    coordinates (normalized to \[0, 1\], with (0, 0) being the top-left corner), `label` of the object, `score` (a confidence value in the range
    of \[0, 1\]), and the `width` and `height` of the frame.
  * If a model is enabled but detects no objects in a frame, it will not publish any messages that indicate a detection. Topics like `hsv_filtered` will still be published to for the purpose of debugging HSV color bounds.
  * Type: `custom_msgs/CVObject`

Note that the camera feed frame rate will likely be greater than the rate at which predictions can
be generated (especially if more than one model is enabled at the same time), so the publishing rate
could be anywhere from like 0.2 to 10 FPS depending on computing power/the GPU/other factors.

## Other Files

The `utils.py` file contains the `DetectionVisualizer` class which provides functions to draw bounding boxes and their labels onto images. It is used by DepthAI files when publishing visualized detections.

The `image_tools.py` file contains the `ImageTools` class which provides functions to convert between OpenCV, ROS Image, and ROS CompressedImage formats. Scripts in this package use `ImageTools` to perform conversions between these types. `cv_bridge` is not used by any file or class in this package other than `ImageTools` itself.
