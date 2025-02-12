# Computer Vision

The computer vision package listens for images/frames coming from multiple cameras. The package
will then run pre-trained machine learning models or an HSV filtering algorithm on frames from each camera and output bounding boxes for the various objects
in the frames. These objects could be the gate, buoys, etc. The package will publish to different topics depending
on which classes are being detected and which cameras are being used.

## Structure

The following are the folders and files in the CV package:

`assets`: Additional assets (usually images) that the scripts may use.

`config`: Configuration files that contain information about how various parts of the pipeline are configured.

`cv`: This is the "meat" of our package that contains all of the files that connects to the camera and/or performs detection.
* `bin_detector.py`: Detects bins using HSV filtering.
* `blue_rectangle_detector.py`: Detects blue rectangles using HSV filtering.
* `buoy_detector_contour_matching`: Detects buoy using HSV filtering.
* `config.py`: Constants.
* `correct.py`: Auxiliary functions that correct any distortion.
* `depthai_camera_connect.py`: Connects to the OAK camera and uploads the image pipeline. Used by all other DepthAI scripts.
* `depthai_mono_detection.py`: Publishes detections using a specified model in `depthai_models.yaml` on a mono camera image feed.
* `depthai_publish_save_streams.py`: Publishes a preview of the image feed from the OAK camera and saves encoded streams. This can be used to verify connection to the camera and to check if there are any issues with the camera feed.
* `depthai_spatial_detection.py`: Publishes live detections using a specified model in `depthai_models.yaml`.
* `image_tools.py`: Auxiliary functions to convert ROS image messages to other types and vice versa.
* `path_marker_detector.py`: Detects path marker using HSV filtering.
* `pink_bins_detector.py`: Detects pink bins using HSV filtering.
* `usb_camera_connect.py`: Connects and opens USB cameras.
* `usb_camera.py`: Publishes images coming from USB cameras.
* `utils.py`: Auxiliary functions that visualize detections on an image feed and modularize certain repeated calculations such as distance.

`launch`: Contains the various launch files for our CV package. There are specific launch files for each script in `cv`.
* `bin_detector.xml`: Runs the bin detector script.
* `blue_rectangle_detector`: Runs the blue rectangle detector script.
* `buoy_detector_contour_matching`: Runs the buoy detector contour matching script.
* `depthai_camera_connect`: Runs the DepthAI camera connect script.
* `depthai_mono_detection`: Runs the DepthAI mono detection script.
* `depthai_publish_save_streams.xml`: Runs the DepthAI publish and save streams script.
* `depthai_spatial_detection.xml`: Runs the DepthAI spatial detection script.
* `path_marker_detector.xml`: Runs the path marker detector script.
* `pink_bins_detector.xml`: Runs the pink bins detector script.
* `usb_camera_connect.xml`: Runs the USB camera connect script.
* `usb_camera.xml`: Runs the USB camera script.

`models`: Contains our pre-trained models and a `.yaml` file that specifies the details of each model (classes predicted, topic name, and the path to the model weights).
* `depthai_models.yaml`: contains models for object detection. A model is specified by a name, what classes it predicts, and the path to a .blob file, as well as other configuration parameters. `input_size` is [width, height]. The blob file format is specific to the processors that the OAK cameras use.

`CMakeLists.txt`: A text file stating the necessary package dependencies and the files in our package.

`package.xml`: A xml file stating the basic information about the CV package.

The CV package also has dependencies in the `core/catkin_ws/src/custom_msgs` folder.

## DepthAI Cameras
This package contains code for the Luxonis OAK-D PoE camera, which uses a python package called [depthai](https://docs.luxonis.com/en/latest/). This camera handles neural network and image processing on the camera's processor, which necessitates a different code structure. Because of this, we have DepthAI-specific scripts and launch files that can be run for any Luxonis / DepthAI camera. For running other cameras, see the section below, titled [Non-DepthAI Cameras](#non-depthai-cameras).

### Setup

Generally, you would train a separate object detection model for each task you need computer vision for (gates, buoys, etc.). You can then load them as follows:

* Create object detection models and save them as .pth files (see [here](https://github.com/DukeRobotics/documentation/tree/master/cv/training))
* Place these models in the `/models` folder
* Update the `/models/models.yaml` file with each model's details in the following format:

```yaml
<model_name>:  # A name/identifier for your model
  classes: [<class1>, <class2>, ...]  # The classes the model is trained to predict
  topic: <topic_name>  # set to /cv by default; change if you want to specify model in publisher topics .etc
  weights: <file_name>  # the name of your model file
...
```

Example entry for a buoy model:

```yaml
buoy:
  classes: [alien, bat, witch]
  topic: /cv
  weights: buoy_model.pth
```

Note: To get the model files onto the Docker container, you may have to use `scp`. Also, if you come across the following error:

`URLError: <urlopen error [Errno -3] Temporary failure in name resolution>`

Navigate to [this url](https://download.pytorch.org/models/fasterrcnn_resnet50_fpn_coco-258fb6c6.pth)
to manually download the default model file used by the Detecto package. Move this file onto the Docker
container under the directory `/root/.cache/torch/checkpoints/` (do not rename the file).

### Running the Code
To stream the feed or perform spatial detection using the OAK camera, use `ros2 launch cv <module>` with the following files as needed.
* `depthai_publish_save_streams.xml`: Streams the live feed from the camera. You can choose what to publish from the camera (rgb video, rgb preview, left mono, right mono, disparity map, and depth map) by setting the appropriate boolean parameters. Also encodes the live feeds and saves them to files. You can choose what to save by setting the appropriate boolean parameters. You can also choose to automatically convert the encoded streams to video files.
* `depthai_spatial_detection.xml`: Connects to the camera and runs spatial detection. This requires a valid `.blob` file in `models/` and the path to this `.blob` file should be specified in the `depthai_models.yaml` file. For more information about these files, see the code structure outline above. This will publish `CVObject` messages to a topic for each class that the model detects, unaltered rgb preview frames that were input to the neural network, and rgb preview frames with bounding boxes, classes, and confidence values overlaid.

## Non-DepthAI Cameras

### USB Camera
This package also contains driver code to publish a camera stream from a USB-type camera in `usb_camera.py`. A USB camera can be located by `/dev/video*` on a linux computer, where `*` can be replaced by any number specifying a given camera channel. The default channel is `0`. Each camera may provide multiple channels, however, typically, only the first channel can be used by OpenCV to capture the camera stream. If multiple cameras are plugged in, the channels are enumerated in the order in which the cameras are plugged in. For example, channels `0-3` correspond to one camera, while channels `4-7` correspond to another camera.

The script `usb_camera.py` uses OpenCV to capture a stream frame by frame from a specified USB camera channel and publishes it to a specified ROS topic. Use `ros2 launch cv usb_camera.xml` to start a stream once a USB camera has been plugged in. You can specify the ROS topic to which the USB camera feed is published via

```bash
ros2 launch cv usb_camera.xml topic:=<topic>
```

By default, `<topic>` is set to `/camera/usb_camera/compressed`. Note that the camera must be plugged in _before_ the Docker container is started.

The `udev` rules located at the repository root in the rile `99-robosub-ros2.rules` are used to symlink the first channel provided by each of the front and bottom cameras to `/dev/video_front` and `/dev/video_bottom` respectively. This is done to ensure that the cameras are always accessible at the same path, regardless of the channel they are plugged into. The `usb_camera_connect.py` script is used to connect to both of these cameras simultaneously, and can be launched using `ros2 launch cv usb_camera_connect.xml`.

### Running the Code

To start up a CV node, run the following command:

```bash
ros2 launch cv <file_name>.xml
```

where `<file_name>` refers to the file name one is trying to launch (for more details about the files, see the section titled [Structure](#Structure)).

No further arugments are required in the command line -- additional parameters can be modified in the `.xml` files and `config.py`.

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

The `image_tools.py` file contains the `ImageTools` class which provides functions to convert between OpenCV, ROS Image, and ROS CompressedImage formats. All scripts in this package use `ImageTools` to perform conversions between these types. `cv_bridge` is not used by any file or class in this package other than `ImageTools` itself.
