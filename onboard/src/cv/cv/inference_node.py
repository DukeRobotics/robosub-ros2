import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String

from cv_bridge import CvBridge
import torch
import cv2
import numpy as np
import sys, os

sys.path.append(os.path.join(os.path.dirname(__file__), 'yolov7'))

from yolov7.models.experimental import attempt_load
from yolov7.utils.general import non_max_suppression

class YoloDetectorNode(Node):
    def __init__(self):
        super().__init__("yolov7_detector")

        # Load model
        weights = "/home/ubuntu/robosub-ros2/onboard/src/cv/cv/yolov7/yolov7_tiny.pt"
        self.device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
        self.model = attempt_load(weights, map_location=self.device)
        self.model.eval()
        self.names = self.model.names if hasattr(self.model, 'names') \
                                      else self.model.module.names

        # CV bridge
        self.bridge = CvBridge()

        # Subscribers
        self.subscription = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_callback,
            10
        )

        # Publishers
        self.pub_image = self.create_publisher(Image, "/detections/image", 10)
        self.pub_boxes = self.create_publisher(String, "/detections/boxes", 10)

        self.get_logger().info("YOLOv7 Detector Node Initialized!")

    def preprocess(self, img):
        """Resize → RGB → CHW → Normalize → Tensor."""
        img_resized = cv2.resize(img, (640, 640))

        img_rgb = img_resized[:, :, ::-1].transpose(2, 0, 1)
        img_rgb = np.ascontiguousarray(img_rgb)

        img_tensor = torch.from_numpy(img_rgb).to(self.device).float() / 255.0
        img_tensor = img_tensor.unsqueeze(0)
        return img_tensor, img_resized

    def draw_boxes(self, image, detections):
        img_draw = image.copy()
        for det in detections:
            if len(det):
                for *xyxy, conf, cls in reversed(det):
                    x1, y1, x2, y2 = [int(x.item()) for x in xyxy]
                    label = f"{self.names[int(cls)]} {conf:.2f}"

                    cv2.rectangle(img_draw, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(img_draw, label, (x1, y1 - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                                (0, 255, 0), 2)
        return img_draw

    def image_callback(self, msg):
        """Runs each time a new image is received."""
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Preprocess
        img_tensor, img_resized = self.preprocess(cv_image)

        # YOLOv7 inference
        with torch.no_grad():
            pred = self.model(img_tensor)[0]
            pred = non_max_suppression(pred, conf_thres=0.25, iou_thres=0.45)

        # Draw bounding boxes
        img_out = self.draw_boxes(img_resized, pred)

        # Publish predictions
        det_list = []
        for det in pred:
            for *xyxy, conf, cls in det:
                det_list.append(
                    f"{self.names[int(cls)]},{conf:.2f},{int(xyxy[0])},{int(xyxy[1])},{int(xyxy[2])},{int(xyxy[3])}"
                )

        det_msg = String()
        det_msg.data = "\n".join(det_list)
        self.pub_boxes.publish(det_msg)

        # Publish image with bounding boxes
        img_msg_out = self.bridge.cv2_to_imgmsg(img_out, encoding="bgr8")
        self.pub_image.publish(img_msg_out)

        self.get_logger().info(f"Published {len(det_list)} detections.")

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
