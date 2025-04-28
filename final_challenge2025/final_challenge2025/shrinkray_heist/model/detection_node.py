import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from .detector import Detector
import os 
import numpy as np
import cv2

class DetectorNode(Node):
    def __init__(self):
        super().__init__("detector")
        self.detector = Detector()
        self.publisher = self.create_publisher(Image, "/detector/annotated_img", 1)
        self.subscriber = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.callback, 1)
        self.bridge = CvBridge()

        self.get_logger().info("Detector Initialized")

    def callback(self, img_msg):
        # Process image with CV Bridge
        image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")

        # Run yolo on this image 
        results = self.detector.predict(image)
        predictions = results["predictions"]
        original_image = results["original_image"]

        out = self.detector.draw_box(original_image, predictions, draw_all=True)

        rgb_np = np.array(out)
        bgr_np = cv2.cvtColor(rgb_np, cv2.COLOR_RGB2BGR)
        ros_img = self.bridge.cv2_to_imgmsg(bgr_np, "bgr8")
        ros_img.header = img_msg.header

        # Publish the image
        self.publisher.publish(ros_img)

def main(args=None):
    rclpy.init(args=args)
    detector = DetectorNode()
    rclpy.spin(detector)
    rclpy.shutdown()

if __name__=="__main__":
    main()
