import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from .detector import Detector
import os 
import numpy as np
import cv2
from std_msgs.msg import String

class DetectorNode(Node):
    def __init__(self):
        super().__init__("detector")
        self.detector = Detector()
        self.publisher = self.create_publisher(Image, "/detector/annotated_img", 1)
        self.subscriber = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.callback, 1)
        self.bridge = CvBridge()

        self.traffic_light_state = self.create_publisher(String, '/traffic_light/state', 1)
        self.banana_state = self.create_publisher(String, '/banana/state', 1)
        self.person_state = self.create_publisher(String, '/person/state', 1)

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
        
        # get traffic state 
        self.check_states(image, predictions)

    def check_states(self, frame, preds):
        for (x1, y1, x2, y2), label in preds:
            if label == 'traffic_light':
                x1, y1, x2, y2 = map(int, (x1, y1, x2, y2))
                roi = frame[y1:y2, x1:x2]
                if roi.size == 0:
                    continue
                hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                area = roi.shape[0] * roi.shape[1]
                # red mask (two ranges)
                m1 = cv2.inRange(hsv, (0,50,50), (10,255,255))
                m2 = cv2.inRange(hsv, (160,50,50), (180,255,255))
                red_count = int(cv2.countNonZero(m1) + cv2.countNonZero(m2))
                # green mask
                mg = cv2.inRange(hsv, (40,50,50), (90,255,255))
                green_count = int(cv2.countNonZero(mg))
                if red_count > 0.1 * area:
                    self.traffic_light_state.publish('RED')
                if green_count > 0.1 * area:
                    self.traffic_light_state.publish('GREEN')
            elif label == 'banana':
                x1, y1, x2, y2 = map(int, (x1, y1, x2, y2))
                roi = frame[y1:y2, x1:x2]
                if roi.size == 0:
                    continue
                hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                area = roi.shape[0] * roi.shape[1]
                # yellow mask
                m = cv2.inRange(hsv, (20,50,50), (30,255,255))
                yellow_count = int(cv2.countNonZero(m))
                if yellow_count > 0.1 * area:
                    self.banana_state.publish('YELLOW')
            elif label == 'person': 
                x1, y1, x2, y2 = map(int, (x1, y1, x2, y2))
                roi = frame[y1:y2, x1:x2]
                if roi.size == 0:
                    continue
                self.person_state.publish('DETECTED')
            else: 
                self.traffic_light_state.publish('NONE')
                self.banana_state.publish('NONE')
                self.person_state.publish('NONE')
        # if no detections, also publish NONE
        if not preds:
            self.traffic_light_state.publish('NONE')
            self.banana_state.publish('NONE')
            self.person_state.publish('NONE')

def main(args=None):
    rclpy.init(args=args)
    detector = DetectorNode()
    rclpy.spin(detector)
    rclpy.shutdown()

if __name__=="__main__":
    main()
