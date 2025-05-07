import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from .detector import Detector
import os 
import numpy as np
import cv2
from final_interfaces.msg import DetectionStates
from ackermann_msgs.msg import AckermannDriveStamped


class DetectorNode(Node):
    def __init__(self):
        super().__init__("detector")
        self.detector = Detector()
        self.publisher = self.create_publisher(Image, "/detector/annotated_img", 1)
        self.subscriber = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.callback, 1)
        self.bridge = CvBridge()

        self.states_publisher = self.create_publisher(DetectionStates, "/detector/states", 1)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/drive", 1)

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
        
        self.banana_counter = 0

        detection_msg = self.check_states(image, predictions, out)
        self.states_publisher.publish(detection_msg)

    def check_states(self, frame, preds, out):
        msg = DetectionStates()
        msg.traffic_light_state = 'NONE'
        msg.banana_state = 'NONE'
        msg.person_state = 'NONE'
        for (x1, y1, x2, y2), label in preds:
            x1, y1, x2, y2 = map(int, (x1, y1, x2, y2))
            region = frame[y1:y2, x1:x2, :]
            if region.size == 0:
                continue

            if label == 'traffic light':
                self.get_logger().info("traffic light detected ahead")
                hsv = cv2.cvtColor(region, cv2.COLOR_BGR2HSV)
                area = region.shape[0] * region.shape[1]
                # red
                mr = cv2.inRange(hsv, (0, 100, 100), (15, 225, 225))      # hue, saturation, value
                red_count = cv2.countNonZero(mr)
                # green
                mg = cv2.inRange(hsv, (40, 100, 100), (80, 225, 225))
                green_count = cv2.countNonZero(mg)
                if red_count > 0.01 * area:
                    msg.traffic_light_state = 'RED'
                    self.get_logger().info('RED TRAFFIC LIGHT!')
                elif green_count > 0.01 * area:
                    msg.traffic_light_state = 'GREEN'
                    self.get_logger().info('GREEN TRAFFIC LIGHT!')
                else:
                    msg.traffic_light_state = 'NONE'

            elif label == 'banana':
                # hsv = cv2.cvtColor(region, cv2.COLOR_BGR2HSV)
                # area = region.shape[0] * region.shape[1]
                # m = cv2.inRange(hsv, (20, 50, 50), (30, 255, 255))
                # yellow_count = cv2.countNonZero(m)
                # if yellow_count > 0.1 * area:
                    # self.banana_counter += 1
                self.banana_counter += 1
                if self.banana_counter >= 1: 
                    self.banana_counter = 0
                    msg.banana_state = 'DETECTED'
                    # Save the image with the banana    
                    save_path = f"{os.path.dirname(__file__)}/banana_output.png"
                    out.save(save_path)
                    print(f"Saved banana pic to {save_path}!")

            elif label == 'person':
                msg.person_state = 'DETECTED'

        return msg

def main(args=None):
    rclpy.init(args=args)
    detector = DetectorNode()
    rclpy.spin(detector)
    rclpy.shutdown()

if __name__=="__main__":
    main()
