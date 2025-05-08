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
from geometry_msgs.msg import PoseArray, PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String
from nav_msgs.msg import Odometry


PTS_IMAGE_PLANE = [[287, 312],
                   [466, 310],
                   [411, 255],
                   [292, 254]] 

PTS_GROUND_PLANE = [[12.5, 2],
                    [12.5, -5.5],
                    [20.5, -5.5],
                    [20.5, 2]]

METERS_PER_INCH = 0.0254

np_pts_ground = np.array(PTS_GROUND_PLANE)
np_pts_ground = np_pts_ground * METERS_PER_INCH
np_pts_ground = np.float32(np_pts_ground[:, np.newaxis, :])

np_pts_image = np.array(PTS_IMAGE_PLANE)
np_pts_image = np_pts_image * 1.0
np_pts_image = np.float32(np_pts_image[:, np.newaxis, :])

h, err = cv2.findHomography(np_pts_image, np_pts_ground)

class DetectorNode(Node):
    def __init__(self):
        super().__init__("detector")
        self.detector = Detector()
        self.subscriber = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.callback, 1)
        self.state_sub = self.create_subscription(String, '/state', self.state_callback, 1)
        self.create_subscription(Odometry, '/pf/pose/odom', self.odom_cb, 1)

        self.bridge = CvBridge()

        self.states_publisher = self.create_publisher(DetectionStates, "/detector/states", 1)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/drive", 1)
        self.publisher = self.create_publisher(Image, "/detector/annotated_img", 1)
        self.debug_pub = self.create_publisher(Image, "/detector/debug_img", 1)

        self.start_publish = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 1)
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 1)

        self.get_logger().info("Detector Initialized")
        self.state_callback_msg = String()

        self.state = None 
        self.curr_pos = None 

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

    def state_callback(self, msg):
        self.state = msg.data

    def odom_cb(self, msg: Odometry):
        self.curr_pos = msg

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
                stop_mask = cv2.inRange(hsv, (120, 220, 160), (179, 255, 255))  # hue, saturation, value
                red_count = cv2.countNonZero(stop_mask)
                # yellow
                my = cv2.inRange(hsv, (60, 220, 160), (119, 255, 255))
                yellow_count = cv2.countNonZero(my)
                # green
                mg = cv2.inRange(hsv, (0, 220, 160), (59, 255, 255))
                green_count = cv2.countNonZero(mg)

                debug_msg = self.bridge.cv2_to_imgmsg(stop_mask+my+mg, "mono8")
                self.debug_pub.publish(debug_msg)
                if red_count > 0.035 * area:
                    msg.traffic_light_state = 'RED'
                    self.get_logger().info('RED TRAFFIC LIGHT!')
                elif yellow_count > 0.035 * area:
                    msg.traffic_light_state = 'YELLOW'
                    self.get_logger().info('YELLOW TRAFFIC LIGHT!')
                elif green_count > 0.035 * area:
                    msg.traffic_light_state = 'GREEN'
                    self.get_logger().info('GREEN TRAFFIC LIGHT!')
                else:
                    msg.traffic_light_state = 'NONE'

            elif self.state == "HeistState.SCOUT" and label == 'banana':
                # hsv = cv2.cvtColor(region, cv2.COLOR_BGR2HSV)
                # area = region.shape[0] * region.shape[1]
                # m = cv2.inRange(hsv, (20, 50, 50), (30, 255, 255))
                # yellow_count = cv2.countNonZero(m)
                # if yellow_count > 0.1 * area:
                    # self.banana_counter += 1
                self.banana_counter += 1
                if self.banana_counter >= 2: 
                    msg.banana_state = 'DETECTED'

                    # get banana pixel coordinates of the center and apply homography 
                    u = (x1 + x2) / 2
                    v = (y1 + y2) / 2
                    x, y = self.transform_uv_to_xy(u, v)
                    self.get_logger().info(f"Banana in world: x={x:.2f}, y={y:.2f}")
                    park_x = x - 0.7
                    park_y = y 

                    start_pose = PoseWithCovarianceStamped()
                    start_pose.header.frame_id ='map'
                    start_pose.header.stamp = self.get_clock().now().to_msg()
                    start_pose.pose.pose = self.curr_pos.pose.pose
                    self.start_publish.publish(start_pose)

                    pose = PoseStamped()
                    pose.header.frame_id = "map"
                    pose.header.stamp = self.get_clock().now().to_msg()
                    pose.pose.position.x = park_x
                    pose.pose.position.y = park_y
                    self.goal_pub.publish(pose)

                    self.get_logger().info(f"Parking in front of banana")

                    # Save the image with the banana    
                    save_path = f"{os.path.dirname(__file__)}/banana_output.png"
                    out.save(save_path)
                    self.banana_counter = 0

            elif label == 'person':
                msg.person_state = 'DETECTED'

        return msg
    
    def transform_uv_to_xy(self, u, v):
        homogeneous_point = np.array([[u], [v], [1]])
        xy = np.dot(self.homography_matrix, homogeneous_point)
        scaling_factor = 1.0 / xy[2, 0]
        homogeneous_xy = xy * scaling_factor
        x = homogeneous_xy[0, 0]
        y = homogeneous_xy[1, 0]
        return np.array([x, y], dtype=float)

def main(args=None):
    rclpy.init(args=args)
    detector = DetectorNode()
    rclpy.spin(detector)
    rclpy.shutdown()

if __name__=="__main__":
    main()
