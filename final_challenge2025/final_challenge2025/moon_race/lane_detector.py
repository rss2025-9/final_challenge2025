#!/usr/bin/env python

import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge

import numpy as np

from sensor_msgs.msg import Image
from geometry_msgs.msg import Pose #geometry_msgs not in CMake file
from final_interfaces.msg import TrajInfo

# import color segmentation algorithm; call this function in ros_image_callback
from .color_segmentation import cd_color_segmentation


class LaneDetector(Node):
    """
    A class for applying your cone detection algorithms to the real robot.
    Subscribes to: /zed/zed_node/rgb/image_rect_color (Image) : the live RGB image from the onboard ZED camera.
    Publishes to: /relative_lane_px (ConeLocationPixel) : the coordinates of the center of the lane in the image frame (units are pixels).
    """
    def __init__(self):
        super().__init__("lane_detector")

        # Subscribe to ZED camera RGB frames
        self.lane_pub = self.create_publisher(TrajInfo, "/relative_lane_px", 10)
        self.debug_pub = self.create_publisher(Image, "/lane_debug_img", 10)
        self.image_sub = self.create_subscription(Image, "/zed/zed_node/rgb/image_rect_color", self.image_callback, 5)
        self.bridge = CvBridge() # Converts between ROS images and OpenCV Images

        self.get_logger().info("Lane Detector Initialized")

    def image_callback(self, image_msg: Image):
        # Apply the imported color segmentation function (cd_color_segmentation) to the image msg
        # From the two lines of the lane, get the center trajectory
        # (We know this pixel corresponds to a point on the ground plane)
        # publish this pixel (u, v) to the /relative_lane_px topic; the homography transformer will
        # convert it to the car frame.

        #################################
        # detect the lane and publish its center's
        # pixel location in the image.
        # vvvvvvvvvvvvvvvvvvvvvvvvvvvvvv
        #################################

        # convert the ROS image to OpenCV image
        image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")

        # get the bounding box from color_segmentation.py
        lines, crop_y_start, result = cd_color_segmentation(image, None)

        # ensure the lane is detected
        if not lines:
            self.get_logger().warn("No lanes detected!")

        height, width = image.shape[:2]
        img_center_x = width // 2

        left_lines = []
        right_lines = []

        for line in lines:
            x1, y1, x2, y2 = line[0]
            y1 += crop_y_start
            y2 += crop_y_start

            # visualize filtered lines in debugging image
            pt1 = (int(x1), int(y1))
            pt2 = (int(x2), int(y2))
            cv2.line(image, pt1, pt2, color=(255, 0, 0), thickness=2)  # blue lines

            # middle x-coordinate of the line
            mid_x = (x1 + x2) // 2

            # classify lines on left and right
            if mid_x < img_center_x:
                left_lines.append([[x1, y1, x2, y2]])
            else:
                right_lines.append([[x1, y1, x2, y2]])
            
        # get inner left and right lines
        left_inner_line = self.choose_inner(left_lines, "left")[0]
        right_inner_line = self.choose_inner(right_lines, "right")[0]

        if left_inner_line:
            self.draw_line(image, left_inner_line, (0, 255, 255))

        if right_inner_line:
            self.draw_line(image, right_inner_line, (0, 255, 255))

        left_start = np.array(left_inner_line[0:2])
        left_end = np.array(left_inner_line[2:4])

        right_start = np.array(right_inner_line[0:2])
        right_end = np.array(right_inner_line[2:4])

        # get equally placed points from left and right lines
        num_points = 10     # can tune this value
        center_points = []

        for i in range(num_points):
            t = i / (num_points - 1)    # ranges from 0 to 1
            # Calculate the left and right line points
            left_pt = left_start + t * (left_end - left_start)
            right_pt = right_start + t * (right_end - right_start)

            # get the center point of the left and right lines
            center = (left_pt + right_pt) / 2
            center_points.append(center)

        # get the x coordinate of the bottom of the center line
        bottom_center = max(center_points, key=lambda p: p[1])
        bottom_center_x = bottom_center[0]
        deviation = float(img_center_x - bottom_center_x)

        # publish the center points array in pixel
        center_poses = TrajInfo()
        center_poses.header.stamp = self.get_clock().now().to_msg()
        center_poses.header.frame_id = "zed_left_camera_frame"

        for (u, v) in center_points:
            pose = Pose()
            pose.position.x = float(u)
            pose.position.y = float(v)
            pose.position.z = 0.0
            center_poses.poses.append(pose)

        center_poses.deviation = deviation

        self.lane_pub.publish(center_poses)

        # publish debugging image
        for (u, v) in center_points:
            cv2.circle(image, (int(u), int(v)), radius=4, color=(0, 0, 255), thickness=-1)
        debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self.debug_pub.publish(debug_msg)

    # helper function to choose the inner line of the track
    def choose_inner(self, lines, side):
            if not lines:
                return None
            # choose the line with maximum mid_x for left lines
            if side == "left":
                return max(lines, key=lambda l: (l[0][0] + l[0][2]) / 2)
            # choose the line with minimum mid_x for right lines
            if side == "right":
                return min(lines, key=lambda l: (l[0][0] + l[0][2]) / 2)
            
    # helper function to draw line
    def draw_line(self, image, line, color):
        x1, y1, x2, y2 = line
        cv2.line(image, (int(x1), int(y1)), (int(x2), int(y2)), color, thickness=2)

def main(args=None):
    rclpy.init(args=args)
    lane_detector = LaneDetector()
    rclpy.spin(lane_detector)
    rclpy.shutdown()

if __name__ == '__main__':
    main()