#!/usr/bin/env python

import rclpy
from rclpy.node import Node

import cv2
from cv_bridge import CvBridge

import numpy as np

from sensor_msgs.msg import Image
from final_interfaces.msg import Pixel

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
        self.intersection_pub = self.create_publisher(Pixel, "/goal_px", 10)
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

        height, width = image.shape[:2]
        img_center_x = width // 2

        overlap = width // 10   # 10% overlap (tune this value if needed)

        left_img = image[:, :width // 2 + overlap]
        right_img = image[:, width // 2 - overlap:]

        # get left and right lines from color_segmentation.py
        left_lines_raw, right_lines_raw = None, None
        # Sorts the point in each line so vectors always point up.
        sort_lines = lambda raw: [[line[0][0], line[0][1], line[0][2], line[0][3]] if line[0][1] >= line[0][3] 
                                    else [line[0][2], line[0][3], line[0][0], line[0][1]]
                                    for line in raw]
        try:
            left_lines_raw, crop_y_start, _ = cd_color_segmentation(left_img, None, None)
        except TypeError:
            self.get_logger().warning(f"Missing lines on the left")
            return
        if left_lines_raw != None:
            left_lines_raw = sort_lines(left_lines_raw)
            # For lines going left, dx should be positive
            left_lines_raw = [line for line in left_lines_raw if (line[2] - line[0]) >= 0]

        try:
            right_lines_raw, crop_y_start, _ = cd_color_segmentation(right_img, None, None)
        except TypeError as e:
            self.get_logger().warning(f"Missing lines on the right {e}")
            return
        if right_lines_raw != None:
            right_lines_raw = sort_lines(right_lines_raw)
            # For lines going right, dx should be negative
            right_lines_raw = [line for line in right_lines_raw if (line[2] - line[0]) <= 0]

        # ensure the lane is detected
        if not left_lines_raw and not right_lines_raw:
            self.get_logger().warn("No lanes detected!")
            turn_side = "straight"
        elif not left_lines_raw and right_lines_raw:
            turn_side = "left"
        elif left_lines_raw and not right_lines_raw:
            turn_side = "right"
        else:
            turn_side = "straight"

        # offset right lines to align with the full image
        offset_x_right = width // 2 - overlap
        for line in right_lines_raw:
            line[0] += offset_x_right    # x1
            line[2] += offset_x_right    # x2

        left_lines = []
        right_lines = []

        for line in left_lines_raw + right_lines_raw:
            x1, y1, x2, y2 = line
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
        left_inner_line = self.choose_inner(left_lines, "left")
        right_inner_line = self.choose_inner(right_lines, "right")

        if left_inner_line is None or right_inner_line is None:
            return
        left_inner_line = left_inner_line[0]
        right_inner_line = right_inner_line[0]

        if left_inner_line:
            self.draw_line(image, left_inner_line, (0, 255, 255))

        if right_inner_line:
            self.draw_line(image, right_inner_line, (0, 255, 255))

        # get the start and end points of the left and right lines
        left_start = np.array(left_inner_line[0:2], dtype=float)
        left_end = np.array(left_inner_line[2:4], dtype=float)
        right_start = np.array(right_inner_line[0:2], dtype=float)
        right_end = np.array(right_inner_line[2:4], dtype=float)
        
        # Gets the vector intersection of the two lines, which from point perspective
        # will lead to something that is collinear with a point in the line center.
        left_vec = left_end - left_start
        right_vec = right_end - right_start

        delta = left_vec[0] * right_vec[1] - left_vec[1] * right_vec[0]
        if delta == 0:
            self.get_logger().warn("Lines are parallel!")
            return
        lane_width_px = right_start - left_start
        t = (lane_width_px[0] * right_vec[1] - lane_width_px[1] * right_vec[0]) / delta

        # publish the center points array in pixel
        intersection = Pixel()
        intersection.header.stamp = self.get_clock().now().to_msg()
        intersection.header.frame_id = "zed_left_camera_frame"
        intersection.x = left_start[0] + t * left_vec[0]
        intersection.y = left_start[1] + t * left_vec[1]
        intersection.turn_side = turn_side
        self.intersection_pub.publish(intersection)

        # Prints a debug image for detection.
        debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self.debug_pub.publish(debug_msg)

    # helper function to choose the inner line of the track
    def choose_inner(self, lines, side):
        if not lines:
            self.get_logger().warning(f"### No line on {side} ###")
            return
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