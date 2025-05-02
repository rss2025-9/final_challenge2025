#!/usr/bin/env python

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge

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

        # get equally placed points from left and right lines
        num_points = 10     # can tune this value
        left_points = []
        right_points = []

        for i in range(num_points):
            t = i / (num_points - 1)    # ranges from 0 to 1
            x_left = (1 - t) * left_inner_line[0][0] + t * left_inner_line[0][2]    # linear interpolation
            y_left = (1 - t) * left_inner_line[0][1] + t * left_inner_line[0][3]
            left_points.append((x_left, y_left))

            x_right = (1 - t) * right_inner_line[0][0] + t * right_inner_line[0][2]
            y_right = (1 - t) * right_inner_line[0][1] + t * right_inner_line[0][3]
            right_points.append((x_right, y_right))
        
        center_points = []
        # get an array of center points
        for (xl, yl), (xr, yr) in zip(left_points, right_points):
            x_center = (xl + xr) / 2
            y_center = (yl + yr) / 2
            center_points.append((x_center, y_center))

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
        debug_msg = self.bridge.cv2_to_imgmsg(image, "bgr8")
        self.debug_pub.publish(debug_msg)

    # helper function to choose the inner line of the track
    def choose_inner(lines, side):
            if not lines:
                return None
            # choose the line with maximum mid_x for left lines
            if side == "left":
                return max(lines, key=lambda l: (l[0][0] + l[0][2]) // 2)
            # choose the line with minimum mid_x for right lines
            if side == "right":
                return min(lines, key=lambda l: (l[0][0] + l[0][2]) // 2)
            

def main(args=None):
    rclpy.init(args=args)
    lane_detector = LaneDetector()
    rclpy.spin(lane_detector)
    rclpy.shutdown()

if __name__ == '__main__':
    main()