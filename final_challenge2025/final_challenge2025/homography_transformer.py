#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np

import cv2
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import String
from sensor_msgs.msg import Image
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from final_interfaces.msg import TrajInfo, WorldTrajInfo
from geometry_msgs.msg import Pose

#The following collection of pixel locations and corresponding relative
#ground plane locations are used to compute our homography matrix

# PTS_IMAGE_PLANE units are in pixels
# see README.md for coordinate frame description

######################################################
## DUMMY POINTS -- ENTER YOUR MEASUREMENTS HERE
# PTS_IMAGE_PLANE = [[442, 228],
#                    [127, 296],
#                    [268, 220],
#                    [347, 270],
#                    [323, 291], 
#                    [60, 348], 
#                    [256, 173], 
#                    [402, 173], 
#                    [553, 208], 
#                    [546, 258],
#                    [344, 179], # dummy points
######################################################

# PTS_GROUND_PLANE units are in inches
# car looks along positive x axis with positive y axis to left

######################################################
## DUMMY POINTS -- ENTER YOUR MEASUREMENTS HERE
# PTS_GROUND_PLANE = [[34, -12],
#                     [18, 12],
#                     [39, 7],
#                     [21, 0],
#                     [19, 3], 
#                     [15, 14],
#                     [96, 18],
#                     [96, -23], 
#                     [38, -29], 
#                     [23, -13],
#                     [118, -6], # dummy points
######################################################

######################################################
## DUMMY POINTS -- ENTER YOUR MEASUREMENTS HERE
PTS_IMAGE_PLANE = [[287, 312],
                   [466, 310],
                   [411, 255],
                   [292, 254]] # dummy points
######################################################

# PTS_GROUND_PLANE units are in inches
# car looks along positive x axis with positive y axis to left

######################################################
## DUMMY POINTS -- ENTER YOUR MEASUREMENTS HERE
PTS_GROUND_PLANE = [[12.5, 2],
                    [12.5, -5.5],
                    [20.5, -5.5],
                    [20.5, 2]] # dummy points
######################################################

METERS_PER_INCH = 0.0254

class HomographyTransformer(Node):
    def __init__(self):
        super().__init__("homography_transformer")

        self.lane_pub = self.create_publisher(WorldTrajInfo, "/trajectory/midpoint", 10)
        self.marker_pub = self.create_publisher(Marker, "/lane_marker", 1)
        self.lane_px_sub = self.create_subscription(TrajInfo, "/relative_lane_px", self.lane_detection_callback, 1)

        if not len(PTS_GROUND_PLANE) == len(PTS_IMAGE_PLANE):
            rclpy.logerr("ERROR: PTS_GROUND_PLANE and PTS_IMAGE_PLANE should be of same length")

        #Initialize data into a homography matrix

        np_pts_ground = np.array(PTS_GROUND_PLANE)
        np_pts_ground = np_pts_ground * METERS_PER_INCH
        np_pts_ground = np.float32(np_pts_ground[:, np.newaxis, :])

        np_pts_image = np.array(PTS_IMAGE_PLANE)
        np_pts_image = np_pts_image * 1.0
        np_pts_image = np.float32(np_pts_image[:, np.newaxis, :])

        self.h, err = cv2.findHomography(np_pts_image, np_pts_ground)

        self.get_logger().info("Homography Transformer Initialized")
        self.get_logger().info(f"Homography Matrix: \n {self.h}" )

    def lane_detection_callback(self, msg: TrajInfo):

        # Create world trajectory message
        relative_traj = WorldTrajInfo()
        relative_traj.header.stamp = self.get_clock().now().to_msg()
        relative_traj.header.frame_id = "zed_left_camera_frame"

        #Extract information from message
        center_poses_px = msg.poses

        #Call to main function for each point of the trajectory
        for center_pose_px in center_poses_px:
            u = center_pose_px.position.x
            v = center_pose_px.position.y
            x, y = self.transformUvToXy(u, v)

            #Publish relative xy position of trajectory in real world
            relative_xy_pose = Pose()
            relative_xy_pose.position.x = x
            relative_xy_pose.position.y = y
            relative_xy_pose.position.z = 0.0
            relative_traj.poses.append(relative_xy_pose)

        relative_traj.deviation = msg.deviation

        self.draw_marker(x, y, "zed_left_camera_frame")
        self.get_logger().info(f"relative lane positions in real world (meters): {x}, {y}")
        self.lane_pub.publish(relative_traj)

    def transformUvToXy(self, u, v):
        """
        u and v are pixel coordinates.
        The top left pixel is the origin, u axis increases to right, and v axis
        increases down.

        Returns a normal non-np 1x2 matrix of xy displacement vector from the
        camera to the point on the ground plane.
        Camera points along positive x axis and y axis increases to the left of
        the camera.

        Units are in meters.
        """
        homogeneous_point = np.array([[u], [v], [1]])
        xy = np.dot(self.h, homogeneous_point)
        scaling_factor = 1.0 / xy[2, 0]
        homogeneous_xy = xy * scaling_factor
        x = homogeneous_xy[0, 0]
        y = homogeneous_xy[1, 0]
        return x, y

    # def draw_marker(self, cone_x, cone_y, message_frame):
    #     """
    #     Publish a marker to represent the cone in rviz.
    #     (Call this function if you want)
    #     """
    #     marker = Marker()
    #     marker.header.frame_id = message_frame
    #     marker.type = marker.CYLINDER
    #     marker.action = marker.ADD
    #     marker.scale.x = .2
    #     marker.scale.y = .2
    #     marker.scale.z = .2
    #     marker.color.a = 1.0
    #     marker.color.r = 1.0
    #     marker.color.g = .5
    #     marker.pose.orientation.w = 1.0
    #     marker.pose.position.x = cone_x
    #     marker.pose.position.y = cone_y
    #     self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    homography_transformer = HomographyTransformer()
    rclpy.spin(homography_transformer)
    rclpy.shutdown()

if __name__ == "__main__":
    main()