#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np

import cv2

from visualization_msgs.msg import Marker
from final_interfaces.msg import LineBounds, WorldTrajInfo
from geometry_msgs.msg import Pose, Point

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
#                    [344, 179],
#                    # dummy points
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

# PTS_IMAGE_PLANE = [[323, 211], 
#                     [370, 314], 
#                     [18, 294], 
#                     [465, 209], 
#                     [42, 18.5],
#                     [466, 310], 
#                     [411, 355]]
# PTS_GROUND_PLANE = [[42, 0], 
#                     [14, -2.25], 
#                     [16, 51], 
#                     [42, 18.5],
#                     [134, 0],
#                     [12.5, -5.5], 
#                     [20.5, -5.5]]

METERS_PER_INCH = 0.0254

class HomographyTransformer(Node):
    def __init__(self):
        super().__init__("homography_transformer")
        self.extension = 12
        self.correction = 1.0

        self.lane_pub = self.create_publisher(WorldTrajInfo, "/trajectory/midpoint", 10)
        self.marker_pub = self.create_publisher(Marker, "/lane_marker", 1)
        self.lane_px_sub = self.create_subscription(LineBounds, "/relative_lane_px", self.lane_detection_callback, 1)

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

    def lane_detection_callback(self, msg: LineBounds):

        # Create world trajectory message
        relative_traj = WorldTrajInfo()
        relative_traj.header.stamp = msg.header.stamp
        relative_traj.header.frame_id = "zed_left_camera_frame"

        # Extract information from message
        left_start = self.transformUvToXy(msg.left.start.x, msg.left.start.y)
        left_end = self.transformUvToXy(msg.left.end.x, msg.left.end.y)
        right_start = self.transformUvToXy(msg.right.start.x, msg.right.start.y)
        right_end = self.transformUvToXy(msg.right.end.x, msg.right.end.y)

        # Averages the start and end points to create a midline.
        mid_start = (left_start + right_start) / 2
        mid_end = (left_end + right_end) / 2
        # Extends the midline to extension.
        mid_vec = mid_end - mid_start
        mid_vec /= np.linalg.norm(mid_vec)
        mid_end = mid_start + self.extension * mid_vec

        mid_start[0] += self.correction
        mid_end[0] += self.correction

        turn_side = msg.turn_side

        relative_traj.poses.extend([
            Pose(
                position=Point(
                    x=mid_start[0],
                    y=mid_start[1]
                )
            ),
            Pose(
                position=Point(
                    x=mid_end[0],
                    y=mid_end[1]
                )
            )
        ])
        relative_traj.turn_side = turn_side
        relative_traj.deviation = mid_start[1]
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
        return np.array([x, y], dtype=float)

    def draw_marker(self, x, y, message_frame):
        """
        Publish a marker to represent the waypoint in rviz.
        (Call this function if you want)
        """
        marker = Marker()
        marker.header.frame_id = message_frame
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = .2
        marker.scale.y = .2
        marker.scale.z = .2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = .5
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = x
        marker.pose.position.y = y
        self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    homography_transformer = HomographyTransformer()
    rclpy.spin(homography_transformer)
    rclpy.shutdown()

if __name__ == "__main__":
    main()