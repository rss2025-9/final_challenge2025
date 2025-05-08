#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from enum import Enum, auto
import threading
import time
from rclpy.time import Time
import os

import numpy as np
import numpy.typing as npt
import heapq
import math
import cv2
from cv_bridge import CvBridge
from scipy.ndimage import distance_transform_edt

from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool, String
from tf_transformations import euler_from_quaternion

from final_interfaces.msg import DetectionStates
from ..utils import LineTrajectory
from .model.detector import Detector

# Homography setup
PTS_IMAGE_PLANE = [[287, 312], [466, 310], [411, 255], [292, 254]]
PTS_GROUND_PLANE = [[12.5, 2], [12.5, -5.5], [20.5, -5.5], [20.5, 2]]
METERS_PER_INCH = 0.0254
np_pts_ground = np.array(PTS_GROUND_PLANE)
np_pts_ground = np_pts_ground * METERS_PER_INCH
np_pts_ground = np.float32(np_pts_ground[:, np.newaxis, :])

np_pts_image = np.array(PTS_IMAGE_PLANE)
np_pts_image = np_pts_image * 1.0
np_pts_image = np.float32(np_pts_image[:, np.newaxis, :])

homography_matrix, _ = cv2.findHomography(np_pts_image, np_pts_ground)

class HeistState(Enum):
    IDLE = auto()
    PLAN_TRAJ = auto()
    FOLLOW_TRAJ = auto()
    SCOUT = auto()
    PICKUP = auto()
    ESCAPE = auto()

class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine')
        # -- Params --
        # General params 
        self.declare_parameter('initial_pose_topic', 'default')

        # path planner params
        self.declare_parameter('map_topic', 'default')
        self.declare_parameter('odom_topic', 'default')
        self.declare_parameter('buffer_meters', 1.0)

        # trajectory follower params 
        self.declare_parameter('drive_topic', '/drive')
        self.declare_parameter('lookahead', 1.2)
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('wheelbase_length', 0.3302)

        # detector params
        self.declare_parameter('image_topic', '/zed/zed_node/rgb/image_rect_color')

        # retrieve
        self.map_topic = self.get_parameter('map_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.initial_pose_topic = self.get_parameter('initial_pose_topic').value
        self.buffer_meters = self.get_parameter('buffer_meters').value
        self.drive_topic = self.get_parameter('drive_topic').value
        self.lookahead = self.get_parameter('lookahead').value
        self.speed = self.get_parameter('speed').value
        self.wheelbase_length = self.get_parameter('wheelbase_length').value
        self.image_topic = self.get_parameter('image_topic').value

        # -- State & data --
        self.detector = Detector()
        self.bridge = CvBridge()
        self.state = HeistState.IDLE
        self.start_pose = None
        self.odom_msg = None
        self.goals = []
        self.goal_idx = 0
        self.red_detected = False

        # -- pubs & subs --
        self.create_subscription(PoseWithCovarianceStamped, self.initial_pose_topic, self.initial_pose_cb, 1)
        self.create_subscription(Odometry, self.odom_topic, self.odom_cb, 1)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.drive_topic, 1)

        # basement point publisher subscriber 
        self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 1)

        # path planner pubs and subs and params
        self.create_subscription(OccupancyGrid, self.map_topic, self.map_cb, 1)
        self.trajectory = LineTrajectory(node=self, viz_namespace="/planned_trajectory")
        self.traj_pub = self.create_publisher(PoseArray, '/trajectory/current', 1)
        self.map = None
        self.start_pose = None
        x = 25.900000
        y = 48.50000
        theta = 3.14
        self.transform = np.array([[np.cos(theta), -np.sin(theta), x],
                    [np.sin(theta), np.cos(theta), y],
                    [0,0,1]])
        self.get_logger().info("path planner initialized")

        # banana parking 

        # detector pubs and subs and params 
        self.create_subscription(Image, self.image_topic, self.image_cb, 1)
        self.annot_pub = self.create_publisher(Image, '/detector/annotated_img', 1)
        self.debug_pub = self.create_publisher(Image, '/detector/debug_img', 1)
        self.detector_states = {"traffic light": 'NONE', "banana": 'NONE'} 
        self.get_logger().info("Detector Initialized")

        self.create_timer(0.1, self.tick)
        self.get_logger().info('State Machine Initialized')

    def initial_pose_cb(self, msg: PoseWithCovarianceStamped):
        """Callback for the initial pose of the robot"""
        self.start_pose = msg
        self.get_logger().info("Initial pose received.")
    
    def odom_cb(self, msg: Odometry):
        self.odom_msg = msg

    def goal_cb(self, msg: PoseStamped):
        """Callback for the goal pose of the robot"""
        self.goals.append(msg)
        self.get_logger().info(f"Goal added: {msg.pose.position.x}, {msg.pose.position.y}")

    # YOLO callback
    def image_cb(self, img_msg: Image):
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
        self.annot_pub.publish(ros_img)

        for (x1, y1, x2, y2), label in predictions:
            x1, y1, x2, y2 = map(int, (x1, y1, x2, y2))
            region = image[y1:y2, x1:x2, :]
            if region.size == 0:
                continue
            
            self.detector_states["traffic light"] = "NONE"
            self.detector_states["banana"] = "NONE"

            if label == 'traffic light':
                if y2 < image.shape[0]//3:
                    continue

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
                if red_count > 0.005 * area:
                    self.detector_states[label] = 'RED'
                    self.red_detected = True
                    self.get_logger().info('RED TRAFFIC LIGHT!')
                elif yellow_count > 0.035 * area:
                    self.detector_states[label] = 'YELLOW'
                    self.get_logger().info('YELLOW TRAFFIC LIGHT!')
                elif green_count > 0.035 * area:
                    self.detector_states[label] = 'GREEN'
                    self.red_detected = False
                    self.get_logger().info('GREEN TRAFFIC LIGHT!')
                else:
                    self.detector_states[label] = 'NONE'

            elif self.state == HeistState.SCOUT and label == 'banana': 
                self.detector_states[label] = 'DETECTED'
                self.get_logger().info(f"Banana detected")
                # get banana position and park 
                u = float(x1 + x2) / 2.0 
                v = float(y2)
                banana_x, banana_y = self.transformUvToXy(u, v)

                # Save the image with the banana    
                save_path = f"{os.path.dirname(__file__)}/banana_output.png"
                out.save(save_path)

                self.HeistState = HeistState.PICKUP

    def tick(self):
        self.get_logger().info(f"State: {self.state}")

        if self.state == HeistState.IDLE:
            if self.start_pose and len(self.goals) >= 2:
                self.goal_idx = 0
                self.state = HeistState.PLAN_TRAJ
        elif self.state == HeistState.PLAN_TRAJ:
            self.plan_path(self.start_pose.pose, self.goals[self.goal_idx])
            self.state = HeistState.FOLLOW_TRAJ
        elif self.state == HeistState.FOLLOW_TRAJ:
            if not self.red_detected and self.detector_states["traffic light"] == 'RED':
                self.red_detected = True
                self.get_logger().info("STOPPING due to red light.")
                self.publish_drive_cmd(0.0, 0.0)
                return

            if self.red_detected:
                if self.detector_states["traffic light"] in ('GREEN'):
                    self.red_detected = False
                    self.get_logger().info("Done waiting for traffic, proceeding.")
                else:
                    self.get_logger().info("STOPPING due to red light.")
                    self.publish_drive_cmd(0.0, 0.0)
                    return
            
            if self.odom_msg: 
                self.follow_trajectory(self.odom_msg)
        elif self.state == HeistState.SCOUT:
            
            self.publish_drive_cmd(-0.5, 0.0)
        elif self.state == HeistState.PICKUP:
            # park code --> wait 5 seconds and then go into plan trajectory for next banana or escape 
            pass 
        elif self.state == HeistState.ESCAPE:
            # escape code 
            pass

    # PLANNER FUNCTIONS # 
    def map_cb(self, msg: OccupancyGrid):
        """Takes the Occupancy Grid of the map and creates an internal representation"""
        # occupied_threshold = 0.65

        self.get_logger().info("Map received.")

        map_width = msg.info.width
        map_height = msg.info.height
        map_data = np.array(msg.data).reshape((map_height, map_width))
        self.map_resolution = msg.info.resolution
        self.map_origin = msg.info.origin.position

        # Mark the grid as 1 if its occupancy value is -1
        self.map = (map_data == -1).astype(int)

        # marginalize the walls so that we have some safety distance away from the walls
        free_space = (self.map == 0)
        self.distance_map = distance_transform_edt(free_space) * self.map_resolution

        self.get_logger().info(f"Map added.")

    def plan_path(self, start_point, end_point):
        # start_point --> PoseWithCovarianceStamped
        # end_point --> PoseStamped

        start_time = time.time()
        # In world coordinates
        start_x = start_point.pose.position.x
        start_y = start_point.pose.position.y
        end_x = end_point.pose.position.x
        end_y = end_point.pose.position.y

        start_map = self.world_to_map(start_x, start_y) 
        end_map = self.world_to_map(end_x, end_y)
        # self.get_logger().info(f"Start grid: {start_map}, Map value: {self.map[start_map[1], start_map[0]]}")
        # self.get_logger().info(f"Goal grid: {end_map}, Map value: {self.map[end_map[1], end_map[0]]}")

        path = self.a_star_search(start_map, end_map, self.map)  # returns (y, x) 
        # self.get_logger().info(f"Path: {path}")
        if path is None or len(path) == 0:
            self.get_logger().error("No path found!")
            return
        self.get_logger().info(f"Path found with {len(path)} waypoints.")

        elasped = time.time() - start_time
        self.get_logger().info(f"Path planning took {elasped:.3f} seconds")

        world_coords = []
        for (row, col) in path:
            world_xy = self.map_to_world(col, row)
            world_coords.append(world_xy)

        self.trajectory.clear()
        for point in world_coords:
            self.trajectory.addPoint(point)
        
        self.traj_pub.publish(self.trajectory.toPoseArray())
        self.trajectory.publish_viz()
    
    def world_to_map(self, x, y):
        """World to map index using transform matrix (inverse of the transform)."""
        point = np.array([x, y, 1.0])
        pixel = self.transform @ point
        pixel = pixel / self.map_resolution
        return int(pixel[1]), int(pixel[0])  # (row, col)
    
    def map_to_world(self, col, row):
        """Map index to world coordinates using inverse transform."""
        pixel = np.array([col * self.map_resolution, row * self.map_resolution, 1.0])
        point = np.linalg.inv(self.transform) @ pixel
        return float(point[0]), float(point[1])

    def heuristic(self, a, b):
        # Euclidean distance from a to b
        x = abs(a[0] - b[0])
        y = abs(a[1] - b[1])
        return math.hypot(x, y)
    
    def get_neighbors(self, map, node):
        (y, x) = node
        neighbors = []
        candidates = [(y+1, x), (y-1, x), (y, x+1), (y, x-1)]
        for candidate in candidates:
            # check if the candidate neighbor is out of the map
            if 0 <= candidate[0] < map.shape[0] and 0 <= candidate[1] < map.shape[1]:
                # add the candidate to neighbors list only if it has no obstacle
                if map[candidate[0], candidate[1]] == 0:
                    neighbors.append(candidate)
        # self.get_logger().info(f"Neighbors: {neighbors}")
        return neighbors
    
    def reconstruct_path(self, came_from, start_point, end_point):
        """Go backward from end point to start point to construct a path"""
        current = end_point
        path = []
        # self.get_logger().info(f"Came from: {came_from}")
        # return an empty path if no path is found
        if end_point not in came_from:
            return []
        while current != start_point:
            path.append(current)
            current = came_from[current]
        path.append(start_point)
        path.reverse()
        return path

    def a_star_search(self, start_point, end_point, map):
        frontier = []
        heapq.heappush(frontier, (0, start_point))

        came_from = {}  # to reconstruct the path (node: came_from node)
        c_score = {start_point: 0}
        f_score = {start_point: self.heuristic(start_point, end_point)}

        while frontier:
            current = heapq.heappop(frontier)[1]    # get the node with the lowest priority

            if current == end_point:
                self.get_logger().info(f"Current node: {current}")
                return self.reconstruct_path(came_from, start_point, current)
            
            for neighbor in self.get_neighbors(map, current):

                # add penalty depending on how close the node is to the obstacle
                distance_to_obstacle = self.distance_map[neighbor[0], neighbor[1]]
                if distance_to_obstacle < self.buffer_meters:  # safety threshold of 0.7 (tune this value)
                    penalty = (self.buffer_meters - distance_to_obstacle) * 10
                else:
                    penalty = 0

                new_c_cost = c_score[current] + 1 + penalty  # cost of moving to neighbor
                if neighbor not in c_score or new_c_cost < c_score[neighbor]:
                    c_score[neighbor] = new_c_cost
                    # f(x) = c(x) + h(x) from lecture
                    f_score[neighbor] = new_c_cost + self.heuristic(neighbor, end_point)
                    heapq.heappush(frontier, (f_score[neighbor], neighbor))
                    came_from[neighbor] = current

        return None   # no path found
    

    # TRAJECTORY FOLLOWING FUNCTIONS #
    def publish_drive_cmd(self, speed: float, steering_angle: float):
        """
        Publishes the drive command to the vehicle.
        """
        drive_cmd: AckermannDriveStamped = AckermannDriveStamped()
        drive_cmd.drive.speed = speed
        drive_cmd.drive.steering_angle = steering_angle
        drive_cmd.header.stamp = self.get_clock().now().to_msg()
        drive_cmd.header.frame_id = "base_link"
        self.drive_pub.publish(drive_cmd)
    
    def get_trajectory(
        self, closest_idx: int, 
        relative_positions: npt.NDArray[np.float64], 
        distances: npt.NDArray[np.float64]
    ) -> npt.NDArray:
        """
        Returns the trajectory points as a numpy array.
        """
        goal_point: npt.NDArray[np.float64] = None
        for i in range(closest_idx + 1, len(relative_positions)):
            if distances[i] >= self.lookahead:
                # Determines the unit vector of the trajectory.
                traj: npt.NDArray[np.float] = relative_positions[i] - relative_positions[i-1]
                traj /= np.linalg.norm(traj)
                # Finds the unit vector of the first point to the vehicle.
                p2v: np.float = relative_positions[i-1] / distances[i]
                # Dot product of traj and p2v, to determine distance projection.
                delta: npt.NDArray = np.dot(p2v, traj)
                # Parameterized equation of the line to find the goal point.
                t: np.float = -delta + np.sqrt(
                    delta ** 2 + self.lookahead ** 2 - 1
                )
                # Get the goal point + safety from square root.
                goal_point = (relative_positions[i-1] if np.isnan(t)
                                else relative_positions[i-1] + t * traj)
                break
        # If no points are found, use the last point.
        if goal_point is None:
            goal_point = relative_positions[-1]
        return goal_point
    
    def follow_trajectory(self, odometry_msg: Odometry):
        """
        Takes the current position of the robot, finds the nearest point on the
        path, sets that as the goal point, and navigates towards it.
        """
        # Gets vectorized Pose of the robot.
        pose: Pose = odometry_msg.pose.pose
        position: npt.NDArray = np.array([pose.position.x, pose.position.y])
        yaw: np.float64 = -euler_from_quaternion(
            [pose.orientation.x, pose.orientation.y,
             pose.orientation.z, pose.orientation.w]
        )[2]

        # Moves only if the trajectory is initialized, otherwise publish stop.
        if not self.trajectory:
            self.publish_drive_cmd(0.0, 0.0)
            return
        
        # Calculates the distance to all points in the trajectory, vectorized.
        relative_positions: npt.NDArray = np.array(
            self.trajectory.points
        ) - position  # vector from vehicle to each trajectory point
        distances: npt.NDArray = np.linalg.norm(relative_positions, axis=1)
        # Rotates relative positions to the vehicle's frame.
        rotation_matrix: npt.NDArray = np.array([
            [np.cos(yaw), -np.sin(yaw)],
            [np.sin(yaw), np.cos(yaw)]
        ])
        # Calculates whether the points are ahead of the vehicle.
        forwards: npt.NDArray[np.bool] = relative_positions @ rotation_matrix.T[:, 0] > 0
        # Replaces the distance of behind points with a large value.
        distances_ahead: npt.NDArray = np.where(forwards, distances, np.inf)
        # Finds the index of the closest point ahead.
        closest_idx: int = np.argmin(distances_ahead)

        # Only consider points with positive projection on the heading vector.
        goal_point: npt.NDArray[np.float64]
        # If no points are ahead.
        if not np.any(forwards):
            self.get_logger().warning("No points ahead of the vehicle.")
            # Get the closest point in the trajectory.
            closest_idx = np.argmin(distances)
            # If the closest point is the last one, stop.
            if closest_idx == len(relative_positions) - 1:
                self.get_logger().warning("Last point in trajectory reached, stopping.")
                self.publish_drive_cmd(0.0, 0.0)
                self.state = HeistState.SCOUT
                return
            # Otherwise, set the goal point to the closest point.
            goal_point = self.get_trajectory(
                closest_idx, relative_positions, distances
            )
        elif distances[closest_idx] >= self.lookahead:
            # If the closest point is beyond the lookahead distance, interpolate
            # between the car and it to find the goal point.
            goal_point = relative_positions[closest_idx] * \
                         (distances[closest_idx] / self.lookahead)
        else:
            # Find the first point that is within the lookahead distance.
            goal_point = self.get_trajectory(
                closest_idx, relative_positions, distances
            )

        # Rotates the goal point to the vehicle's frame.
        goal_point = goal_point @ rotation_matrix.T

        # Calculate the curvature 
        gamma: float = 2 * goal_point[1] / (self.lookahead ** 2)

        # Calculate the steering angle.
        steering_angle: float = np.arctan(gamma * self.wheelbase_length)
        # Calculates the speed proportional to the gamma.
        speed: float = max(self.speed * (1 - np.tanh(np.log(self.wheelbase_length * np.abs(gamma) + 1))), min(self.speed, 1.0))
        # Publish the drive command.
        self.publish_drive_cmd(speed, steering_angle)

    # Banana Parking 
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
        xy = np.dot(homography_matrix, homogeneous_point)
        scaling_factor = 1.0 / xy[2, 0]
        homogeneous_xy = xy * scaling_factor
        x = homogeneous_xy[0, 0]
        y = homogeneous_xy[1, 0]
        return np.array([x, y], dtype=float)
    

def main(args=None):
    rclpy.init(args=args)
    state_machine = StateMachine()
    rclpy.spin(state_machine)
    rclpy.shutdown()