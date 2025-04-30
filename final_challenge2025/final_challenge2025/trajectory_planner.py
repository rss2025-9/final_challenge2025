import rclpy
from rclpy.node import Node

assert rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, PoseArray
from nav_msgs.msg import OccupancyGrid
from .utils import LineTrajectory

import numpy as np
import heapq
import math
import time

# from scipy.ndimage import binary_dilation
from scipy.ndimage import distance_transform_edt

class PathPlan(Node):
    """ Listens for goal pose published by RViz and uses it to plan a path from
    current car pose.
    """

    def __init__(self):
        super().__init__("trajectory_planner")
        self.declare_parameter('odom_topic', "default")
        self.declare_parameter('map_topic', "default")
        self.declare_parameter('initial_pose_topic', "default")
        self.declare_parameter('buffer_meters', 1.0)

        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.map_topic = self.get_parameter('map_topic').get_parameter_value().string_value
        self.initial_pose_topic = self.get_parameter('initial_pose_topic').get_parameter_value().string_value
        self.buffer_meters = self.get_parameter('buffer_meters').get_parameter_value().double_value

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_cb,
            1)

        self.goal_sub = self.create_subscription(
            PoseStamped,
            "/goal_pose",
            self.goal_cb,
            10
        )

        self.traj_pub = self.create_publisher(
            PoseArray,
            "/trajectory/current",
            10
        )

        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.initial_pose_topic,
            self.pose_cb,
            10
        )

        self.trajectory = LineTrajectory(node=self, viz_namespace="/planned_trajectory")

        self.map = None
        self.start_pose = None
        x = 25.900000
        y = 48.50000
        theta = 3.14
        self.transform = np.array([[np.cos(theta), -np.sin(theta), x],
                    [np.sin(theta), np.cos(theta), y],
                    [0,0,1]])

        self.get_logger().info("Path planner initialized")

    # def add_margin_to_walls(self, map: np.ndarray, margin: int):
    #     structure = np.ones((2 * margin + 1, 2 * margin + 1), dtype=bool)
    #     return binary_dilation(map, structure=structure).astype(int)

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

    def pose_cb(self, pose: PoseWithCovarianceStamped):
        """Sets initial pose"""
        self.start_pose = pose.pose


    def goal_cb(self, msg: PoseStamped):
        """Sets goal pose"""
        self.goal_pose = msg
        if self.start_pose is None:
            self.get_logger().warn("Start pose is not set!")
            return
        if self.map is None:
            self.get_logger().warn("Map not found!")
            return
        
        self.plan_path(self.start_pose, self.goal_pose, self.map)

    def plan_path(self, start_point, end_point, map):

        start_time = time.time()
        # In world coordinates
        start_x = start_point.pose.position.x
        start_y = start_point.pose.position.y
        end_x = end_point.pose.position.x
        end_y = end_point.pose.position.y

        start_map = self.world_to_map(start_x, start_y) 
        end_map = self.world_to_map(end_x, end_y)
        self.get_logger().info(f"Start grid: {start_map}, Map value: {self.map[start_map[1], start_map[0]]}")
        self.get_logger().info(f"Goal grid: {end_map}, Map value: {self.map[end_map[1], end_map[0]]}")

        path = self.a_star_search(start_map, end_map, map)  # returns (y, x)
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


def main(args=None):
    rclpy.init(args=args)
    planner = PathPlan()
    rclpy.spin(planner)
    rclpy.shutdown()