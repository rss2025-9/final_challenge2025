import rclpy
from rclpy.node import Node
# Driving.
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
# Geometry.
from geometry_msgs.msg import Pose, PoseArray
from tf_transformations import euler_from_quaternion

import numpy as np
import numpy.typing as npt

from .utils import LineTrajectory


class PurePursuit(Node):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """

    def __init__(self):
        super().__init__("trajectory_follower")
        # Topics to be used.
        self.declare_parameter('odom_topic', "default")
        self.declare_parameter('drive_topic', "default")

        self.odom_topic: str = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.drive_topic: str = self.get_parameter('drive_topic').get_parameter_value().string_value

        # Pure Pursuit parameters.
        self.declare_parameter('lookahead', 1.2)
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('wheelbase_length', 0.3302)

        self.lookahead: float = self.get_parameter('lookahead').get_parameter_value().double_value
        self.speed: float = self.get_parameter('speed').get_parameter_value().double_value
        self.wheelbase_length: float = self.get_parameter('wheelbase_length').get_parameter_value().double_value

        # Subscribers to the planned path and publishers for the drive command.
        self.trajectory: LineTrajectory = LineTrajectory("/followed_trajectory")
        self.initialized_traj = False

        self.traj_sub = self.create_subscription(
            PoseArray, "/trajectory/current",
            self.trajectory_callback, 1
        )
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic,
            self.pose_callback, 1
        )
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, self.drive_topic, 1
        )
    
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

    def pose_callback(self, odometry_msg: Odometry):
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
        if not self.initialized_traj:
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
        speed: float = max(self.speed * (1 - np.tanh(np.log(self.wheelbase_length * np.abs(gamma) + 1))), 1.0)
        # Publish the drive command.
        self.publish_drive_cmd(speed, steering_angle)

    def trajectory_callback(self, msg):
        self.get_logger().info(f"Receiving new trajectory {len(msg.poses)} points")

        self.trajectory.clear()
        # Converts from poses to the utility trajectory class.
        self.trajectory.fromPoseArray(msg)
        self.trajectory.publish_viz(duration=0.0)
        # flag to check that we have a trajectory.
        self.initialized_traj = True


def main(args=None):
    rclpy.init(args=args)
    follower = PurePursuit()
    rclpy.spin(follower)
    rclpy.shutdown()
