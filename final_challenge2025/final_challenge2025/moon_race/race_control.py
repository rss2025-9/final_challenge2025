import threading

import rclpy
from rclpy.node import Node
# Driving.
from ackermann_msgs.msg import AckermannDriveStamped

# Numpy
import numpy as np
import numpy.typing as npt

from .utils import LineTrajectory

# Imports TrajInfo custom message.
from final_interfaces.msg import TrajInfo

class PurePursuit(Node):
    """ Implements Pure Pursuit trajectory tracking with a fixed lookahead and speed.
    """

    def __init__(self):
        super().__init__("trajectory_follower")
        # Topics to be used.
        self.declare_parameter('drive_topic', "default")
        self.drive_topic: str = self.get_parameter('drive_topic').get_parameter_value().string_value

        # Pure Pursuit parameters.
        self.declare_parameter('lookahead', 1.2)
        self.declare_parameter('speed', 1.0)
        self.declare_parameter('wheelbase_length', 0.3302)

        self.lookahead: float = self.get_parameter('lookahead').get_parameter_value().double_value
        self.speed: float = self.get_parameter('speed').get_parameter_value().double_value
        self.wheelbase_length: float = self.get_parameter('wheelbase_length').get_parameter_value().double_value
        
        # Lane following parameters.
        self.declare_parameter('lane_width', 0.5)
        self.declare_parameter('lane_buffer', 0.1)

        self.lane_width: float = self.get_parameter('lane_width').get_parameter_value().double_value
        self.lane_buffer: float = self.get_parameter('lane_buffer').get_parameter_value().double_value

        # Derived parameters.
        self.max_deviation: float = (self.lane_width / 2) - self.lane_buffer
        assert self.max_deviation > 0, "Lane buffer is too large for the lane width."

        # Refresh rate of the controller.
        self.declare_parameter('refresh_rate', 0.001)
        self.refresh_rate: float = self.get_parameter('refresh_rate').get_parameter_value().double_value
        # Timer to call the timer callback at a fixed rate.
        self.timer = self.create_timer(self.refresh_rate, self.timer_callback)

        # Subscribers to the planned path and publishers for the drive command.
        self.trajectory: LineTrajectory = LineTrajectory("/followed_trajectory")
        self.deviation = 0.0
        self.initialized_traj = False

        self.midpoint_sub = self.create_subscription(
            TrajInfo, "/trajectory/midpoint",
            self.trajectory_callback, 1
        )
        self.drive_pub = self.create_publisher(
            AckermannDriveStamped, self.drive_topic, 1
        )
        # Lock for the trajectory updates.
        self.trajectory_lock = threading.Lock()
    
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

    def real_time_kinematics(self, steering_angle: float, speed: float):
        """
        Employs real time kinematics to evolve the trajectory with commanded motion.
        """
        # Applies the translation to the trajectory.
        translation_vector: npt.NDArray = np.array([
            np.cos(steering_angle),
            np.sin(steering_angle)
        ])
        # Applies the rotation to the trajectory.
        rotation_matrix: npt.NDArray = np.array([
            [np.cos(steering_angle), -np.sin(steering_angle)],
            [np.sin(steering_angle), np.cos(steering_angle)]
        ])
        ## TODO:: May need to apply adaptive time evolution.
        # Applies the translation and rotation to the trajectory.
        with self.trajectory_lock:
            drive_distance: float = speed * self.refresh_rate
            self.trajectory.points -= drive_distance * translation_vector
            self.trajectory.points = np.dot(
                self.trajectory.points, rotation_matrix.T
            )

    def timer_callback(self):
        """
        Takes the current position of the robot, finds the nearest point on the
        path, sets that as the goal point, and navigates towards it.
        """
        # Moves only if the trajectory is initialized, otherwise publish stop.
        if not self.initialized_traj:
            self.publish_drive_cmd(0.0, 0.0)
            return
        
        # Calculates the distance to all points in the trajectory, vectorized.
        distances: npt.NDArray = np.linalg.norm(self.trajectory.points, axis=1)
        # Calculates whether the points are ahead of the vehicle.
        forwards: npt.NDArray[np.bool] = self.trajectory.points[:, 0] > 0
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
            if closest_idx == len(self.trajectory.points) - 1:
                self.get_logger().warning("Last point in trajectory reached, stopping.")
                self.publish_drive_cmd(0.0, 0.0)
                return
            # Otherwise, set the goal point to the closest point.
            goal_point = self.get_trajectory(
                closest_idx, self.trajectory.points, distances
            )
        elif distances[closest_idx] >= self.lookahead:
            # If the closest point is beyond the lookahead distance, interpolate
            # between the car and it to find the goal point.
            goal_point = self.trajectory.points[closest_idx] * \
                         (distances[closest_idx] / self.lookahead)
        else:
            # Find the first point that is within the lookahead distance.
            goal_point = self.get_trajectory(
                closest_idx, self.trajectory.points, distances
            )

        # If the deviation is significant, turn harder to the side.
        if np.abs(self.deviation) > self.max_deviation:
            # Gets how far into the red we are from the safe lane zone.
            deviation_delta: float = np.abs(self.deviation) - self.max_deviation
            # Feeds the deviation delta into a tanh function to get a bounded correction.
            correction: float = np.tanh((
                np.sign(self.deviation) * deviation_delta
            ) / self.max_deviation)
            # Applies the correction to the goal point.
            goal_point[1] += correction
            # Renormalizes to the distance of the lookahead.
            goal_point *= self.lookahead / np.linalg.norm(goal_point)

        # Calculate the curvature 
        gamma: float = 2 * goal_point[1] / (self.lookahead ** 2)

        # Calculate the steering angle.
        steering_angle: float = np.arctan(gamma * self.wheelbase_length)
        # Calculates the speed proportional to the gamma.
        speed: float = max(self.speed * (
            1 - np.tanh(np.log(self.wheelbase_length * np.abs(gamma) + 1))
        ), 1.0)
        # Publish the drive command.
        self.publish_drive_cmd(speed, steering_angle)

        # Update the trajectory given the commanded motion.
        self.real_time_kinematics(steering_angle, speed)

    def trajectory_callback(self, msg):
        """
        Sets a new trajectory to follow.
        """
        with self.trajectory_lock:
            self.get_logger().info(f"Receiving new trajectory {len(msg.poses)} points")

            self.trajectory.clear()
            # Converts from poses to the utility trajectory class.
            self.trajectory.fromPoseArray(msg)
            self.trajectory.publish_viz(duration=0.0)
            # Notes the deviation from the trajectory.
            self.deviation = msg.deviation
            # flag to check that we have a trajectory.
            self.initialized_traj = True


def main(args=None):
    rclpy.init(args=args)
    follower = PurePursuit()
    rclpy.spin(follower)
    rclpy.shutdown()