# Installs rclpy and other dependencies.
import rclpy
from rclpy.node import Node

# Imports WorldTrajInfo custom message.
from final_interfaces.msg import WorldTrajInfo
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

from tf_transformations import euler_from_quaternion
from final_challenge2025.utils import LineTrajectory

# Numpy
import numpy as np
import numpy.typing as npt

import threading

class LapAnalysis(Node):
    """ Implements lap analysis with odom and trajectory data.
    """

    def __init__(self):
        super().__init__("lap_analysis")
        # Topics to be used.
        self.declare_parameter('odom_topic', "/vesc/odom")
        self.odom_topic: str = self.get_parameter('odom_topic').get_parameter_value().string_value

        # Saves the last odom time.
        self.last_odom_time = None

        # Subscribers to the planned path and publishers for the drive command.
        self.trajectory: LineTrajectory = LineTrajectory("/followed_trajectory")
        self.deviation = 0.0
        self.initialized_traj = False
        # The trajectory points that are RTK'd are stored here.
        self.traj_pts: npt.NDArray[np.float64] = np.zeros((0, 2))
        # The trajectory vector is stored here.
        self.traj_vec: npt.NDArray[np.float64] = np.zeros((0, 2))
        # Lock for the trajectory updates.
        self.trajectory_lock = threading.Lock()

        self.midpoint_sub = self.create_subscription(
            WorldTrajInfo, "/trajectory/midpoint",
            self.trajectory_callback, 1
        )
        self.odom_sub = self.create_subscription(
            Odometry, self.odom_topic,
            self.real_time_kinematics, 1
        )
        self.deviation_pub = self.create_publisher(
            Float32, "/trajectory/deviation", 1
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

    def real_time_kinematics(self, msg):
        """
        Evolve the stored trajectory by moving the vehicle forward by
        drive_distance along its x‑axis and rotating the frame by the
        small yaw change from the bicycle model.
        """
        if self.last_odom_time is None:
            self.last_odom_time = msg.header.stamp
            return
        delta_time = (msg.header.stamp - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = msg.header.stamp

        # Get the speed and steering angle from the odometry message.
        speed: float = np.linalg.norm([msg.twist.twist.linear.x, msg.twist.twist.linear.y])
        steering_angle: float = euler_from_quaternion(
            [msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z,
             msg.pose.pose.orientation.w]
        )[2]

        # how far we move in this timestep
        drive_distance = speed * delta_time
        # yaw change = v/L * tan(delta) * dt
        delta_yaw = drive_distance * np.tan(steering_angle) / self.wheelbase_length

        with self.trajectory_lock:
            # translate every point backward by drive_distance along the x‑axis
            # since points are in the vehicle frame
            self.traj_pts[:, 0] -= drive_distance
            # rotation matrix for a frame rotation of -delta_yaw:
            c, s = np.cos(delta_yaw), np.sin(delta_yaw)
            R = np.array([
                [c, -s],
                [s, c]
            ])
            # rotate all points into the new vehicle frame
            self.traj_pts = self.traj_pts @ R
            # Rotates the trajectory vector into the new vehicle frame.
            self.traj_vec = self.traj_vec @ R
            # Recalculates the deviation from the trajectory.
            self.deviation = np.mean(self.traj_pts[:, 1])
            # Publishes the deviation for visualization.
            deviation_msg = Float32()
            deviation_msg.data = self.deviation
            self.deviation_pub.publish(deviation_msg)

    def trajectory_callback(self, msg: WorldTrajInfo):
        """
        Sets a new trajectory to follow.
        """
        deviation_msg = Float32()
        with self.trajectory_lock:
            self.turn_side = msg.turn_side
            if msg.turn_side != "straight":
                # If the trajectory is not straight, stop the vehicle.
                self.get_logger().warning("Trajectory is not straight, extrapolating.")
                # Stops if no trajectory is initialized.
                if not self.initialized_traj:
                    self.get_logger().warning("No trajectory initialized, stopping.")
                    self.publish_drive_cmd(0.0, 0.0)
                    return
                # Otherwise, extrapolate the trajectory.
                else:
                    # Extends the trajectory by the lookahead distance.
                    self.traj_pts[1] += self.traj_vec * self.lookahead
            else:
                self.trajectory.clear()
                # Converts from poses to the utility trajectory class.
                self.trajectory.fromPoseArray(msg)
                self.traj_pts: npt.NDArray[np.float64] = np.array(self.trajectory.points)
                self.traj_vec: npt.NDArray[np.float64] = self.traj_pts[1] - self.traj_pts[0]
                self.traj_vec /= np.linalg.norm(self.traj_vec)
                # Notes the deviation from the trajectory.
                self.deviation = msg.deviation
                # flag to check that we have a trajectory.
                self.initialized_traj = True
                # Publishes the deviation for visualization.
                deviation_msg.data = self.deviation
        
        self.deviation_pub.publish(deviation_msg)
        # Publish the trajectory for visualization.
        self.trajectory.publish_viz(duration=0.0)


def main(args=None):
    rclpy.init(args=args)
    follower = PurePursuit()
    rclpy.spin(follower)
    rclpy.shutdown()