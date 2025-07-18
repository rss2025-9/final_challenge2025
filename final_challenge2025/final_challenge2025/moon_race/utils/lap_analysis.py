# Installs rclpy and other dependencies.
import rclpy
from rclpy.node import Node
from rclpy.time import Time

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
# Matplotlib
import matplotlib.pyplot as plt

import threading

class LapAnalysis(Node):
    """ Implements lap analysis with odom and trajectory data.
    """

    def __init__(self):
        super().__init__("lap_analysis")
        # Topics to be used.
        self.declare_parameter('odom_topic', "/vesc/odom")
        self.odom_topic: str = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.declare_parameter('wheelbase_length', 0.3302)
        self.wheelbase_length: float = self.get_parameter('wheelbase_length').get_parameter_value().double_value
        self.declare_parameter('lookahead', 1.5)
        self.lookahead: float = self.get_parameter('lookahead').get_parameter_value().double_value

        # Saves the last odom time.
        self.last_odom_time = None
        # Gives the starting odom time.
        self.start_odom_time = None

        # Subscribers to the planned path and publishers for the drive command.
        self.trajectory: LineTrajectory = LineTrajectory("/followed_trajectory")
        self.deviation = 0.0
        self.deviation_data: list[float] = []
        self.time_data: list[float] = []
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

    def real_time_kinematics(self, msg):
        """
        Evolve the stored trajectory by moving the vehicle forward by
        drive_distance along its x‑axis and rotating the frame by the
        small yaw change from the bicycle model.
        """
        if self.last_odom_time is None:
            self.start_odom_time = Time.from_msg(msg.header.stamp)
            self.last_odom_time = Time.from_msg(msg.header.stamp)
            return
        delta_time = (Time.from_msg(msg.header.stamp) - self.last_odom_time).nanoseconds / 1e9
        self.last_odom_time = Time.from_msg(msg.header.stamp)

        # Get the speed and steering angle from the odometry message.
        speed: float = np.linalg.norm([msg.twist.twist.linear.x, msg.twist.twist.linear.y])
        yaw: float = euler_from_quaternion(
            [msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z,
             msg.pose.pose.orientation.w]
        )[2]

        # how far we move in this timestep
        drive_distance = speed * delta_time

        with self.trajectory_lock:
            if not self.initialized_traj:
                # If no trajectory is initialized, stop the vehicle.
                self.get_logger().warning("No trajectory initialized, stopping.")
                return
            # translate every point backward by drive_distance along the x‑axis
            # since points are in the vehicle frame
            self.traj_pts[:, 0] -= drive_distance
            # rotation matrix for a frame rotation of -delta_yaw:
            c, s = np.cos(yaw), np.sin(yaw)
            R = np.array([
                [c, -s],
                [s, c]
            ])
            # rotate all points into the new vehicle frame
            self.traj_pts = self.traj_pts @ R
            # Rotates the trajectory vector into the new vehicle frame.
            self.traj_vec = self.traj_vec @ R
            # Recalculates the deviation from the trajectory as the y coordinate
            # at x = 0.
            t = -self.traj_pts[0, 0] / self.traj_vec[0]
            self.deviation = self.traj_pts[0, 1] + t * self.traj_vec[1]
            # Publishes the deviation for visualization.
            deviation_msg = Float32()
            deviation_msg.data = self.deviation
            self.deviation_pub.publish(deviation_msg)

            self.time_data.append(
                msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            )
            self.deviation_data.append(self.deviation)

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
                # Publishes the deviation for visualization.
                self.time_data.append(
                    msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
                )
                self.deviation_data.append(self.deviation)
        
        self.deviation_pub.publish(deviation_msg)
        # Publish the trajectory for visualization.
        self.trajectory.publish_viz(duration=0.0)


def main(args=None):
    rclpy.init(args=args)
    node = LapAnalysis()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # --- plot deviation over time ---
        if node.time_data and node.deviation_data:
            # clips node times to between 630 and 700 seconds
            start_time = node.start_odom_time.nanoseconds / 1e9 + 10
            end_time = node.start_odom_time.nanoseconds / 1e9 + 80

            # Makes sure the length of the time and deviation data is the same.
            if len(node.time_data) != len(node.deviation_data):
                min_length = min(len(node.time_data), len(node.deviation_data))
                node.time_data = node.time_data[:min_length]
                node.deviation_data = node.deviation_data[:min_length]
            
            times = []
            deviations = []
            print(node.time_data)
            print(start_time, end_time)
            for t, d in zip(node.time_data, node.deviation_data):
                if start_time <= t <= end_time:
                    times.append(t)
                    deviations.append(d)
            # Gets a max deviation
            max_deviation = max(deviation for deviation in deviations if 25 > deviation > 0)
            # Gets a min deviation
            min_deviation = min(deviation for deviation in deviations if 0 > deviation > -25)
            # Gets the average deviation
            avg_deviation = sum(deviation for deviation in deviations if 25 > deviation > -25) / len([deviation for deviation in deviations if 1 > deviation > -1])
            # Prints the deviation statistics
            print(f"Max deviation: {max_deviation:.2f} m")
            print(f"Min deviation: {min_deviation:.2f} m")
            print(f"Avg deviation: {avg_deviation:.2f} m")
            
            plt.figure()
            plt.plot(times, deviations)
            plt.xlabel("Time since start (s)")
            plt.ylabel("Lateral deviation (m)")
            plt.title("Trajectory Deviation Over Time")
            plt.grid(True)
            plt.tight_layout()
            plt.savefig("deviation_over_time.png")