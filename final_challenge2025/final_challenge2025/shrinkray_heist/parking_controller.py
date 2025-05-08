#!/usr/bin/env python

import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.time import Time
from rcl_interfaces.msg import SetParametersResult
from vs_msgs.msg import ConeLocation, ParkingError
from ackermann_msgs.msg import AckermannDriveStamped

class ParkingController(Node):
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):
        super().__init__("parking_controller")
        #! @brief Parameters for the parking controller.
        # Set in launch file; different for simulator vs racecar
        self.declare_parameter("drive_topic", "/vesc/low_level/input/navigation")
        # Boundary for when the car is considered pointed in the right direction.
        self.declare_parameter("anglular_error_threshold", np.radians(30))
        # Boundaries for when the car is considered too close/too far from the 
        # cone for angle adjustment manuevers (negative is too close).
        self.declare_parameter("distance_error_thresholds", [-1.0, 1.0])
        # How fast we want the car to go while parking
        self.declare_parameter("parking_velocity", 0.8)
        # How far we want to park from the location.
        self.declare_parameter("parking_distance", 0.1)

        #! @brief Gets the parameters for the parking controller.
        DRIVE_TOPIC: str = self.get_parameter(
            "drive_topic").get_parameter_value().string_value
        self.angular_error_threshold: float = self.get_parameter(
            "anglular_error_threshold").get_parameter_value().double_value
        self.distance_error_threshold: float = self.get_parameter(
            "distance_error_thresholds").get_parameter_value().double_array_value
        self.parking_velocity: float = self.get_parameter(
            "parking_velocity").get_parameter_value().double_value
        self.parking_distance: float = self.get_parameter(
            "parking_distance").get_parameter_value().double_value

        #! @brief PID variables for the controller.
        # PID constants for the controller.
        self.declare_parameters(
            namespace="pid",
            parameters=[
                ("kp", 1.0),
                ("ki", 0.0),
                ("kd", 0.5)
            ]
        )
        self.kp: float = self.get_parameter("pid.kp").get_parameter_value().double_value
        self.ki: float = self.get_parameter("pid.ki").get_parameter_value().double_value
        self.kd: float = self.get_parameter("pid.kd").get_parameter_value().double_value
        # PID variables for the controller.
        self.previous_x_error: float = 0 
        self.previous_time: float = Time()
        self.x_error_integral: float = 0
        # Integral error clipping for the PID controller to prevent windup.
        self.integral_bounds: tuple[float, float] = (-10.0, 10.0)

        # Publishers for the drive command and error between the car and parking
        # location.
        self.drive_pub = self.create_publisher(AckermannDriveStamped, DRIVE_TOPIC, 10)
        self.error_pub = self.create_publisher(ParkingError, "/parking_error", 10)

        # Subscriber for the relative cone location.
        self.create_subscription(ConeLocation, "/relative_cone", 
            self.relative_cone_callback, 1)

        # Log that the controller has been initialized.
        self.get_logger().info("Parking Controller Initialized")

    def relative_cone_callback(self, msg: ConeLocation) -> None:
        """
        Callback for the relative cone location. The parking controller will
        use this information to coordinate a parking maneuver to park in front
        of the cone.
        
        Args:
            msg:    The relative location of the cone.
        Postcondition:
            A drive command is published to the drive topic.
            A parking error is published to the error topic.
            PID variables are updated.
        """
        # Gets dt for the PID controller.
        current_time: float = self.get_clock().now()
        dt: float = (current_time.nanoseconds - self.previous_time.nanoseconds) / 10e9

        #! @note: We want to be right in front of the cone, so we want an angle
        #! of 0 (aka angle is also the angle error).
        # Current angle of the cone relative to front of the car.
        angle_error: float = np.arctan2(msg.y_pos, msg.x_pos)
        # Gets the error of the x position of the car from the cone.
        x_error: float = msg.x_pos - self.parking_distance

        # Updates PID values.
        self.x_error_integral: float = np.clip(
            self.x_error_integral + dt * self.previous_x_error, 
            *self.integral_bounds
        )
        self.previous_x_error = x_error
        self.previous_time = current_time

        # Checks if the distance error is large enough to require reversing.
        reverse: bool = (x_error <= self.distance_error_threshold[0] and
            not x_error >= self.distance_error_threshold[1])
        
        # Car is pointed in the wrong direction. Reversing for 3 point parking 
        # type maneuvers.
        heading: float = -angle_error if reverse else angle_error
        
        # Detects of the car is too far out of alignment with the cone.
        high_angular_error: bool = (np.abs(angle_error) > self.angular_error_threshold)
        velocity: float = 0
        if not high_angular_error and (
            self.distance_error_threshold[0] < x_error < self.distance_error_threshold[1]
        ):
            velocity = np.clip(self.parking_velocity * (
                self.kp * x_error +
                self.ki * self.x_error_integral +
                self.kd * (x_error - self.previous_x_error) / dt
            ), -self.parking_velocity, self.parking_velocity)
            heading /= 2        
        else:
            velocity = -self.parking_velocity if reverse else self.parking_velocity

        # Writes items into the drive command and publishes it.
        drive_cmd: AckermannDriveStamped = AckermannDriveStamped()
        drive_cmd.header.frame_id: str = "base_link"
        drive_cmd.header.stamp: float = current_time.to_msg()
        drive_cmd.drive.steering_angle: float = heading
        drive_cmd.drive.speed: float = velocity
        self.drive_pub.publish(drive_cmd)

        # Publishes the error between the car and the cone for rqt_plot.
        error_msg = ParkingError()
        error_msg.x_error: float = msg.x_pos - self.parking_distance
        error_msg.y_error: float = msg.y_pos
        error_msg.distance_error: float = np.hypot(error_msg.x_error, error_msg.y_error)
        self.error_pub.publish(error_msg)


    def parameters_callback(self, params):
        for param in params:
            if param.name == 'parking_velocity':
                self.parking_velocity = param.value
                self.get_logger().info(f"Updated parking velocity to {self.parking_velocity}")
            elif param.name == 'parking_distance':
                self.parking_distance = param.value
                self.get_logger().info(f"Updated parking distance to {self.parking_distance}")
        return SetParametersResult(successful=True)


def main(args=None):
    rclpy.init(args=args)
    pc = ParkingController()
    rclpy.spin(pc)
    rclpy.shutdown()

if __name__ == '__main__':
    main()