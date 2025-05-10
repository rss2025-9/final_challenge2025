#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
import numpy as np
import os
import csv

class SafetyController(Node):

    def __init__(self):
        super().__init__("safety_controller")

        self.stop_thresh_base = 0.4  # meters (base stopping threshold at very low speeds)
        self.stop_speed = 0.0  # stopping speed
        # self.min_speed = 1.0 # minimum speed for stopping - this is when we call publish stop command 
        # self.braking_speed = 0.5 # braking speed - how much to slow down gradually 
        self.current_speed = 0.0 # current speed updated based on drive msgs 
        self.current_steer = 0.0 # current angle based off drive msgs
        self.step_brake_speed = 0.3  # Slow down to this speed before stop (instead of hard stop immediately)
        self.hard_stop_speed = 0.0   # Full stop speed
        self.min_slowing_distance = 0.5  # distance to start slowing down (m)

        self.lookahead_angle = np.pi/36  #narrower cone: +/- 5 deg


        # Declare ROS params
        self.declare_parameter("use_real_racecar", True)  #for real car vs. sim
        self.declare_parameter("drive_topic", "/drive")  #default for simulation
        self.declare_parameter("safety_topic", "/vesc/low_level/input/safety")  #for real car

        # Get params
        self.use_real_racecar = self.get_parameter("use_real_racecar").get_parameter_value().bool_value
        self.DRIVE_TOPIC = self.get_parameter("drive_topic").get_parameter_value().string_value
        self.SAFETY_TOPIC = self.get_parameter("safety_topic").get_parameter_value().string_value

        # Output topic based on real car or sims 
        self.output_topic = self.SAFETY_TOPIC if self.use_real_racecar else self.DRIVE_TOPIC

        # Publishers
        self.drive_pub = self.create_publisher(AckermannDriveStamped, self.output_topic, 1)

        # Subscribers
        self.scan_sub = self.create_subscription(LaserScan, "/scan", self.scan_callback, 1)
        self.ackermann_sub = self.create_subscription(AckermannDriveStamped, "/vesc/high_level/input/nav_0", self.ackermann_callback, 1)


        ###Optional log data
        '''
        self.safety_controller_data = "safety_controller_data.csv" 
        if not os.path.exists(self.safety_controller_data):
            with open(self.safety_controller_data, mode="w", newline="") as file:
                writer = csv.writer(file)
                writer.writerow(["Estimated Distance to Obstacle (m)", "Velocity (m/s)", "Action"])

        self.safety_controller_data_records = []
        '''


    def scan_callback(self, msg):
        # LIDAR to np array
        ranges = np.array(msg.ranges)
        angles=np.linspace(msg.angle_min, msg.angle_max, len(ranges))

        # only -pi/6 to pi/6
        # mask = (angles >= self.current_steer-np.pi/6) & (angles <= self.current_steer+np.pi/6)
        mask = (angles >= -self.lookahead_angle) & (angles <= self.lookahead_angle) #new range
        good_range=ranges[mask]

        # bad data out
        good_range = good_range[(good_range>msg.range_min) & (good_range<msg.range_max)]

        # closest thing to hit
        if len(good_range) > 0:
            closest_pt = np.min(good_range)
        else:
            closest_pt = float('inf')  #no obstacle

        # self.get_logger().info(f"Closest point: {closest_pt:.3f}m")  #to debug

    
        #Dynamic stop threshold (depends on speed)
        stop_thresh = max(self.stop_thresh_base, 0.3 + 0.8*self.current_speed)

        # Self-explanatory
        if closest_pt < stop_thresh:
            self.publish_stop_command()
            self.get_logger().info("stopping")
            #self.record_data(closest_pt, self.hard_stop_speed, "STOP")
        elif closest_pt < self.min_slowing_distance:
            self.publish_brake_command()
            #self.record_data(closest_pt, self.step_brake_speed, "SLOW")
        else:
            pass  #let controller drive normally

    

    def ackermann_callback(self, msg):
        # self.get_logger().info(f"Received Drive Command: Speed={msg.drive.speed}, Steering={msg.drive.steering_angle}")
        self.current_speed = msg.drive.speed
        self.current_steer = msg.drive.steering_angle
    

    def publish_stop_command(self):
        stop_msg = AckermannDriveStamped()
        stop_msg.drive.speed = self.hard_stop_speed
        stop_msg.drive.steering_angle = 0.0
        self.drive_pub.publish(stop_msg)
        self.get_logger().warn("EMERGENCY STOP", throttle_duration_sec=1.0)

    def publish_brake_command(self):
        brake_msg = AckermannDriveStamped()
        brake_msg.drive.speed = self.step_brake_speed
        brake_msg.drive.steering_angle = 0.0
        self.drive_pub.publish(brake_msg)
        # self.get_logger().warn(f"SLOWING DOWN - reducing to {self.step_brake_speed} m/s.")


def main():
    rclpy.init()
    safety_controller=SafetyController()
    rclpy.spin(safety_controller)
    safety_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
