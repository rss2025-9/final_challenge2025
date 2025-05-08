#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from visualization_msgs.msg import Marker
from rcl_interfaces.msg import SetParametersResult
import os 
import csv

from wall_follower.visualization_tools import VisualizationTools


class WallFollower(Node):

    def __init__(self):
        super().__init__("wall_follower")

        #ADDED gains
        self.kp=1.0 #3 (12)
        self.kd=0.5 #1.5 (4)
        #ki (10)

        self.prev_error = 0.0

        # Declare parameters to make them available for use
        # DO NOT MODIFY THIS! 
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("drive_topic", "/vesc/high_level/input/nav_0")
        self.declare_parameter("side", -1)
        self.declare_parameter("velocity", 1.5)
        self.declare_parameter("desired_distance", 0.75)
        self.declare_parameter('using_real_car', True)

        # Fetch constants from the ROS parameter server
        # DO NOT MODIFY THIS! This is necessary for the tests to be able to test varying parameters!
        self.SCAN_TOPIC = self.get_parameter('scan_topic').get_parameter_value().string_value
        self.DRIVE_TOPIC = self.get_parameter('drive_topic').get_parameter_value().string_value
        self.SIDE = self.get_parameter('side').get_parameter_value().integer_value
        #self.SIDE = 1 #-1 right
        self.VELOCITY = self.get_parameter('velocity').get_parameter_value().double_value
        self.DESIRED_DISTANCE = self.get_parameter('desired_distance').get_parameter_value().double_value      

        # This activates the parameters_callback function so that the tests are able
        # to change the parameters during testing.
        # DO NOT MODIFY THIS! 
        self.add_on_set_parameters_callback(self.parameters_callback)

        #ADDED subscribe to lidar data (subscibe to SCN_TOPIC), listens for messages of type LaserScan, when msg arrives lidar_callback called
        self.subscription=self.create_subscription(LaserScan, self.SCAN_TOPIC, self.lidar_callback, 1)
        self.subscription 
        #ADDED publish drive,  publishes msgs of type AckermannDriveStamped to topic DRIVE_TOPIC
        self.drive_pub=self.create_publisher(AckermannDriveStamped, self.DRIVE_TOPIC, 10)
        # a publisher for our line marker
        self.line_pub = self.create_publisher(Marker, "wall", 1)

        # File for wall follower data collection
        self.wall_follower_data = "wall_follower_data.csv" 
        if not os.path.exists(self.wall_follower_data):
            with open(self.wall_follower_data, mode="w", newline="") as file:
                writer = csv.writer(file)
                writer.writerow(["Time (s)", "Desired Distance (m)", "Estimated Distance to Wall (m)", "Error (m)", "Velocity (m/s)"])

        self.wall_follower_data_records = []
        self.current_time = 0

    # TODO: Write your callback functions here   
    #ADDED
    def lidar_callback(self, msg):
        if not msg.ranges: #no lidar data received
            self.get_logger().warn("No LIDAR data")
            return

        #to np array
        msg_arr=np.array(msg.ranges)
        angles=np.linspace(msg.angle_min, msg.angle_max, len(msg_arr))

        # angle range
        ang_min=-np.pi/8 if self.SIDE==1 else -np.pi*3/8
        ang_max=np.pi*3/8 if self.SIDE==1 else np.pi/8

        #indices of this range
        mask=(angles>=ang_min) & (angles<=ang_max)
        selected_angles=angles[mask]
        selected_ranges=msg_arr[mask]

        #get rid of bad readings
        #valid_mask=(selected_ranges>msg.range_min) & (selected_ranges<msg.range_max) 
        valid_mask=(selected_ranges>msg.range_min) & (selected_ranges<4.0) 
        selected_angles=selected_angles[valid_mask]
        selected_ranges=selected_ranges[valid_mask]

        if len(selected_ranges) < 2:
            self.get_logger().warn("not enough data")
            return

        #to cartesian
        x_vals=selected_ranges*np.cos(selected_angles)
        y_vals=selected_ranges*np.sin(selected_angles)

        #fit line
        slope, y_int=np.polyfit(x_vals, y_vals, 1)

        #show hypothesized wall
        x = np.linspace(-2., 2., num=20)
        y = slope*x +y_int
        VisualizationTools.plot_line(x, y, self.line_pub, frame="/laser")

        #perp dist from car to line
        dist_to_wall=abs(y_int)/np.sqrt(slope**2+1)

        #find error
        error=self.DESIRED_DISTANCE-dist_to_wall #FLIP?

        #compute derivative
        dt=max(msg.scan_time, 0.05)  #so no /0
        der=(error-self.prev_error)/dt

        #regular PD control
        if self.SIDE==-1: #if right side
            steer_ang=self.kp*error+self.kd*slope 
        else:
            steer_ang=-(self.kp*error+self.kd*-slope)

        # mid angle range 
        mid_ang_min = -np.pi/8 
        mid_ang_max = np.pi/8

        mid_mask=(angles>=mid_ang_min) & (angles<=mid_ang_max)
        selected_mid_angles=angles[mid_mask]
        selected_mid_ranges=msg_arr[mid_mask]

        valid_mid_mask=(selected_mid_ranges>msg.range_min) & (selected_mid_ranges<4.0) 
        selected_mid_angles=selected_mid_angles[valid_mid_mask]
        selected_mid_ranges=selected_mid_ranges[valid_mid_mask]

        if len(selected_mid_ranges) != 0: 
            closest_mid_point = np.mean(selected_mid_ranges)
            if closest_mid_point >= 2.0: 
                self.VELOCITY = 1.5
            elif closest_mid_point >= 1.0: 
                self.VELOCITY = 1.25
            else: 
                self.VELOCITY = 1.0 

        self.publish_drive_command(steer_ang)
        self.prev_error=error

        # Writing to wall follower data spreadsheet
        self.wall_follower_data_records.append([self.current_time, self.DESIRED_DISTANCE, round(dist_to_wall,2), round(error,2), self.VELOCITY])
        self.current_time += 1 

        # Write to file every 10 records
        if len(self.wall_follower_data_records) >= 10:
            self.write_to_csv()

    def write_to_csv(self): 
        with open(self.wall_follower_data, mode = 'a', newline = '') as file: 
            writer = csv.writer(file)
            writer.writerows(self.wall_follower_data_records)
        self.wall_follower_data_records = []

    def publish_drive_command(self, steer_ang):
        #max_steer_ang=0.5  #something reasonable
        #steer_ang=np.clip(steer_ang, -max_steer_ang, max_steer_ang) #limits range of steering ang to [-max,max]
        # msg=AckermannDriveStamped() #message of type AckermannDriveStamped()
        # #set
        # base_speed = self.VELOCITY 
        # min_speed = 1.0 
        # max_speed = 3.0
        # steer_thresh = 0.3

        # if abs(steer_ang) > steer_thresh:
        #     drive_speed = max(min_speed, base_speed * (1 - abs(steer_ang)))
        # else:
        #     drive_speed = min(max_speed, base_speed * 1.2)
        # msg.drive.speed = drive_speed 
        # msg.drive.steering_angle=steer_ang
        # #publish msg (steering ang an speed) to /drive topic
        # self.drive_pub.publish(msg)

        msg=AckermannDriveStamped() #message of type AckermannDriveStamped()
        #set
        msg.drive.speed=self.VELOCITY
        msg.drive.steering_angle=steer_ang
        #publish msg (steering ang an speed) to /drive topic
        self.drive_pub.publish(msg)
    
    def parameters_callback(self, params):
        """
        DO NOT MODIFY THIS CALLBACK FUNCTION!
        
        This is used by the test cases to modify the parameters during testing. 
        It's called whenever a parameter is set via 'ros2 param set'.
        """
        for param in params:
            if param.name == 'side':
                self.SIDE = param.value
                self.get_logger().info(f"Updated side to {self.SIDE}")
            elif param.name == 'velocity':
                self.VELOCITY = param.value
                self.get_logger().info(f"Updated velocity to {self.VELOCITY}")
            elif param.name == 'desired_distance':
                self.DESIRED_DISTANCE = param.value
                self.get_logger().info(f"Updated desired_distance to {self.DESIRED_DISTANCE}")
        return SetParametersResult(successful=True)


def main():
    rclpy.init()
    wall_follower = WallFollower()
    rclpy.spin(wall_follower)
    wall_follower.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()