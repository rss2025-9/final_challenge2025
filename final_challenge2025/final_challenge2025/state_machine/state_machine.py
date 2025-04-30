#!/usr/bin/env python3
"""
ROS2 node implementing the full Shrink-Ray Heist state machine,
with YOLO detection for bananas and traffic lights (red/green stop/go logic).
"""
import rclpy
from rclpy.node import Node
from enum import Enum, auto
import time
import numpy as np
import cv2

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, PoseStamped
from cv_bridge import CvBridge

from shrinkray_heist.model.detector_msgs.msg import DetectionArray

from final_challenge2025.particle_filter import ParticleFilter
from final_challenge2025.trajectory_planner import PathPlan
from final_challenge2025.trajectory_follower import PurePursuit
from final_challenge2025.wall_follower import WallFollower
from std_msgs.msg import String

class HeistState(Enum):
    IDLE = auto()
    PLAN_TRAJ1 = auto()
    FOLLOW_TRAJ1 = auto()
    WAIT_TRAFFIC1 = auto()
    INSPECT1 = auto()
    IDENTIFY1 = auto()
    PICKUP1 = auto()
    PLAN_TRAJ2 = auto()
    FOLLOW_TRAJ2 = auto()
    WAIT_TRAFFIC2 = auto()
    INSPECT2 = auto()
    IDENTIFY2 = auto()
    PICKUP2 = auto()
    ESCAPE = auto()
    COMPLETE = auto()

class StateMachineNode(Node):
    # helpers 
    def any_detection(self, target_label):
        return any(label == target_label for _, label, _ in self.detections)

    def detect_green_light(self):
        if self.last_frame is None: return False
        for _, label, bbox in self.detections:
            if label == 'traffic_light':
                x, y = bbox.x_offset, bbox.y_offset
                w, h = bbox.width, bbox.height
                roi = self.last_frame[y:y+h, x:x+w]
                hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
                mask = cv2.inRange(hsv, (40,50,50), (90,255,255))
                if cv2.countNonZero(mask) > 0.1*h*w:
                    return True
        return False

    # constructor 
    def __init__(self):
        super().__init__('state_machine')
        self.get_logger().info('Heist State Machine Initialized')

        # Initialize state machine variables
        self.state = HeistState.IDLE
        self.intial_pose = None 
        self.goals = []             # two (x,y) targets
        self.detections = []        # [(timestamp,label,bbox)]
        self.last_frame = None      # latest raw image for color segmentation
        self.pickup_time = None

        # subscriptions
        self.bridge = CvBridge()
        self.create_subscription(PoseArray, '/shrinkray_part', self.goals_cb, 1)
        self.create_subscription(PoseStamped, '/initialpose', self.pose_cb, 1)
        self.create_subscription(String, '/traffic_light/state', self.traffic_cb, 1)
        self.create_timer(0.1, self.on_timer)

        # moving parts 
        self.planner = PathPlan()
        self.follower = PurePursuit()
        self.wall_follow = WallFollower()

    # intial pose callback 
    def pose_cb(self, msg: PoseStamped):
        self.intial_pose = msg.pose
        self.get_logger().info(f'Initial pose: {self.intial_pose.position.x}, {self.intial_pose.position.y}')

    # Goals callback: set the state once both goals are received
    def goals_cb(self, msg: PoseArray):
        self.goals = [(p.position.x, p.position.y) for p in msg.poses]
        self.get_logger().info(f'Received goals: {self.goals}')
        if len(self.goals) == 2 and self.state == HeistState.IDLE:
            self.state = HeistState.PLAN_TRAJ1

    # State machine 
    def on_timer(self):
        if self.state == HeistState.IDLE:
            self.get_logger().info('Waiting for initial pose and goals')
            if self.intial_pose is not None and len(self.goals) == 2:
                self.state = HeistState.PLAN_TRAJ1

        if self.state == HeistState.PLAN_TRAJ1:
            self.get_logger().info('Planning to goal #1')
            self.planner.plan_path(self.goals[0])
            self.state = HeistState.FOLLOW_TRAJ1

        elif self.state == HeistState.FOLLOW_TRAJ1:
            # check for traffic light
            if not self.detect_green_light():
                self.get_logger().info('waiting for green light')
                self.state = HeistState.WAIT_TRAFFIC1
                return
            # normal follow
            self.follower.follow_path()
            if self.follower.has_reached_goal():
                self.get_logger().info('Reached goal #1')
                self.state = HeistState.INSPECT1
                self.pickup_time = None

        elif self.state == HeistState.WAIT_TRAFFIC1:
            # wait until green
            if self.detect_green_light():
                self.get_logger().info('Green light, resuming')
                # back to prior follow state
                self.state = HeistState.FOLLOW_TRAJ1

        elif self.state == HeistState.INSPECT1:
            if self.any_detection('banana'):
                self.get_logger().info('Banana seen #1')
                self.state = HeistState.IDENTIFY1

        elif self.state == HeistState.IDENTIFY1:
            if self.any_detection('banana'):
                self.get_logger().info('Identified banana #1')
                self.state = HeistState.PICKUP1
            else:
                self.state = HeistState.INSPECT1

        elif self.state == HeistState.PICKUP1:
            if self.pickup_time is None:
                self.pickup_time = time.time(); self.get_logger().info('Picking up #1')
            elif time.time() - self.pickup_time > 5.0:
                self.state = HeistState.PLAN_TRAJ2

        elif self.state == HeistState.PLAN_TRAJ2:
            self.get_logger().info('Planning to goal #2')
            self.planner.plan_path(self.goals[1])
            self.state = HeistState.FOLLOW_TRAJ2

        elif self.state == HeistState.FOLLOW_TRAJ2:
            if not self.detect_green_light():
                self.state = HeistState.WAIT_TRAFFIC2
                return
            self.follower.follow_path()
            if self.follower.has_reached_goal():
                self.state = HeistState.INSPECT2; self.pickup_time=None

        elif self.state == HeistState.WAIT_TRAFFIC2:
            # wait until green
            if self.detect_green_light():
                self.get_logger().info('Green light, resuming')
                # back to prior follow state
                self.state = HeistState.FOLLOW_TRAJ2

        elif self.state == HeistState.INSPECT2:
            if self.any_detection('banana'):
                self.state = HeistState.IDENTIFY2

        elif self.state == HeistState.IDENTIFY2:
            if self.any_detection('banana'):
                self.state = HeistState.PICKUP2
            else:
                self.state = HeistState.INSPECT2

        elif self.state == HeistState.PICKUP2:
            if self.pickup_time is None:
                self.pickup_time = time.time(); self.get_logger().info('Picking up #2')
            elif time.time() - self.pickup_time > 5.0:
                self.state = HeistState.ESCAPE

        elif self.state == HeistState.ESCAPE:
            self.get_logger().info('Escaping')
            self.planner.plan_path(self.intial_pose.position)

            if not self.detect_green_light():
                self.state = HeistState.WAIT_TRAFFIC2
                return
            
            self.follower.follow_path()

            self.state = HeistState.COMPLETE

        elif self.state == HeistState.COMPLETE:
            self.get_logger().info('Heist COMPLETE')

def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
