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
from final_interfaces.msg import DetectionStates
from ackermann_msgs.msg import AckermannDriveStamped

class HeistState(Enum):
    IDLE = auto()
    PLAN_TRAJ = auto()
    FOLLOW_TRAJ = auto()
    WAIT_TRAFFIC = auto()
    SCOUT = auto()
    PARK = auto()
    PICKUP = auto()
    ESCAPE = auto()
    COMPLETE = auto()

class StateMachineNode(Node):
    def __init__(self):
        super().__init__('state_machine')
        self.get_logger().info('Heist State Machine Initialized')

        self.state = HeistState.IDLE
        self.intial_pose = None
        self.goals = []
        self.goal_idx = None
        self.pickup_time = None
        self.bridge = CvBridge()

        self.create_subscription(PoseArray, '/shrinkray_part', self.goals_cb, 1)
        self.create_subscription(PoseStamped, '/initialpose', self.pose_cb, 1)
        self.create_subscription(DetectionStates, '/detector/states', self.detection_cb, 1)
        self.create_timer(0.1, self.on_timer)

        self.planner = PathPlan()
        self.follower = PurePursuit()
        self.wall_follow = WallFollower()

        self.parking_started = False
        self.parking_done_time = None

        self.sweep_count = 0
        self.max_sweep_attempts = 3
        self.sweep_start_time = None
        self.sweep_duration = 2.0
        self.sweep_drive_pub = self.create_publisher(AckermannDriveStamped, "/drive", 10)

    def pose_cb(self, msg: PoseStamped):
        self.intial_pose = msg.pose
        self.get_logger().info(f'Initial pose: {self.intial_pose.position.x}, {self.intial_pose.position.y}')

    def goals_cb(self, msg: PoseArray):
        self.goals = [(p.position.x, p.position.y) for p in msg.poses]
        self.get_logger().info(f'Received goals: {self.goals}')
        if len(self.goals) >= 2 and self.state == HeistState.IDLE:
            self.state = HeistState.PLAN_TRAJ
            self.goal_idx = 0

    def detection_cb(self, msg: DetectionStates):
        if msg.traffic_light_state != 'GREEN':
            self.state = HeistState.WAIT_TRAFFIC
        elif msg.traffic_light_state == 'GREEN' and self.state == HeistState.WAIT_TRAFFIC:
            self.state = HeistState.FOLLOW_TRAJ
        if msg.banana_state == 'DETECTED':
            self.state = HeistState.PARK
        if msg.person_state == 'DETECTED':
            self.get_logger().info('Human detected - stopping temporarily')
            # Could pause controller or use safety state

    def publish_sweep_motion(self, direction: int = 1):
        sweep_msg = AckermannDriveStamped()
        sweep_msg.drive.speed = 0.2
        sweep_msg.drive.steering_angle = direction * 0.34
        self.sweep_drive_pub.publish(sweep_msg)

    def on_timer(self):
        match self.state:
            case HeistState.IDLE:
                self.get_logger().info('Waiting for initial pose and goals')
                if self.intial_pose is not None and len(self.goals) >= 2:
                    self.state = HeistState.PLAN_TRAJ
                    self.goal_idx = 0

            case HeistState.PLAN_TRAJ:
                self.get_logger().info(f'Planning to goal #{self.goal_idx}')
                self.planner.plan_path(self.goals[self.goal_idx])
                self.state = HeistState.FOLLOW_TRAJ

            case HeistState.FOLLOW_TRAJ:
                self.follower.follow_path()
                if self.follower.has_reached_goal():
                    self.get_logger().info(f'Reached goal #{self.goal_idx}')
                    self.pickup_time = None
                    self.state = HeistState.SCOUT

            case HeistState.WAIT_TRAFFIC:
                self.get_logger().info('No green light, waiting')

            case HeistState.SCOUT:
                if self.sweep_count < self.max_sweep_attempts:
                    if self.sweep_start_time is None:
                        self.sweep_start_time = time.time()
                        direction = 1 if self.sweep_count % 2 == 0 else -1
                        self.publish_sweep_motion(direction)
                        self.get_logger().info(f'Sweep #{self.sweep_count + 1} started')
                    elif time.time() - self.sweep_start_time > self.sweep_duration:
                        self.get_logger().info(f'Sweep #{self.sweep_count + 1} complete')
                        self.sweep_count += 1
                        self.sweep_start_time = None
                else:
                    self.get_logger().warn('Banana not found after sweeps, continuing')
                    self.sweep_count = 0
                    self.state = HeistState.PLAN_TRAJ
                    self.goal_idx += 1

            case HeistState.PARK:
                if not self.parking_started:
                    self.get_logger().info('Starting parking maneuver')
                    self.parking_started = True
                    self.parking_done_time = time.time() + 5.0
                elif time.time() >= self.parking_done_time:
                    self.get_logger().info('Parking complete')
                    self.state = HeistState.PICKUP

            case HeistState.PICKUP:
                if self.pickup_time is None:
                    self.pickup_time = time.time()
                    self.get_logger().info(f'Picking up #{self.goal_idx}')
                elif (elapsed := time.time() - self.pickup_time) > 5.0:
                    self.goal_idx += 1
                    self.sweep_count = 0
                    self.parking_started = False
                    if self.goal_idx >= len(self.goals):
                        self.state = HeistState.ESCAPE
                    else:
                        self.state = HeistState.PLAN_TRAJ
                else:
                    self.get_logger().info(f'Waiting during pickup: {elapsed:.1f}s')

            case HeistState.ESCAPE:
                self.get_logger().info('Escaping')
                self.planner.plan_path(self.intial_pose.position)
                self.follower.follow_path()
                if self.follower.has_reached_goal():
                    self.state = HeistState.COMPLETE

            case HeistState.COMPLETE:
                self.get_logger().info('Heist COMPLETE')

def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()