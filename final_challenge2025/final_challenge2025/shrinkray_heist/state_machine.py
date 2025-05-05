import rclpy
from rclpy.node import Node
from enum import Enum, auto
import time

# from scipy.ndimage import binary_dilation
from scipy.ndimage import distance_transform_edt
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, PoseStamped
from cv_bridge import CvBridge
from final_interfaces.msg import DetectionStates
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from . import HeistState
from . import HEISTSTATE

class StateMachineNode(Node):
    # constructor 
    def __init__(self):
        super().__init__('state_machine')
        self.get_logger().info('Heist State Machine Initialized')

        HEISTSTATE = HeistState.IDLE

        self.initial_pose = None
        self.goals = []
        self.goal_idx = None
        self.pickup_time = None
        self.bridge = CvBridge()

        self.create_subscription(PoseArray, '/shrinkray_part', self.goals_cb, 1) # 2 goal positions 
        self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.pose_cb, 1) # initial pose
        self.create_subscription(DetectionStates, '/detector/states', self.detection_cb, 1) # custom YOLO detection messages
        self.create_subscription(Odometry, '/pf/pose/odom', self.odom_cb, 1) # odometry data

        # dummy fix for the trajectory follower
        self.create_subscription(Bool, '/end_trajectory', self.trajectory_cb, 1)

        self.start_publish = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", 10)
        self.goal_publish = self.create_publisher(PoseStamped, "/goal_pose", 10)

        self.create_timer(0.1, self.on_timer)

        self.parking_started = False
        self.parking_done_time = None

        self.sweep_count = 0
        self.max_sweep_attempts = 3
        self.sweep_start_time = None
        self.sweep_duration = 2.0

        self.drive_pub = self.create_publisher(AckermannDriveStamped, "/drive", 1)

        self.pose_set = False
        self.curr_pos = None
        self.finished_traj = False
    
    # dummy fix for the trajectory follower
    def trajectory_cb(self, msg: Bool):
        if msg.data:
            self.finished_traj = True
            HEISTSTATE = HeistState.SCOUT
        else:
            self.finished_traj = False
    
    def odom_cb(self, msg: Odometry):
        self.curr_pos = msg

    def pose_cb(self, msg: PoseWithCovarianceStamped):
        if self.pose_set:
            return
        self.initial_pose = msg
        self.get_logger().info(f'Initial pose: {self.initial_pose.pose.pose.position.x}, {self.initial_pose.pose.pose.position.y}')
        self.pose_set = True

    def goals_cb(self, msg: PoseArray):
        self.goals = [(p.position.x, p.position.y) for p in msg.poses]
        self.get_logger().info(f'Received goals: {self.goals}')

    def detection_cb(self, msg: DetectionStates):
        if msg.traffic_light_state == 'RED':
            HEISTSTATE = HeistState.WAIT_TRAFFIC
        elif msg.traffic_light_state == 'GREEN' and HEISTSTATE == HeistState.WAIT_TRAFFIC:
            HEISTSTATE = HeistState.FOLLOW_TRAJ
        if msg.banana_state == 'DETECTED' and HEISTSTATE == HeistState.SCOUT:
            HEISTSTATE = HeistState.PARK
        # if msg.person_state == 'DETECTED':
        #     self.get_logger().info('Human detected - stopping temporarily')
        #     # Could pause controller or use safety state

    def publish_sweep_motion(self, direction: int = 1):
        sweep_msg = AckermannDriveStamped()
        sweep_msg.drive.speed = 0.2
        sweep_msg.drive.steering_angle = direction * 0.34
        self.drive_pub.publish(sweep_msg)

    def on_timer(self):
        match HEISTSTATE:
            case HeistState.IDLE:
                # self.get_logger().info('Waiting for initial pose and goals')
                if self.initial_pose is not None and len(self.goals) == 2:
                    self.get_logger().info('Initial pose and goals received')
                    HEISTSTATE = HeistState.PLAN_TRAJ
                    self.goal_idx = 0

            case HeistState.PLAN_TRAJ:
                self.get_logger().info(f'Planning to goal #{self.goal_idx}')
                if self.goal_idx == 0: 
                    self.start_publish.publish(self.initial_pose)
                    goal_pose = PoseStamped()
                    goal_pose.header.frame_id = 'map'
                    goal_pose.header.stamp = self.get_clock().now().to_msg()
                    goal_pose.pose.position.x = self.goals[0][0]
                    goal_pose.pose.position.y = self.goals[0][1]
                    self.goal_publish.publish(goal_pose)
                else:
                    start_pose = PoseWithCovarianceStamped()
                    start_pose.header.frame_id ='map'
                    start_pose.header.stamp = self.get_clock().now().to_msg()
                    start_pose.pose.pose.position.x = self.goals[self.goal_idx - 1][0]
                    start_pose.pose.pose.position.y = self.goals[self.goal_idx - 1][1]
                    self.start_publish.publish(start_pose)
                    goal_pose = PoseStamped()
                    goal_pose.header.frame_id = 'map'
                    goal_pose.header.stamp = self.get_clock().now().to_msg()
                    goal_pose.pose.position.x = self.goals[self.goal_idx][0]
                    goal_pose.pose.position.y = self.goals[self.goal_idx][1]
                    self.goal_publish.publish(goal_pose)
                HEISTSTATE = HeistState.FOLLOW_TRAJ

            case HeistState.FOLLOW_TRAJ:
                # if self.curr_pos.pose.pose.position.x == self.goals[self.goal_idx][0] and self.curr_pos.pose.pose.position.y == self.goals[self.goal_idx][1]:
                #     self.get_logger().info(f'Reached goal #{self.goal_idx}')
                #     self.pickup_time = None
                #     HEISTSTATE = HeistState.SCOUT

                # dummy fix 
                if self.finished_traj:
                    self.get_logger().info(f'Reached goal #{self.goal_idx}')
                    self.pickup_time = None
                    self.finished_traj = False
                    HEISTSTATE = HeistState.SCOUT

            case HeistState.WAIT_TRAFFIC:
                # publish stop cmd
                msg = AckermannDriveStamped()
                msg.drive.speed = 0.0
                msg.drive.steering_angle = 0.0
                self.drive_pub.publish(msg)
                # self.get_logger().info('No green light, waiting')

            case HeistState.SCOUT:
                HEISTSTATE = HeistState.PARK
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
                    HEISTSTATE = HeistState.PLAN_TRAJ
                    self.goal_idx += 1

            case HeistState.PARK:
                if self.parking_done_time is None:
                    self.parking_done_time = time.time() + 5.0
                    self.get_logger().info('Starting parking maneuver')
                    self.parking_started = True
                elif time.time() >= self.parking_done_time:
                    self.get_logger().info('Parking complete')
                    self.goal_idx += 1
                    self.sweep_count = 0
                    self.parking_started = False
                    if self.goal_idx >= len(self.goals):
                        HEISTSTATE = HeistState.ESCAPE
                    else:
                        HEISTSTATE = HeistState.PLAN_TRAJ
                else:
                    # publish stop cmd
                    msg = AckermannDriveStamped()
                    msg.drive.speed = 0.0
                    msg.drive.steering_angle = 0.0
                    self.drive_pub.publish(msg)

            case HeistState.ESCAPE:
                self.get_logger().info('Escaping')
                start_pose = PoseWithCovarianceStamped()
                start_pose.header.frame_id ='map'
                start_pose.header.stamp = self.get_clock().now().to_msg()
                start_pose.pose.pose.position.x = self.goals[-1][0]
                start_pose.pose.pose.position.y = self.goals[-1][1]
                self.start_publish.publish(self.goals[-1])
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = 'map'
                goal_pose.header.stamp = self.get_clock().now().to_msg()
                goal_pose.pose = self.initial_pose.pose.pose
                self.goal_publish.publish(goal_pose)
                if self.curr_pos.pose.pose.position.x == self.initial_pose.pose.pose.position.x and self.curr_pos.pose.pose.position.y == self.initial_pose.pose.pose.position.y:
                    HEISTSTATE = HeistState.COMPLETE

            case HeistState.COMPLETE:
                self.get_logger().info('Heist COMPLETE')

def main(args=None):
    rclpy.init(args=args)
    node = StateMachineNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()