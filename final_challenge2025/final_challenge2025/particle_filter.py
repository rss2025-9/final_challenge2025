from localization.sensor_model import SensorModel
from localization.motion_model import MotionModel

from std_msgs.msg import Header
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray, Pose, Point, Quaternion

import numpy as np
import numpy.typing as npt

from rclpy.node import Node
import rclpy

assert rclpy

from sensor_msgs.msg import LaserScan

import threading 
import numpy.typing as npt 

class ParticleFilter(Node):
    def __init__(self):
        super().__init__("particle_filter")

        self.declare_parameter('particle_filter_frame', "default")
        self.particle_filter_frame = self.get_parameter('particle_filter_frame').get_parameter_value().string_value
        
        # Visualization control for in vivo optimization.
        self.declare_parameter('optimize_publishes', True)
        self.optimize_publishes = self.get_parameter('optimize_publishes').get_parameter_value().bool_value

        #  *Important Note #1:* It is critical for your particle
        #     filter to obtain the following topic names from the
        #     parameters for the autograder to work correctly. Note
        #     that while the Odometry message contains both a pose and
        #     a twist component, you will only be provided with the
        #     twist component, so you should rely only on that
        #     information, and *not* use the pose component.
        
        self.declare_parameter('odom_topic', "/odom")
        self.declare_parameter('scan_topic', "/scan")

        scan_topic = self.get_parameter("scan_topic").get_parameter_value().string_value
        odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value

        self.laser_sub = self.create_subscription(LaserScan, scan_topic,
                                                  self.laser_callback,
                                                  1)

        self.odom_sub = self.create_subscription(Odometry, odom_topic,
                                                 self.odom_callback,
                                                 1)

        #  *Important Note #2:* You must respond to pose
        #     initialization requests sent to the /initialpose
        #     topic. You can test that this works properly using the
        #     "Pose Estimate" feature in RViz, which publishes to
        #     /initialpose.

        self.pose_sub = self.create_subscription(PoseWithCovarianceStamped, "/initialpose",
                                                 self.pose_callback,
                                                 1)

        #  *Important Note #3:* You must publish your pose estimate to
        #     the following topic. In particular, you must use the
        #     pose field of the Odometry message. You do not need to
        #     provide the twist part of the Odometry message. The
        #     odometry you publish here should be with respect to the
        #     "/map" frame.

        self.odom_pub = self.create_publisher(Odometry, "/pf/pose/odom", 1)
        self._odom_msg = Odometry(
            header=Header(
                stamp=self.get_clock().now().to_msg(),
                frame_id="map",
            ),
            child_frame_id=self.particle_filter_frame
        )

        # Initialize the models
        self.motion_model = MotionModel(self)
        self.sensor_model = SensorModel(self)

        self.get_logger().info("=============+READY+=============")

        # Implement the MCL algorithm
        # using the sensor model and the motion model
        #
        # Make sure you include some way to initialize
        # your particles, ideally with some sort
        # of interactive interface in rviz
        #
        # Publish a transformation frame between the map
        # and the particle_filter_frame.

        # visualizing particles
        self.particles_pub = self.create_publisher(PoseArray, "/particles", 1)

        self.prev_time = None

        # initializing number of particles, particles, + their weights 
        self.declare_parameter('num_particles', 200) 
        self.num_particles = self.get_parameter('num_particles').get_parameter_value().integer_value

        self.particles = np.zeros((self.num_particles, 3)) # (x, y, theta)  
        self.weights = np.ones(self.num_particles)/self.num_particles 

        self.lock = threading.Lock()

        self.simulation = False

    def pose_callback(self, pose: PoseWithCovarianceStamped): 
        """
        use rviz /initial_pose for initializing the particles and a guess of where the robot's location is 
        with a random spread of particles around a clicked point or pose. 
        """

        # getting x, y, yaw from the pose
        pose: tuple = (
            pose.pose.pose.position.x,
            pose.pose.pose.position.y,
            np.arctan2(pose.pose.pose.orientation.z, pose.pose.pose.orientation.w) * 2
        )
        
        # intialize particles around this with gaussian 
        if self.weights is not None: 
            with self.lock: 
                self.particles = np.random.normal(
                    pose, 
                    [self.motion_model.x_std, self.motion_model.y_std, self.motion_model.theta_std], 
                    (self.num_particles, 3)
                )
                self.weights.fill(1 / self.num_particles) # weights set uniformly across all particles for initialization 

    def odom_callback(self, odometry: Odometry): 
        """
        process odometry data to pass into motion model
        only rely on twist and not pose 
        """
        
        # calculate the change in time (dt)
        curr_time: float = odometry.header.stamp.sec + odometry.header.stamp.nanosec * 1e-9

        if self.prev_time is None:
            self.prev_time = curr_time
            return
        
        dt: float = curr_time - self.prev_time
        self.prev_time = curr_time

        # get odometry data 
        odom: npt.NDArray[np.float] = np.array(
            [odometry.twist.twist.linear.x, # vx
             odometry.twist.twist.linear.y, # vy
             odometry.twist.twist.angular.z]# yaw
        )

        # IRL odom flip fix.
        if not self.simulation: 
            odom = -odom

        # evaluate through motion model and update particles
        if self.weights is not None: 
            with self.lock: 
                # Evolve particles by odometry * dt.
                self.particles = self.motion_model.evaluate(self.particles, odom * dt)

    def laser_callback(self, scan: LaserScan): 
        """
        process scan data to pass into sensor model 
        and resample particles based on sensor model probabilities, numpy.random.choice can be useful 
        """
        
        scan_ranges = np.array(scan.ranges)

        # self.get_logger().info("before lock")
        with self.lock: 
            # get probabilities for each particle by passing scans into the sensor model and update weights 
            self.weights = self.sensor_model.evaluate(self.particles, scan_ranges)

            if self.weights is None: 
                return # no weights  
            
            if (cum_weights := np.sum(self.weights)) != 0:
                self.weights /= cum_weights # normalize all the weights 

            # resample particles 
            self.particles = self.particles[np.random.choice(
                self.particles.shape[0], size = self.particles.shape[0], 
                p = self.weights, replace = True
            )]
            self.publish_avg_pose()
        
        self.publish_particles()

    def publish_avg_pose(self):
        # publish msg
        # determine "Average" particle pose and publish 
        # publishes estimated pose as a transformation between the /map frame and a frame for the expected car's base link 
        # --> publish to /base_link_pf for simulator 
        # --> publish to /base_link for real car 

        # weighted means for x and y, circular mean for theta 
        mean: npt.NDArray[np.float] = np.sum(
            self.particles * self.weights[..., np.newaxis], axis=0
        )
        # publish estimated pose 
        self._odom_msg.header.stamp = self.get_clock().now().to_msg()
        self._odom_msg.pose.pose.position.x = mean[0]
        self._odom_msg.pose.pose.position.y = mean[1] 
        self._odom_msg.pose.pose.orientation.z = np.sin(mean[2] / 2)
        self._odom_msg.pose.pose.orientation.w = np.cos(mean[2] / 2)

        self.odom_pub.publish(self._odom_msg)

    def publish_particles(self):
        # The publish_particles function is used to visualize the particles in RViz.
        if self.optimize_publishes:
            return

        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "map"
        
        # Vectorized computations for positions and orientations
        half_thetas: npt.NDArray = self.particles[:, 2] / 2
        sin_half: npt.NDArray = np.sin(half_thetas)
        cos_half: npt.NDArray = np.cos(half_thetas)
        
        # Use list comprehension to create Pose messages for each particle
        pose_array.poses = [
            Pose(
                position=Point(x=self.particles[i, 0], y=self.particles[i, 1], z=0.0),
                orientation=Quaternion(x=0.0, y=0.0, z=sin_half[i], w=cos_half[i])
            )
            for i in range(self.num_particles)
        ]
        
        self.particles_pub.publish(pose_array)

def main(args=None):
    rclpy.init(args=args)
    pf = ParticleFilter()
    rclpy.spin(pf)
    rclpy.shutdown()