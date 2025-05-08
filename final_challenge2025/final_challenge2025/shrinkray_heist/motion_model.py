import numpy.typing as npt
import numpy as np

class MotionModel:

    def __init__(self, node):
        ####################################
        node.declare_parameter('deterministic', False)
        node.declare_parameter('noise', 0.1)
        self.deterministic: bool = node.get_parameter('deterministic').get_parameter_value().bool_value
        std_coeff: float = node.get_parameter('noise').get_parameter_value().double_value if not self.deterministic else 0.0
        #####################################

        self.x_std: float = 2 * std_coeff
        self.y_std: float = 1 * std_coeff
        self.theta_std: float = np.radians(15) * std_coeff

        ####################################

    def evaluate(self, particles: npt.NDArray, odometry: npt.NDArray) -> npt.NDArray:
        """
        Update the particles to reflect probable
        future states given the odometry data.

        args:
            particles: An Nx3 matrix of the form:

                [x0 y0 theta0]
                [x1 y0 theta1]
                [    ...     ]

            odometry: A 3-vector [dx dy dtheta]

        returns:
            particles: An updated matrix of the
                same size
        """

        ####################################
        # Gets number of particles
        N: int = particles.shape[0]
        # Calculates the rotation matrix for the particles.
        sins: npt.NDArray = np.sin(particles[:, 2])
        coss: npt.NDArray = np.cos(particles[:, 2])

        # Generates all odometry noise for all particles.
        noisy_odom: npt.NDArray = np.random.normal(
            odometry[:2], [self.x_std, self.y_std], size=(N, 2)
        )
        particles[:, 0] += noisy_odom[:, 0] * coss - noisy_odom[:, 1] * sins
        particles[:, 1] += noisy_odom[:, 0] * sins + noisy_odom[:, 1] * coss
        particles[:, 2] += np.random.normal(odometry[2], self.theta_std, size=N)
        # Normalize the angles to be between -pi and pi
        particles[:, 2] = np.arctan2(np.sin(particles[:, 2]), np.cos(particles[:, 2]))
        return particles

        ####################################