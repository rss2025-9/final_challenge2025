import glob, os
from setuptools import find_packages, setup

package_name = 'final_challenge2025'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/launch/race', glob.glob(os.path.join('launch', '*launch.*'))),
        (os.path.join('share', package_name, 'config', 'race'), glob.glob('config/race/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rss2025-team-9',
    maintainer_email=['mkzstar2013@gmail.com', 'rengz@mit.edu', 'apados@mit.edu', 'selinna@mit.edu'],
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector = final_challenge2025.shrinkray_heist.model.detection_node:main',
            'homography_transformer = final_challenge2025.homography_transformer:main',
            'lane_detector = final_challenge2025.lane_detector:main',
            'safety_controller = final_challenge2025.safety_controller:main',
            'trajectory_follower = final_challenge2025.trajectory_follower:main',
        ],
    },
)
