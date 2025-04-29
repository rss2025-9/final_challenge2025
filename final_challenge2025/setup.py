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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='racecar',
    maintainer_email='mkzstar2013@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector = final_challenge2025.shrinkray_heist.model.detection_node:main',
            'homography_transformer = final_challenge2025.homography_transformer:main',
            'lane_detector = final_challenge2025.lane_detector:main',
            'safety_controller = final_challenge2025.safety_controller:main',
        ],
    },
)
