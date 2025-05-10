import glob, os
from Cython.Build import cythonize
from setuptools import find_packages, setup, Extension

package_name = 'final_challenge2025'
RACECAR_SIMULATOR_PREFIX = os.path.join(os.environ["SIM_WS"], "install", "racecar_simulator")
extensions = Extension(
    "scan_simulator_2d",
    [package_name + "/scan_simulator_2d.pyx"],
    language="c++",
    libraries=["racecar_simulator"],
    include_dirs=[os.path.join(RACECAR_SIMULATOR_PREFIX, "include")],
    library_dirs=[os.path.join(RACECAR_SIMULATOR_PREFIX, "lib")],
    extra_compile_args=['-Wno-cpp', '-g', '-Wno-maybe-uninitialized'],  # Added '-Wno-maybe-uninitialized'
)
setup(
    ext_modules=cythonize(extensions, force=True, quiet=True),
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (f'share/{package_name}/launch', glob.glob(os.path.join('launch', '*launch.*'))),
        (os.path.join('share', package_name, 'config', 'race'), glob.glob('config/race/*.yaml')),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rss2025-team-9',
    maintainer_email=['mzaw1@mit.edu', 'rengz@mit.edu', 'apados@mit.edu', 'selinna@mit.edu'],
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # State machine files.
            'particle_filter = final_challenge2025.shrinkray_heist.particle_filter:main',
            'state_machine = final_challenge2025.shrinkray_heist.state_machine:main',
            'safety_controller = final_challenge2025.safety_controller:main',
            # Race files.
            'race_control = final_challenge2025.moon_race.race_control:main',
            'lane_detector = final_challenge2025.moon_race.lane_detector:main',
            'homography_transformer = final_challenge2025.moon_race.homography_transformer:main',
            # Analysis files.
            'lap_analysis = final_challenge2025.moon_race.utils.lap_analysis:main',
        ],
    },
)
