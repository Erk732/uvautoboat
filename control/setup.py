from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
<<<<<<< Updated upstream
        # Uses glob to find all launch files (prevents errors if specific files are missing)
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
=======
        ('share/' + package_name + '/launch', [
            'launch/all_in_one_bringup.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/hazard_world_boxes.yaml',
        ]),
>>>>>>> Stashed changes
    ],
    zip_safe=True,
    maintainer='ghostzero',
    maintainer_email='yinpuchen0@gmail.com',
    description='Control package for VRX WAM-V thruster-based path following.',
    license='Apache-2.0',
    tests_require=['pytest'],
    python_requires='>=3.10',
    classifiers=[
        'Programming Language :: Python :: 3',
        'License :: OSI Approved :: Apache Software License',
        'Operating System :: OS Independent',
    ],
    entry_points={
        'console_scripts': [
            # Atlantis Controller
            'atlantis_controller = control.atlantis_controller:main',
            
            # Helper for Obstacle Avoidance (if run standalone)
            'lidar_obstacle_avoidance = control.lidar_obstacle_avoidance:main',

            # Other Controllers (Preserved from your code)
            'buran_controller = control.buran_controller:main',
            'keyboard_teleop = control.keyboard_teleop:main',
            'all_in_one_stack = control.all_in_one_stack:main',
            'gps_imu_pose = control.gps_imu_pose:main',
            'pose_filter = control.pose_filter:main',
            'buran_controller_fixed = control.buran_controller_fixed:main',
            'goal_sequencer = control.goal_sequencer:main',
        ],
    },
)
