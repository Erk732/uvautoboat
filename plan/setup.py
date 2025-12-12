from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'plan'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install Python launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Install YAML launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.yaml')),
        # Install rviz config files
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
    ],
    zip_safe=True,
    maintainer='ghostzero',
    maintainer_email='yinpuchen0@gmail.com',
    description='Planning and perception package for VRX autonomous navigation.',
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
            # Atlantis Planner
            'atlantis_planner = plan.atlantis_planner:main',

            # Modular Vostok1 (OKO + SPUTNIK + BURAN)
            'vostok1_cli = plan.vostok1_cli:main',
            'sputnik_planner = plan.sputnik_planner:main',
            'oko_perception = plan.oko_perception:main',

            # Perception & Utilities
            'mission_trigger = plan.mission_trigger:main',
            'waypoint_visualizer = plan.waypoint_visualizer:main',
            'tf_broadcaster = plan.tf_broadcaster:main',
            'tf_broadcaster_gazebo = plan.tf_broadcaster_gazebo:main',
            'tf_broadcaster_gps = plan.tf_broadcaster_gps:main',
            'simple_perception = plan.simple_perception:main',
            'pollutant_planner = plan.pollutant_planner:main',
            'lidar_obstacle_avoidance = plan.lidar_obstacle_avoidance:main',

            # Testing / Fixed versions
            'atlantis_planner_fixed = plan.atlantis_planner_fixed:main',
            'oko_perception_fixed = plan.oko_perception_fixed:main',
        ],
    },
)