from setuptools import setup
import os
from glob import glob

package_name = 'plan'

setup(
    name=package_name,
    version='0.0.0',
    packages=['brain'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        #(os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')), old launch file config
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        # Install rviz config files
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bot',
    maintainer_email='cayhanerk@gmail.com',
    description='Planning and perception package for VRX autonomous navigation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_perception = brain.simple_perception:main',
            'mission_trigger = brain.mission_trigger:main',
            'tf_broadcaster = brain.tf_broadcaster:main',
            'tf_broadcaster_gps = brain.tf_broadcaster_gps:main',
            'tf_broadcaster_gazebo = brain.tf_broadcaster_gazebo:main',
            # Vostok1 - integrated navigation node
            'vostok1 = brain.vostok1:main',
            'waypoint_visualizer = brain.waypoint_visualizer:main',
            # Vostok1 CLI - terminal control for Vostok1/Sputnik
            'vostok1_cli = brain.vostok1_cli:main',
            # Modular nodes (TNO style names)
            'oko_perception = brain.oko_perception:main',
            'sputnik_planner = brain.sputnik_planner:main',
            # Atlantis - separated control and planning
            'atlantis_planner = brain.atlantis_planner:main',
            'pollutant_planner = brain.pollutant_planner:main',
            'atlantis_a_planner = plan.brain.atlantis_a_planner:main',
        ],
    },
)