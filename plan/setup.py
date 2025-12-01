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
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
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
            'apollo11 = brain.apollo11:main',
            'vostok1 = brain.vostok1:main',
            'waypoint_visualizer = brain.waypoint_visualizer:main',
            # Modular nodes (TNO style names)
            'oko_perception = brain.oko_perception:main',
            'sputnik_planner = brain.sputnik_planner:main',
            # Atlantis - separated control and planning
            'atlantis = brain.atlantis:main',
            'atlantis_planner = brain.atlantis_planner:main',
        ],
    },
)