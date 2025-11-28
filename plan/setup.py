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
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='VRX Navigation Logic',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_perception = brain.simple_perception:main',
            'astar_planner = brain.astar_planner:main',
            'mission_trigger = brain.mission_trigger:main',
            'avoidingobs_ts_planner = brain.avoidingobs_ts_planner:main',
            'tf_broadcaster = brain.tf_broadcaster:main',
            'path_follower = control.path_follower:main',
        ],
    },
)