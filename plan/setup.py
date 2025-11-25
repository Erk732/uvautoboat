from setuptools import setup
import os
from glob import glob

package_name = 'plan'      # ROS package name is 'plan'

setup(
    name=package_name,
    version='0.0.0',
    #  CHANGE: Tell python code is in the 'brain' folder
    packages=['brain'], 
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        #  ADD THIS: Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        # ADD THIS: Install rviz config files
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
            # CHANGE: Point to 'brain' module, not 'plan' or 'path'
            'simple_perception = brain.simple_perception:main',
            'astar_planner = brain.astar_planner:main',
        ],
    },
)