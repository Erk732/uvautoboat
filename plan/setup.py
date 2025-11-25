from setuptools import setup
import os
from glob import glob  ### <--- 1. IMPORTANT: We import 'glob' to find files

package_name = 'plan'

setup(
    name=package_name,
    version='0.0.0',
    packages=['brain'], # This points to your 'brain' folder where code lives
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        ### <--- 2. ADD THIS LINE: Installs the launch folder
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        
        ### <--- 3. ADD THIS LINE: Installs the rviz folder
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
        ],
    },
)