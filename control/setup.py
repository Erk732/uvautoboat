from setuptools import find_packages
from distutils.core import setup  # IMPORTANT: use distutils.setup for colcon / Python 3.12

package_name = 'control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/simple_controller.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ghostzero',
    maintainer_email='yinpuchen0@gamil.com',
    description='Control package for VRX WAM-V thruster-based path following.',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'simple_controller = control.simple_controller:main',
            'path_follower = control.path_follower:main',
        ],
    },
)
