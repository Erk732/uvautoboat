from setuptools import find_packages, setup

package_name = 'control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/all_in_one_bringup.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/hazard_world_boxes.yaml',
        ]),
    ],
    install_requires=[
        'setuptools',
        'numpy',
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
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # Modular controller (TNO style name)
            'buran_controller = control.buran_controller:main',

            #NEW CODE FOR SEPERATED CONTROL AND PLANNING
            'atlantis_controller = control.atlantis_controller:main',
            # Keyboard teleop for manual control
            'keyboard_teleop = control.keyboard_teleop:main',

            'all_in_one_stack = control.all_in_one_stack:main',
            'gps_imu_pose = control.gps_imu_pose:main',
            'pose_filter = control.pose_filter:main',
            # If you want you can comment its additional test
            'buran_controller_fixed = control.buran_controller_fixed:main',

        ],
    },
)
