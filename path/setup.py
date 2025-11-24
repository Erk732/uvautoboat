from setuptools import setup

package_name = 'path'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AutoBoat Team',
    maintainer_email='student@example.com',
    description='VRX Path Planning Node',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # THIS IS THE MISSING LINE:
            'simple_planner = path.simple_planner:main',
	    'astar_planner = path.astar_planner:main',
           'coverage_planner = path.coverage_planner:main',
        ],
    },
)
