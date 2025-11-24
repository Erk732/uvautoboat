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
    description='VRX Path Planning',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_perception = path.simple_perception:main',
            'astar_planner = path.astar_planner:main',
        ],
    },
)
