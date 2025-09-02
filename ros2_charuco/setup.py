from setuptools import setup
import os
from glob import glob

package_name = 'ros2_charuco'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ning Ma',
    maintainer_email='ningma@example.com',
    description='ROS2 Charuco Marker Tracking with Visualization',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'charuco_node = ros2_charuco.charuco_node:main',
            'charuco_generate_marker = ros2_charuco.charuco_generate_marker:main',
            'aruco_visualization_node = ros2_charuco.aruco_visualization_node:main',
            'simple_aruco_visualization_node = ros2_charuco.simple_aruco_visualization_node:main'
        ],
    },
)
