import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    charuco_params = os.path.join(
        get_package_share_directory('ros2_charuco'),
        'config',
        'charuco_parameters.yaml'
        )

    charuco_node = Node(
        package='ros2_charuco',
        executable='charuco_node',
        parameters=[charuco_params]
    )

    return LaunchDescription([
        charuco_node
    ])
