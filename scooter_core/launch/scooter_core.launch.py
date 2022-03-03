import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    scooter_fsm = LaunchDescription([
        Node(
            package="scooter_fsm",
            executable="scooter_fsm",
            name="scooter_fsm_node",
            output="screen"
        )
    ])

    scooter_manipulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('scooter_manipulation'), 'launch'),
            '/scooter_manipulation.launch.py'])
    )

    return LaunchDescription([
        scooter_fsm,
        scooter_manipulation,
    ])
