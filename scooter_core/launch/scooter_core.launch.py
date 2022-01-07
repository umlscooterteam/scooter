from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="scooter_fsm",
            namespace="scooter_fsm",
            executable="scooter_fsm",
            name="scooter_fsm_node"
        )
    ])
