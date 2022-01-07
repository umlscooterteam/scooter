import launch
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='scooter_ui_dummy',
            executable='wait_for_begin_service_server',
        ),
        Node(
            package='scooter_ui_dummy',
            executable='pick_selection_service_server',
        ),
        Node(
            package='scooter_ui_dummy',
            executable='pick_selection_confirm_service_server',
        ),
        Node(
            package='scooter_ui_dummy',
            executable='holding_object_service_server',
        ),
    ])
