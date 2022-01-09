from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    get_cloud_action_server = Node(
        package='scooter_perception',
        executable='get_cloud_action_server',
    )
    segmentation_action_server = Node(
        package='scooter_perception',
        executable='segmentation_action_server',
    )
    scooter_fsm = Node(
        package="scooter_fsm",
        executable="scooter_fsm",
    )

    return LaunchDescription([
        get_cloud_action_server,
        segmentation_action_server,
        scooter_fsm,
    ])
