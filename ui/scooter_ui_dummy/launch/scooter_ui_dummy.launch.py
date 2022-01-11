from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    scooter_core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            get_package_share_directory('scooter_core'),
            '/scooter_core.launch.py'])
    )

    return LaunchDescription([
        scooter_core_launch,
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
