import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # ros2 launch ur_bringup ur_moveit.launch.py ur_type:=ur5 robot_ip:=yyy.yyy.yyy.yyy use_fake_hardware:=true
    ur_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ur_bringup'), 'launch'),
            '/ur_control.launch.py']),
        launch_arguments={
            "ur_type": "ur5",
            "robot_ip": "yyy.yyy.yyy.yyy",
            "use_fake_hardware": "true"
        }.items(),
    )

    return LaunchDescription([
        ur_bringup
    ])
