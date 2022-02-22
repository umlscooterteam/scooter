import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# common imports between ur_control.launch and ur_moveit.launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# imports unique to ur_control.launch
from launch.conditions import UnlessCondition
# imports unique to ur_moveit.launch
from ur_bringup.launch_common import load_yaml, load_yaml_abs



def launch_setup(context, *args, **kwargs):

    # Initialize Arguments
    ur_type = LaunchConfiguration("ur_type")
    robot_ip = LaunchConfiguration("robot_ip")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    # General arguments
    description_package = LaunchConfiguration("description_package")
    description_file = LaunchConfiguration("description_file")
    prefix = LaunchConfiguration("prefix")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")
    launch_rviz = LaunchConfiguration("launch_rviz")
    headless_mode = LaunchConfiguration("headless_mode")
    # Arguments unique to ur_control.launch
    runtime_config_package = LaunchConfiguration("runtime_config_package")
    controllers_file = LaunchConfiguration("controllers_file")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    activate_joint_controller = LaunchConfiguration("activate_joint_controller")
    launch_dashboard_client = LaunchConfiguration("launch_dashboard_client")
    use_tool_communication = LaunchConfiguration("use_tool_communication")
    # Arguments unique to Ur-moveit.launch
    moveit_config_package = LaunchConfiguration("moveit_config_package")
    moveit_config_file = LaunchConfiguration("moveit_config_file")
    launch_servo = LaunchConfiguration("launch_servo")


    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare(description_package), "config", ur_type, "visual_parameters.yaml"]
    )
    script_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "ros_control.urscript"]
    )
    input_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_input_recipe.txt"]
    )
    output_recipe_filename = PathJoinSubstitution(
        [FindPackageShare("ur_robot_driver"), "resources", "rtde_output_recipe.txt"]
    )


    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([FindPackageShare(description_package), "urdf", description_file]),
            " ",
            "robot_ip:=",
            robot_ip,
            " ",
            "joint_limit_params:=",
            joint_limit_params,
            " ",
            "kinematics_params:=",
            kinematics_params,
            " ",
            "physical_params:=",
            physical_params,
            " ",
            "visual_params:=",
            visual_params,
            " ",
            "safety_limits:=",
            safety_limits,
            " ",
            "safety_pos_margin:=",
            safety_pos_margin,
            " ",
            "safety_k_position:=",
            safety_k_position,
            " ",
            # DIFF
            "name:=",
            "ur",
            " ",
            "script_filename:=",
            script_filename,
            " ",
            "input_recipe_filename:=",
            input_recipe_filename,
            " ",
            "output_recipe_filename:=",
            output_recipe_filename,
            " ",
            "prefix:=",
            prefix,
            " ",
            "use_fake_hardware:=",
            use_fake_hardware,
            " ",
            "fake_sensor_commands:=",
            fake_sensor_commands,
            " ",
            "headless_mode:=",
            headless_mode,
            " ",
            # unique to ur_control.launch
            "use_tool_communication:=",
            use_tool_communication,
            " ",
        ]
    )
    robot_description = {"robot_description": robot_description_content}


    # unique to ur_control.launch
    initial_joint_controllers = PathJoinSubstitution(
        [FindPackageShare(runtime_config_package), "config", controllers_file]
    )


    # MoveIt Configuration (unique to ur_moveit.launch)
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare(moveit_config_package), "srdf", moveit_config_file]
            ),
            " ",
            "name:=",
            # Also ur_type parameter could be used but then the planning group names in yaml
            # configs has to be updated!
            "ur",
            " ",
            "prefix:=",
            prefix,
            " ",
        ]
    )
    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_content}

    kinematics_yaml = load_yaml("ur_moveit_config", "config/kinematics.yaml")
    robot_description_kinematics = {"robot_description_kinematics": kinematics_yaml}

    robot_description_planning = {
        "robot_description_planning": load_yaml_abs(str(joint_limit_params.perform(context)))
    }


    # Planning Configuration (unique to ur_moveit.launch)
    ompl_planning_pipeline_config = {
        "move_group": {
            "planning_plugin": "ompl_interface/OMPLPlanner",
            "request_adapters": """default_planner_request_adapters/AddTimeOptimalParameterization default_planner_request_adapters/FixWorkspaceBounds default_planner_request_adapters/FixStartStateBounds default_planner_request_adapters/FixStartStateCollision default_planner_request_adapters/FixStartStatePathConstraints""",
            "start_state_max_bounds_error": 0.1,
        }
    }
    ompl_planning_yaml = load_yaml("ur_moveit_config", "config/ompl_planning.yaml")
    ompl_planning_pipeline_config["move_group"].update(ompl_planning_yaml)


    # Trajectory Execution Configuration (unique to ur_moveit.launch)
    controllers_yaml = load_yaml("ur_moveit_config", "config/controllers.yaml")
    moveit_controllers = {
        "moveit_simple_controller_manager": controllers_yaml,
        "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
    }

    trajectory_execution = {
        "moveit_manage_controllers": False,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }


    # Start the actual move_group node/action server (unique to ur_moveit.launch)
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        # output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
    )


    # Warehouse mongodb server (unique to ur_moveit.launch)
    mongodb_server_node = Node(
        package="warehouse_ros_mongo",
        executable="mongo_wrapper_ros.py",
        parameters=[
            {"warehouse_port": 33829},
            {"warehouse_host": "localhost"},
            {"warehouse_plugin": "warehouse_ros_mongo::MongoDatabaseConnection"},
        ],
        # output="screen",
    )


    # rviz with moveit configuration (rviz config from ur_moveit.launch)
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare(moveit_config_package), "rviz", "view_robot.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        condition=IfCondition(launch_rviz),
        executable="rviz2",
        name="rviz2_moveit",
        # output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            ompl_planning_pipeline_config,
            robot_description_kinematics,
            robot_description_planning,
        ],
    )


    # Static TF (unique to ur_moveit.launch)
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        # output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # Servo node for realtime control (unique to ur_moveit.launch)
    servo_yaml = load_yaml("ur_moveit_config", "config/ur_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}
    servo_node = Node(
        package="moveit_servo",
        condition=IfCondition(launch_servo),
        executable="servo_node_main",
        parameters=[
            servo_params,
            robot_description,
            robot_description_semantic,
        ],
        # output={
        #     "stdout": "screen",
        #     "stderr": "screen",
        # },
    )


    # define update rate (unique to ur_control.launch)
    update_rate_config_file = PathJoinSubstitution(
        [
            FindPackageShare(runtime_config_package),
            "config",
            ur_type.perform(context) + "_update_rate.yaml",
        ]
    )


    # Nodes to start from ur_control.launch
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, update_rate_config_file, initial_joint_controllers],
        # output={
        #     "stdout": "screen",
        #     "stderr": "screen",
        # },
    )

    dashboard_client_node = Node(
        package="ur_robot_driver",
        condition=IfCondition(launch_dashboard_client),
        executable="dashboard_client",
        name="dashboard_client",
        # output="screen",
        emulate_tty=True,
        parameters=[{"robot_ip": robot_ip}],
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        # output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    io_and_status_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["io_and_status_controller", "-c", "/controller_manager"],
    )

    speed_scaling_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "speed_scaling_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    force_torque_sensor_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "force_torque_sensor_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    forward_position_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["forward_position_controller", "-c", "/controller_manager", "--stopped"],
    )

    # There may be other controllers of the joints, but this is the initially-started one
    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager"],
        condition=IfCondition(activate_joint_controller),
    )
    initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager", "--stopped"],
        condition=UnlessCondition(activate_joint_controller),
    )


    env_csv_path = PathJoinSubstitution([FindPackageShare("scooter_manipulation"), "config", "ur5_environment.csv"])
    # env_csv_path = os.path.join(
    #     get_package_share_directory('scooter_manipulation'),
    #     'config',
    #     'ur5_environment.csv'
    # )

    static_collision_object_publisher = Node(
        package="scooter_manipulation",
        executable="static_collision_object_publisher",
        name="static_collision_object_publisher",
        parameters=[
            {"env_csv": env_csv_path},
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
        # prefix=["gnome-terminal -- gdb -ex run --args"], # also set -g flag in add_compile_options to debug
        output={
            "stdout": "screen",
            "stderr": "screen",
        }
    )

    go_to_joint_config = Node(
        package="scooter_manipulation",
        executable="go_to_joint_config",
        name="go_to_joint_config",
        parameters=[
            {"env_csv": env_csv_path},
            robot_description,
            robot_description_semantic,
            robot_description_kinematics,
            robot_description_planning,
            ompl_planning_pipeline_config,
            trajectory_execution,
            moveit_controllers,
            planning_scene_monitor_parameters,
        ],
        # prefix=["gnome-terminal -- gdb -ex run --args"], # also set -g flag in add_compile_options to debug
        # prefix=["valgrind"],  # also set -g flag in add_compile_options to debug
        output={
            "stdout": "screen",
            "stderr": "screen",
        }
    )

    nodes_to_start = [
        static_collision_object_publisher,
        go_to_joint_config,

    	# following nodes from ur_control.launch
        control_node,
        dashboard_client_node,
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        io_and_status_controller_spawner,
        speed_scaling_state_broadcaster_spawner,
        force_torque_sensor_broadcaster_spawner,
        forward_position_controller_spawner_stopped,
        initial_joint_controller_spawner_stopped,
        initial_joint_controller_spawner_started,
        # following nodes from ur_moveit.launch
    	move_group_node,
    	mongodb_server_node,
    	static_tf,
    	servo_node,
        # following nodes shared in both launch files
        rviz_node
    ]

    return nodes_to_start


def generate_launch_description():

    declared_arguments = []
    # UR specific arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            description="Type/series of used UR robot.",
            choices=["ur3", "ur3e", "ur5", "ur5e", "ur10", "ur10e", "ur16e"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip", description="IP address by which the robot can be reached."
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_limits",
            default_value="true",
            description="Enables the safety limits controller if true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_pos_margin",
            default_value="0.15",
            description="The margin to lower and upper limits in the safety controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "safety_k_position",
            default_value="20",
            description="k-position factor in the safety controller.",
        )
    )
    # General arguments
    # this arg unique to ur_control.launch
    declared_arguments.append(
        DeclareLaunchArgument(
            "runtime_config_package",
            default_value="ur_bringup",
            description='Package with the controller\'s configuration in "config" folder. \
        Usually the argument is not set, it enables use of a custom setup.',
        )
    )
    # this arg unique to ur_control.launch
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value="ur_controllers.yaml",
            description="YAML file with the controllers configuration.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="ur_description",
            description="Description package with robot URDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="ur.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )
    # this arg unique to ur_moveit.launch
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_package",
            default_value="ur_moveit_config",
            description="MoveIt config package with robot SRDF/XACRO files. Usually the argument \
        is not set, it enables use of a custom moveit config.",
        )
    )
    # this arg unique to ur_moveit.launch
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_config_file",
            default_value="ur.srdf.xacro",
            description="MoveIt SRDF/XACRO description file with the robot.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "prefix",
            default_value='""',
            description="Prefix of the joint names, useful for \
        multi-robot setup. If changed than also joint names in the controllers' configuration \
        have to be updated.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Start robot with fake hardware mirroring command to its states.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "fake_sensor_commands",
            default_value="false",
            description="Enable fake command interfaces for sensors used for simple simulations. \
            Used only if 'use_fake_hardware' parameter is true.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless_mode",
            default_value="false",
            description="Enable headless mode for robot control",
        )
    )
    # this arg unique to ur_control.launch
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="joint_trajectory_controller",
            description="Initially loaded robot controller.",
        )
    )
    # this arg unique to ur_control.launch
    declared_arguments.append(
        DeclareLaunchArgument(
            "activate_joint_controller",
            default_value="true",
            description="Activate loaded joint controller.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="true", description="Launch RViz?")
    )
    # this arg unique to ur_moveit.launch
    declared_arguments.append(
        DeclareLaunchArgument("launch_servo", default_value="true", description="Launch Servo?")
    )
    # this arg unique to ur_control.launch
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_dashboard_client", default_value="true", description="Launch RViz?"
        )
    )
    # this arg unique to ur_control.launch
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_tool_communication",
            default_value="false",
            description="Only available for e series!",
        )
    )

    return LaunchDescription(declared_arguments + [OpaqueFunction(function=launch_setup)])


