"""Launch UR simulation with effort (torque) command interfaces.

This is similar to ur_sim_control.launch.py but generates a patched URDF
that includes effort command interfaces for Gazebo Ignition, enabling
external torque-based controllers (e.g. CRISP) to connect.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    ur_type = LaunchConfiguration("ur_type")
    safety_limits = LaunchConfiguration("safety_limits")
    safety_pos_margin = LaunchConfiguration("safety_pos_margin")
    safety_k_position = LaunchConfiguration("safety_k_position")
    prefix = LaunchConfiguration("prefix")
    start_joint_controller = LaunchConfiguration("start_joint_controller")
    initial_joint_controller = LaunchConfiguration("initial_joint_controller")
    launch_rviz = LaunchConfiguration("launch_rviz")
    gazebo_gui = LaunchConfiguration("gazebo_gui")
    world_file = LaunchConfiguration("world_file")
    controllers_file = LaunchConfiguration("controllers_file")

    # Use the provided controllers config (default: ur_sim_config's built-in)
    initial_joint_controllers = controllers_file

    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "rviz", "view_robot.rviz"]
    )

    # Generate URDF with effort interfaces via our patch script
    generate_script = PathJoinSubstitution(
        [FindPackageShare("ur_sim_config"), "urdf", "generate_effort_urdf.sh"]
    )

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="bash")]),
            " ",
            generate_script,
            " ",
            ur_type,
            " ",
            initial_joint_controllers,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(launch_rviz),
    )

    initial_joint_controller_spawner_started = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager", "--stopped"],
        condition=IfCondition(start_joint_controller),
    )
    initial_joint_controller_spawner_stopped = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[initial_joint_controller, "-c", "/controller_manager", "--stopped"],
        condition=UnlessCondition(start_joint_controller),
    )

    # GZ nodes
    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-string",
            robot_description_content,
            "-name",
            "ur",
            "-allow_renaming",
            "true",
        ],
    )
    gz_launch_description_with_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": [" -r -v 4 ", world_file]}.items(),
        condition=IfCondition(gazebo_gui),
    )
    gz_launch_description_without_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"]
        ),
        launch_arguments={"gz_args": [" -s -r -v 4 ", world_file]}.items(),
        condition=UnlessCondition(gazebo_gui),
    )

    gz_sim_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
        ],
        output="screen",
    )

    # Spawn forward_effort_controller (needed for gravity compensation)
    effort_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_effort_controller", "-c", "/controller_manager"],
    )

    # Spawn additional controllers as inactive (available for runtime switching)
    forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "-c", "/controller_manager", "--inactive"],
    )
    forward_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_velocity_controller", "-c", "/controller_manager", "--inactive"],
    )

    # Gravity compensation node — mimics real UR internal gravity comp.
    # Starts after forward_effort_controller is ready.
    gravity_compensation_node = Node(
        package="ur_sim_config",
        executable="gravity_compensation.py",
        name="gravity_compensation",
        output="screen",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    delay_gravity_comp_after_effort_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=effort_controller_spawner,
            on_exit=[gravity_compensation_node],
        ),
    )

    nodes_to_start = [
        robot_state_publisher_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        initial_joint_controller_spawner_stopped,
        initial_joint_controller_spawner_started,
        gz_spawn_entity,
        gz_launch_description_with_gui,
        gz_launch_description_without_gui,
        gz_sim_bridge,
        effort_controller_spawner,
        forward_position_controller_spawner,
        forward_velocity_controller_spawner,
        delay_gravity_comp_after_effort_controller,
    ]

    return nodes_to_start


def generate_launch_description():
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e",
            choices=["ur3", "ur5", "ur10", "ur3e", "ur5e", "ur7e",
                     "ur10e", "ur12e", "ur16e", "ur8long", "ur15", "ur18", "ur20", "ur30"],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("safety_limits", default_value="true")
    )
    declared_arguments.append(
        DeclareLaunchArgument("safety_pos_margin", default_value="0.15")
    )
    declared_arguments.append(
        DeclareLaunchArgument("safety_k_position", default_value="20")
    )
    declared_arguments.append(
        DeclareLaunchArgument("prefix", default_value='""')
    )
    declared_arguments.append(
        DeclareLaunchArgument("start_joint_controller", default_value="true")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "initial_joint_controller",
            default_value="joint_trajectory_controller",
            description="Controller to start. Options: joint_trajectory_controller, forward_effort_controller",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument("launch_rviz", default_value="false")
    )
    declared_arguments.append(
        DeclareLaunchArgument("gazebo_gui", default_value="false")
    )
    declared_arguments.append(
        DeclareLaunchArgument("world_file", default_value="empty.sdf")
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur_sim_config"), "config", "ur_effort_controllers.yaml"]
            ),
            description="Path to controllers YAML. Override to add custom controllers.",
        )
    )
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
