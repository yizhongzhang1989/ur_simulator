"""Launch UR simulation with MuJoCo physics engine (effort mode).

Uses mujoco_ros2_control as the physics backend instead of Gazebo.
Provides the same ROS 2 interface (joint_states, controllers) as the Gazebo version.
"""

import os
import subprocess
import tempfile

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue, ParameterFile
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    ur_type = LaunchConfiguration("ur_type").perform(context)
    control_mode = LaunchConfiguration("control_mode").perform(context)
    headless = LaunchConfiguration("headless").perform(context)
    launch_rviz = LaunchConfiguration("launch_rviz")
    controllers_file_cfg = LaunchConfiguration("controllers_file").perform(context)

    pkg_share = FindPackageShare("ur_sim_config").perform(context)

    # --- Generate MJCF model (shared for all control modes since we always use motor actuators) ---
    mjcf_dir = os.path.join(pkg_share, "mujoco", ur_type)
    scene_file = os.path.join(mjcf_dir, "ur_scene.xml")

    if not os.path.exists(scene_file):
        generate_script = os.path.join(pkg_share, "scripts", "generate_mujoco_model.sh")
        print(f"[MuJoCo] Generating MJCF model for {ur_type} ({control_mode})...")
        result = subprocess.run(
            ["bash", generate_script, ur_type, control_mode, mjcf_dir],
            capture_output=True, text=True
        )
        if result.returncode != 0:
            print(f"[MuJoCo] MJCF generation failed:\n{result.stderr}")
            raise RuntimeError("MJCF generation failed")
        print(f"[MuJoCo] MJCF generated: {scene_file}")

    # --- Build URDF with MuJoCo hardware interface ---
    # Generate a clean URDF for robot_state_publisher (TF tree)
    ur_desc_dir = subprocess.check_output(
        ["ros2", "pkg", "prefix", "ur_description"],
        text=True
    ).strip()
    xacro_file = os.path.join(ur_desc_dir, "share", "ur_description", "urdf", "ur.urdf.xacro")

    # Generate the URDF with ros2_control block pointing to MuJoCo
    robot_description_content = subprocess.check_output(
        ["xacro", xacro_file, f"ur_type:={ur_type}", "name:=ur"],
        text=True
    )

    # Inject MuJoCo ros2_control block into the URDF
    mujoco_ros2_control_block = _build_ros2_control_block(
        scene_file, control_mode, headless
    )

    # Replace any existing ros2_control block or insert before </robot>
    import re
    robot_description_content = re.sub(
        r'<ros2_control.*?</ros2_control>',
        mujoco_ros2_control_block,
        robot_description_content,
        flags=re.DOTALL,
    )
    if '<ros2_control' not in robot_description_content:
        robot_description_content = robot_description_content.replace(
            '</robot>',
            mujoco_ros2_control_block + '\n</robot>'
        )

    robot_description = {
        "robot_description": ParameterValue(
            robot_description_content, value_type=str
        )
    }

    # --- Nodes ---
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "rviz", "view_robot.rviz"]
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    # MuJoCo ros2_control node (replaces Gazebo)
    # PID gains for position control on motor actuators.
    # mujoco_ros2_control uses these to convert position commands → effort.
    # The gravity_compensation node provides feedforward torques, so PID only
    # handles tracking error — keep gains moderate to avoid oscillation.
    _jn = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
           "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
    _pg = {"shoulder_pan_joint": (800,20,80), "shoulder_lift_joint": (800,20,80),
           "elbow_joint": (800,20,80), "wrist_1_joint": (200,5,20),
           "wrist_2_joint": (200,5,20), "wrist_3_joint": (200,5,20)}
    pid_params = {}
    for name in _jn:
        p, i, d = _pg[name]
        pid_params[f"pid_gains.position.{name}.p"] = float(p)
        pid_params[f"pid_gains.position.{name}.i"] = float(i)
        pid_params[f"pid_gains.position.{name}.d"] = float(d)
        pid_params[f"pid_gains.position.{name}.i_clamp_max"] = 150.0
        pid_params[f"pid_gains.position.{name}.i_clamp_min"] = -150.0
        pid_params[f"pid_gains.position.{name}.antiwindup"] = True

    control_node = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        emulate_tty=True,
        output="both",
        parameters=[
            {"use_sim_time": True},
            ParameterFile(controllers_file_cfg),
            pid_params,
        ],
        remappings=[("~/robot_description", "/robot_description")],
        on_exit=Shutdown(),
    )

    # Controller spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(launch_rviz),
    )

    delay_rviz_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[rviz_node],
        ),
        condition=IfCondition(launch_rviz),
    )

    # Spawn joint_trajectory_controller as inactive
    joint_traj_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager", "--inactive"],
    )

    # MuJoCo always uses motor (effort) actuators. Both position and effort
    # modes use the same effort interface with gravity compensation.
    # In position mode, joint_trajectory_controller is active and sends
    # trajectories handled by the gravity_compensation node.
    # In effort mode, external controllers publish to /external_effort_commands.

    effort_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_effort_controller", "-c", "/controller_manager"],
    )

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

    # Gravity compensation (Pinocchio) — always runs in MuJoCo
    gravity_compensation_node = Node(
        package="ur_sim_config",
        executable="gravity_compensation.py",
        name="gravity_compensation",
        output="screen",
        parameters=[{"use_sim_time": True}, robot_description],
    )

    delay_gravity_comp = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=effort_controller_spawner,
            on_exit=[gravity_compensation_node],
        ),
    )

    return [
        robot_state_publisher_node,
        control_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        joint_traj_controller_spawner,
        effort_controller_spawner,
        forward_position_controller_spawner,
        forward_velocity_controller_spawner,
        delay_gravity_comp,
    ]


def _build_ros2_control_block(scene_file, control_mode, headless):
    """Build the ros2_control XML block for MuJoCo."""
    joints_xml = ""
    joint_names = [
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
    ]
    initial_positions = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]

    for name, init_pos in zip(joint_names, initial_positions):
        # Expose both position and effort command interfaces (like real UR driver)
        joints_xml += f"""
    <joint name="{name}">
      <command_interface name="position"/>
      <command_interface name="effort"/>
      <state_interface name="position">
        <param name="initial_value">{init_pos}</param>
      </state_interface>
      <state_interface name="velocity"/>
      <state_interface name="effort"/>
    </joint>"""

    return f"""
  <ros2_control name="MujocoSystem" type="system">
    <hardware>
      <plugin>mujoco_ros2_control/MujocoSystemInterface</plugin>
      <param name="mujoco_model">{scene_file}</param>
      <param name="headless">{headless}</param>
      <param name="sim_speed_factor">1.0</param>
      <param name="initial_keyframe">home</param>
    </hardware>
{joints_xml}
  </ros2_control>"""


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e",
            choices=["ur3", "ur5", "ur10", "ur3e", "ur5e", "ur7e",
                     "ur10e", "ur12e", "ur16e", "ur8long", "ur15", "ur18", "ur20", "ur30"],
        ),
        DeclareLaunchArgument(
            "control_mode",
            default_value="effort",
            choices=["position", "effort"],
        ),
        DeclareLaunchArgument("headless", default_value="true"),
        DeclareLaunchArgument("launch_rviz", default_value="false"),
        DeclareLaunchArgument(
            "controllers_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("ur_sim_config"), "config", "ur_effort_controllers.yaml"]
            ),
        ),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
