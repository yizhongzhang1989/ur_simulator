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

    # --- Generate MJCF model per-(ur_type, control_mode) ---
    # Different control modes need different actuator types (motor for effort,
    # position for position), so scenes live in separate directories.
    mjcf_dir = os.path.join(pkg_share, "mujoco", ur_type, control_mode)
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

    # --- Build URDF via unified xacro (R5) ---
    # Replaces the previous inline regex/string-builder pipeline. Upstream
    # ur_description geometry is included via ur_macro.xacro with
    # generate_ros2_control_tag:=false, and ur_sim_config appends the
    # MuJoCo-specific <ros2_control> block.
    xacro_file = os.path.join(pkg_share, "urdf", "ur_sim.urdf.xacro")

    xacro_args = [
        "xacro", xacro_file,
        f"ur_type:={ur_type}",
        "name:=ur",
        "simulator:=mujoco",
        f"control_mode:={control_mode}",
        f"mjcf_model:={scene_file}",
        f"headless:={headless}",
        f"controllers_file:={controllers_file_cfg}",
        f"safety_limits:={LaunchConfiguration('safety_limits').perform(context)}",
        f"safety_pos_margin:={LaunchConfiguration('safety_pos_margin').perform(context)}",
        f"safety_k_position:={LaunchConfiguration('safety_k_position').perform(context)}",
        f"tf_prefix:={LaunchConfiguration('tf_prefix').perform(context)}",
    ]
    robot_description_content = subprocess.check_output(xacro_args, text=True)

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

    # Controller spawners.
    #
    # NOTE on startup serialisation: on Humble + FastRTPS, firing all spawners
    # concurrently can overflow the RMW response queue, so the controller
    # manager's reply to a ``load_controller`` call for one spawner is
    # silently dropped. That spawner then retries the load 10 s later and
    # hits a hard "A controller named X was already loaded" error because
    # Humble's ``load_controller`` is not idempotent, which aborts the
    # spawner and leaves its controller un-activated.
    #
    # To avoid this race we:
    #   1. Start ``joint_state_broadcaster`` on its own so it has the CM's
    #      full attention (it's the critical one for /joint_states, which
    #      gates every downstream test and every consumer of robot state).
    #   2. Use a longer ``--service-call-timeout`` on every spawner as
    #      defence in depth.
    #   3. Chain all other controller spawners off the JSB spawner's
    #      OnProcessExit, so at most one spawner is ever talking to the CM.
    _SPAWNER_TIMEOUT = ["--service-call-timeout", "30"]

    # Controller spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            *_SPAWNER_TIMEOUT,
        ],
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

    # Controller activation is mode-dependent:
    #   position mode -> JTC active (MuJoCo <position> actuators handle gravity
    #                    implicitly; gravity_compensation.py is NOT started).
    #   effort mode   -> forward_effort_controller active + gravity_compensation.py.
    position_mode = (control_mode == "position")

    effort_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "forward_effort_controller", "-c", "/controller_manager",
            *_SPAWNER_TIMEOUT,
            *(["--inactive"] if position_mode else []),
        ],
    )

    joint_traj_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller", "-c", "/controller_manager",
            *_SPAWNER_TIMEOUT,
            *([] if position_mode else ["--inactive"]),
        ],
    )

    scaled_joint_traj_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "scaled_joint_trajectory_controller", "-c", "/controller_manager",
            *_SPAWNER_TIMEOUT, "--inactive",
        ],
    )

    forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "forward_position_controller", "-c", "/controller_manager",
            *_SPAWNER_TIMEOUT, "--inactive",
        ],
    )
    forward_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "forward_velocity_controller", "-c", "/controller_manager",
            *_SPAWNER_TIMEOUT, "--inactive",
        ],
    )

    # Gravity compensation (Pinocchio) — only started in effort mode.
    # In position mode, MuJoCo <position> actuators handle gravity implicitly.
    _tf_prefix = LaunchConfiguration("tf_prefix").perform(context)
    gravity_compensation_node = Node(
        package="ur_sim_config",
        executable="gravity_compensation.py",
        name="gravity_compensation",
        output="screen",
        parameters=[
            {"use_sim_time": True, "ur_type": ur_type, "tf_prefix": _tf_prefix},
            robot_description,
        ],
    )

    # Upstream UR driver topic-parity shim (R3). Provides speed_scaling,
    # wrench, tcp_pose, robot/safety mode, program_running, io_states.
    sim_broadcasters_node = Node(
        package="ur_sim_config",
        executable="sim_broadcasters.py",
        name="sim_broadcasters",
        output="log",
        parameters=[{"use_sim_time": True, "tf_prefix": _tf_prefix}],
    )

    delay_gravity_comp = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=effort_controller_spawner,
            on_exit=[gravity_compensation_node],
        ),
    )

    # Chain the other controller spawners off the JSB spawner's exit to
    # serialise startup (see startup-serialisation note above).
    _other_spawners = [
        joint_traj_controller_spawner,
        scaled_joint_traj_controller_spawner,
        effort_controller_spawner,
        forward_position_controller_spawner,
        forward_velocity_controller_spawner,
    ]
    delay_other_spawners_after_jsb = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=_other_spawners,
        ),
    )

    nodes = [
        robot_state_publisher_node,
        control_node,
        joint_state_broadcaster_spawner,
        delay_rviz_after_joint_state_broadcaster_spawner,
        delay_other_spawners_after_jsb,
        sim_broadcasters_node,
    ]
    if not position_mode:
        nodes.append(delay_gravity_comp)
    return nodes


def _build_ros2_control_block(*_, **__):
    # Retained only to avoid breaking any external import; R5 moved the
    # <ros2_control> block into urdf/ur_sim.urdf.xacro.
    raise NotImplementedError(
        "R5 moved the ros2_control block into urdf/ur_sim.urdf.xacro; "
        "call the xacro directly instead."
    )


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
        # R5: forward upstream safety_limits knobs into the xacro.
        DeclareLaunchArgument("safety_limits",     default_value="false"),
        DeclareLaunchArgument("safety_pos_margin", default_value="0.15"),
        DeclareLaunchArgument("safety_k_position", default_value="20"),
        DeclareLaunchArgument("tf_prefix",         default_value=""),
    ]

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)]
    )
