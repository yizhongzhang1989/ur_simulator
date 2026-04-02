#!/usr/bin/env python3
"""Gravity compensation node for UR simulation in effort mode.

Mimics real UR robot behavior where gravity is internally compensated by
the robot firmware. When zero external torque is commanded, the robot holds
its position. External controllers (e.g. CRISP) can publish additional
torques on /external_effort_commands that get summed with gravity compensation
before being sent to the forward_effort_controller.

This node:
  1. Subscribes to /joint_states for current joint positions
  2. Computes gravity torques using KDL dynamics
  3. Subscribes to /external_effort_commands for additional torques
  4. Publishes (gravity + external) to /forward_effort_controller/commands
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory
import PyKDL
from urdf_parser_py.urdf import URDF
import numpy as np
import math


JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# Default UR home position (matches initial_positions.yaml)
HOME_POSITIONS = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]


def urdf_pose_to_kdl_frame(pose):
    """Convert a URDF pose to a KDL Frame."""
    if pose is None:
        return PyKDL.Frame()
    pos = pose.xyz if pose.xyz else [0, 0, 0]
    rot = pose.rpy if pose.rpy else [0, 0, 0]
    return PyKDL.Frame(
        PyKDL.Rotation.RPY(rot[0], rot[1], rot[2]),
        PyKDL.Vector(pos[0], pos[1], pos[2]),
    )


def urdf_inertial_to_kdl(inertial):
    """Convert URDF inertial to KDL RigidBodyInertia."""
    if inertial is None:
        return PyKDL.RigidBodyInertia()
    origin = urdf_pose_to_kdl_frame(inertial.origin)
    mass = inertial.mass if inertial.mass else 0.0
    inertia = inertial.inertia
    if inertia is None:
        return PyKDL.RigidBodyInertia(mass, origin.p)
    return PyKDL.RigidBodyInertia(
        mass,
        origin.p,
        PyKDL.RotationalInertia(
            inertia.ixx, inertia.iyy, inertia.izz,
            inertia.ixy, inertia.ixz, inertia.iyz,
        ),
    )


def build_kdl_chain(urdf_str, base_link, tip_link):
    """Build a KDL chain from URDF string between base_link and tip_link."""
    robot = URDF.from_xml_string(urdf_str)

    # Build parent map: child_link -> (joint, parent_link)
    parent_map = {}
    for j in robot.joints:
        parent_map[j.child] = (j, j.parent)

    # Walk from tip to base to find the chain links/joints
    chain_joints = []
    current = tip_link
    while current != base_link:
        if current not in parent_map:
            raise ValueError(f"Cannot find chain from {base_link} to {tip_link}")
        joint, parent = parent_map[current]
        chain_joints.append((joint, current))
        current = parent
    chain_joints.reverse()

    # Build KDL chain
    chain = PyKDL.Chain()
    for joint, child_link_name in chain_joints:
        frame = urdf_pose_to_kdl_frame(joint.origin)
        child_link = robot.link_map[child_link_name]
        inertia = urdf_inertial_to_kdl(child_link.inertial)

        if joint.type == "revolute" or joint.type == "continuous":
            axis = joint.axis if joint.axis else [0, 0, 1]
            kdl_joint = PyKDL.Joint(
                joint.name,
                PyKDL.Vector(0, 0, 0),
                PyKDL.Vector(axis[0], axis[1], axis[2]),
                PyKDL.Joint.RotAxis,
            )
        elif joint.type == "prismatic":
            axis = joint.axis if joint.axis else [0, 0, 1]
            kdl_joint = PyKDL.Joint(
                joint.name,
                PyKDL.Vector(0, 0, 0),
                PyKDL.Vector(axis[0], axis[1], axis[2]),
                PyKDL.Joint.TransAxis,
            )
        else:
            # Fixed joint
            kdl_joint = PyKDL.Joint(joint.name, PyKDL.Joint.Fixed)

        segment = PyKDL.Segment(child_link_name, kdl_joint, frame, inertia)
        chain.addSegment(segment)

    return chain


class GravityCompensation(Node):
    def __init__(self):
        super().__init__("gravity_compensation")

        # Get robot description from /robot_state_publisher
        self.declare_parameter("robot_description", "")
        urdf_str = (
            self.get_parameter("robot_description")
            .get_parameter_value()
            .string_value
        )

        if not urdf_str:
            self.get_logger().error(
                "No robot_description parameter provided. "
                "Pass it via launch file or remap."
            )
            raise RuntimeError("Missing robot_description")

        # Build KDL chain: base_link -> wrist_3_link (6 revolute joints)
        self.chain = build_kdl_chain(urdf_str, "base_link", "wrist_3_link")
        n_joints = self.chain.getNrOfJoints()
        self.get_logger().info(
            f"KDL chain built: {self.chain.getNrOfSegments()} segments, "
            f"{n_joints} joints"
        )

        if n_joints != 6:
            self.get_logger().error(
                f"Expected 6 joints, got {n_joints}. Check URDF chain."
            )
            raise RuntimeError(f"Unexpected joint count: {n_joints}")

        self.n_joints = n_joints
        self.gravity = PyKDL.Vector(0, 0, -9.81)
        self.dyn_param = PyKDL.ChainDynParam(self.chain, self.gravity)

        self.joint_positions = PyKDL.JntArray(self.n_joints)
        self.joint_velocities = [0.0] * self.n_joints
        self.external_torques = [0.0] * self.n_joints
        self.got_joint_states = False

        # Position hold: lock to home position initially
        self.hold_positions = list(HOME_POSITIONS)
        self.hold_initialized = True
        self.external_active = False

        # Trajectory interpolation state
        self.traj_start_positions = None
        self.traj_target_positions = None
        self.traj_duration = 0.0
        self.traj_start_time = None
        self.traj_active = False

        # Per-joint PID gains for position hold (mimics real UR servo stiffness)
        # Higher gains for heavy base joints, lower for light wrist joints
        self.kp = [1500.0, 1500.0, 1500.0, 100.0, 100.0, 100.0]
        self.ki = [50.0,   50.0,   50.0,   20.0,  20.0,  20.0]
        self.kd = [80.0,   80.0,   80.0,   10.0,  10.0,  10.0]

        # Per-joint effort limits (sim limits — slightly above real UR for
        # startup recovery; real HW: base=150, wrist=28)
        self.effort_limits = [150.0, 150.0, 150.0, 56.0, 56.0, 56.0]

        # Integral error accumulator
        self.integral_error = [0.0] * self.n_joints
        self.integral_limit = 50.0  # Anti-windup clamp

        # Subscribers
        self.create_subscription(
            JointState, "/joint_states", self._joint_state_cb, 10
        )
        self.create_subscription(
            Float64MultiArray,
            "/external_effort_commands",
            self._external_torque_cb,
            10,
        )
        self.create_subscription(
            JointTrajectory,
            "/joint_trajectory_controller/joint_trajectory",
            self._trajectory_cb,
            10,
        )

        # Publisher
        self.effort_pub = self.create_publisher(
            Float64MultiArray, "/forward_effort_controller/commands", 10
        )

        # Publish at 1000 Hz to match Gazebo simulation rate
        self.create_timer(0.001, self._publish_torques)
        self.get_logger().info("Gravity compensation active (1000 Hz)")

    def _joint_state_cb(self, msg):
        for i, name in enumerate(JOINT_NAMES):
            if name in msg.name:
                idx = msg.name.index(name)
                self.joint_positions[i] = msg.position[idx]
                if idx < len(msg.velocity):
                    self.joint_velocities[i] = msg.velocity[idx]
        self.got_joint_states = True

    def _external_torque_cb(self, msg):
        if len(msg.data) == self.n_joints:
            self.external_torques = list(msg.data)
            # If any non-zero external torque, update hold position to current
            if any(abs(t) > 0.01 for t in msg.data):
                self.external_active = True
                self.hold_positions = [self.joint_positions[i] for i in range(self.n_joints)]
            else:
                self.external_active = False
                # Lock to current position when external goes to zero
                self.hold_positions = [self.joint_positions[i] for i in range(self.n_joints)]

    def _trajectory_cb(self, msg):
        """Handle trajectory commands (from web dashboard or other sources).

        Interpolates from current position to the last trajectory point.
        """
        if not msg.points:
            return
        last_point = msg.points[-1]
        if len(last_point.positions) != self.n_joints:
            # Map by joint_names
            target = list(self.hold_positions)
            for i, name in enumerate(msg.joint_names):
                if name in JOINT_NAMES:
                    idx = JOINT_NAMES.index(name)
                    target[idx] = last_point.positions[i]
        else:
            # Reorder to match JOINT_NAMES
            target = [0.0] * self.n_joints
            for i, name in enumerate(msg.joint_names):
                if name in JOINT_NAMES:
                    idx = JOINT_NAMES.index(name)
                    target[idx] = last_point.positions[i]

        duration = (last_point.time_from_start.sec
                    + last_point.time_from_start.nanosec * 1e-9)
        if duration < 0.1:
            duration = 3.0  # default 3 seconds

        self.traj_start_positions = list(self.hold_positions)
        self.traj_target_positions = target
        self.traj_duration = duration
        self.traj_start_time = self.get_clock().now()
        self.traj_active = True
        # Reset integral errors for smooth transition
        self.integral_error = [0.0] * self.n_joints
        self.get_logger().info(
            f"Trajectory received: duration={duration:.1f}s, "
            f"target={[f'{t:.2f}' for t in target]}"
        )

    def _publish_torques(self):
        if not self.got_joint_states:
            return

        # Update hold position if a trajectory is active
        if self.traj_active and self.traj_start_time is not None:
            elapsed = (self.get_clock().now() - self.traj_start_time).nanoseconds * 1e-9
            alpha = min(elapsed / self.traj_duration, 1.0)
            # Smooth interpolation (cubic ease in-out)
            if alpha < 0.5:
                alpha_smooth = 4.0 * alpha * alpha * alpha
            else:
                alpha_smooth = 1.0 - (-2.0 * alpha + 2.0) ** 3 / 2.0
            for i in range(self.n_joints):
                self.hold_positions[i] = (
                    self.traj_start_positions[i]
                    + alpha_smooth * (self.traj_target_positions[i] - self.traj_start_positions[i])
                )
            if alpha >= 1.0:
                self.traj_active = False

        gravity_torques = PyKDL.JntArray(self.n_joints)
        self.dyn_param.JntToGravity(self.joint_positions, gravity_torques)

        msg = Float64MultiArray()
        torques = []
        for i in range(self.n_joints):
            tau = gravity_torques[i] + self.external_torques[i]
            # Add PID position hold + damping (always active, like real UR servo)
            pos_error = self.hold_positions[i] - self.joint_positions[i]
            self.integral_error[i] += pos_error * 0.001  # dt = 1ms
            self.integral_error[i] = max(-self.integral_limit,
                                         min(self.integral_limit, self.integral_error[i]))
            tau += (self.kp[i] * pos_error
                    + self.ki[i] * self.integral_error[i]
                    - self.kd[i] * self.joint_velocities[i])
            # Clamp to effort limits (matching real UR hardware)
            tau = max(-self.effort_limits[i], min(self.effort_limits[i], tau))
            torques.append(tau)
        msg.data = torques
        self.effort_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GravityCompensation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
