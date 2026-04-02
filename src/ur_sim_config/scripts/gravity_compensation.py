#!/usr/bin/env python3
"""Gravity compensation node for UR simulation in effort mode.

Mimics real UR robot behavior where gravity is internally compensated by
the robot firmware. When zero external torque is commanded, the robot holds
its position. External controllers (e.g. CRISP) can publish additional
torques on /external_effort_commands that get summed with gravity compensation
before being sent to the forward_effort_controller.

Uses Pinocchio for rigid body dynamics (gravity torque computation).

This node:
  1. Subscribes to /joint_states for current joint positions
  2. Computes gravity torques using Pinocchio
  3. Subscribes to /external_effort_commands for additional torques
  4. Subscribes to /joint_trajectory_controller/joint_trajectory for motion
  5. Publishes (gravity + PID + external) to /forward_effort_controller/commands
"""

import tempfile
import os

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory
import pinocchio
import numpy as np


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


class GravityCompensation(Node):
    def __init__(self):
        super().__init__("gravity_compensation")

        # Get robot description
        self.declare_parameter("robot_description", "")
        urdf_str = (
            self.get_parameter("robot_description")
            .get_parameter_value()
            .string_value
        )

        if not urdf_str:
            self.get_logger().error("No robot_description parameter provided.")
            raise RuntimeError("Missing robot_description")

        # Build Pinocchio model from URDF string
        # Write to temp file since pinocchio.buildModelFromXML may not be available
        with tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False) as f:
            f.write(urdf_str)
            urdf_path = f.name

        try:
            self.model = pinocchio.buildModelFromUrdf(urdf_path)
        finally:
            os.unlink(urdf_path)

        self.data = self.model.createData()

        # Find indices of the 6 UR joints in the Pinocchio model
        self.pin_joint_ids = []
        for name in JOINT_NAMES:
            if self.model.existJointName(name):
                self.pin_joint_ids.append(self.model.getJointId(name))
            else:
                self.get_logger().error(f"Joint '{name}' not found in Pinocchio model")
                raise RuntimeError(f"Missing joint: {name}")

        self.n_joints = len(JOINT_NAMES)
        self.get_logger().info(
            f"Pinocchio model: {self.model.nq} DOF, {self.model.njoints} joints"
        )

        self.joint_positions = np.zeros(self.n_joints)
        self.joint_velocities = np.zeros(self.n_joints)
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
        # Pinocchio handles gravity accurately, so PID only corrects residual error
        self.kp = [100.0, 100.0, 100.0, 20.0, 20.0, 20.0]
        self.ki = [10.0,  10.0,  10.0,  5.0,  5.0,  5.0]
        self.kd = [20.0,  20.0,  20.0,  3.0,  3.0,  3.0]

        # Per-joint effort limits (matching real UR hardware)
        self.effort_limits = [150.0, 150.0, 150.0, 28.0, 28.0, 28.0]

        # Integral error accumulator
        self.integral_error = [0.0] * self.n_joints
        self.integral_limit = 20.0  # Anti-windup clamp
        self.dt = 0.002  # 500 Hz control rate

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

        # Publish at 500 Hz
        self.create_timer(self.dt, self._publish_torques)
        self.get_logger().info("Gravity compensation active (500 Hz)")

    def _joint_state_cb(self, msg):
        alpha = 0.3  # low-pass filter coefficient for velocity
        for i, name in enumerate(JOINT_NAMES):
            if name in msg.name:
                idx = msg.name.index(name)
                self.joint_positions[i] = msg.position[idx]
                if idx < len(msg.velocity):
                    # Low-pass filter on velocity to reduce noise
                    raw_vel = msg.velocity[idx]
                    self.joint_velocities[i] = (
                        alpha * raw_vel + (1.0 - alpha) * self.joint_velocities[i]
                    )
        self.got_joint_states = True

    def _external_torque_cb(self, msg):
        if len(msg.data) == self.n_joints:
            self.external_torques = list(msg.data)
            # If any non-zero external torque, update hold position to current
            if any(abs(t) > 0.01 for t in msg.data):
                self.external_active = True
                self.hold_positions = list(self.joint_positions)
            else:
                self.external_active = False
                # Lock to current position when external goes to zero
                self.hold_positions = list(self.joint_positions)

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

        # Compute gravity torques using Pinocchio
        # Build full q vector for Pinocchio model (may have more joints than 6)
        q_full = pinocchio.neutral(self.model)
        for i, jid in enumerate(self.pin_joint_ids):
            idx_q = self.model.joints[jid].idx_q
            q_full[idx_q] = self.joint_positions[i]

        gravity_torques_full = pinocchio.computeGeneralizedGravity(
            self.model, self.data, q_full
        )

        # Extract gravity torques for our 6 joints
        gravity_torques = np.zeros(self.n_joints)
        for i, jid in enumerate(self.pin_joint_ids):
            idx_v = self.model.joints[jid].idx_v
            gravity_torques[i] = gravity_torques_full[idx_v]

        msg = Float64MultiArray()
        torques = []
        for i in range(self.n_joints):
            # PID position hold (only corrects residual, gravity is from Pinocchio)
            pos_error = self.hold_positions[i] - self.joint_positions[i]
            self.integral_error[i] += pos_error * self.dt  # integrate
            self.integral_error[i] = max(-self.integral_limit,
                                         min(self.integral_limit, self.integral_error[i]))
            pid_tau = (self.kp[i] * pos_error
                       + self.ki[i] * self.integral_error[i]
                       - self.kd[i] * self.joint_velocities[i])
            # Clamp PID contribution to prevent spikes
            pid_limit = self.effort_limits[i] * 0.5
            pid_tau = max(-pid_limit, min(pid_limit, pid_tau))

            tau = gravity_torques[i] + self.external_torques[i] + pid_tau
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
