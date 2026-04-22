#!/usr/bin/env python3
"""Stub broadcasters providing upstream UR driver topic parity.

Publishes the topics that `ur_controllers/{SpeedScalingStateBroadcaster,
ForceTorqueSensorBroadcaster,PoseBroadcaster,GPIOController}` emit on a real
UR, but without requiring the corresponding ros2_control hardware state
interfaces. This unblocks clients (MoveIt, ur_robot_driver test utilities,
dashboards) that subscribe to those topics.

Replace with proper ros2_control broadcasters once the hardware plugin
exposes the matching state interfaces (SHORTCOMINGS.md #1).
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import WrenchStamped, PoseStamped
from tf2_ros import Buffer, TransformListener
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException

try:
    from ur_msgs.msg import IOStates
    HAVE_UR_MSGS = True
except Exception:
    HAVE_UR_MSGS = False

try:
    from ur_dashboard_msgs.msg import RobotMode, SafetyMode
    HAVE_UR_DASHBOARD = True
except Exception:
    HAVE_UR_DASHBOARD = False


class SimBroadcasters(Node):
    def __init__(self):
        super().__init__("sim_broadcasters")

        self.declare_parameter("tf_prefix", "")
        self.declare_parameter("base_frame", "base")
        self.declare_parameter("tcp_frame", "tool0")

        prefix = self.get_parameter("tf_prefix").get_parameter_value().string_value
        self.base_frame = prefix + self.get_parameter("base_frame").get_parameter_value().string_value
        self.tcp_frame = prefix + self.get_parameter("tcp_frame").get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.pub_speed = self.create_publisher(
            Float64, "/speed_scaling_state_broadcaster/speed_scaling", 10)
        self.pub_wrench = self.create_publisher(
            WrenchStamped, "/force_torque_sensor_broadcaster/wrench", 10)
        self.pub_tcp = self.create_publisher(
            PoseStamped, "/tcp_pose_broadcaster/pose", 10)
        self.pub_prog = self.create_publisher(
            Bool, "/io_and_status_controller/program_running", 10)

        self.pub_io = None
        if HAVE_UR_MSGS:
            self.pub_io = self.create_publisher(
                IOStates, "/io_and_status_controller/io_states", 10)

        self.pub_mode = None
        self.pub_safety = None
        if HAVE_UR_DASHBOARD:
            self.pub_mode = self.create_publisher(
                RobotMode, "/io_and_status_controller/robot_mode", 10)
            self.pub_safety = self.create_publisher(
                SafetyMode, "/io_and_status_controller/safety_mode", 10)

        # 100 Hz for fast state (speed, wrench, tcp pose)
        self.create_timer(0.01, self._publish_fast)
        # 10 Hz for metadata (io, mode, program)
        self.create_timer(0.1, self._publish_slow)

        self.get_logger().info(
            "sim_broadcasters active: tcp=%s base=%s ur_msgs=%s ur_dashboard=%s",
            *[self.tcp_frame, self.base_frame, HAVE_UR_MSGS, HAVE_UR_DASHBOARD]
        ) if False else self.get_logger().info(
            f"sim_broadcasters active: tcp={self.tcp_frame} base={self.base_frame} "
            f"ur_msgs={HAVE_UR_MSGS} ur_dashboard={HAVE_UR_DASHBOARD}"
        )

    def _publish_fast(self):
        now = self.get_clock().now().to_msg()

        speed = Float64()
        speed.data = 1.0
        self.pub_speed.publish(speed)

        w = WrenchStamped()
        w.header.stamp = now
        w.header.frame_id = self.tcp_frame
        self.pub_wrench.publish(w)

        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame, self.tcp_frame, rclpy.time.Time(),
                timeout=Duration(seconds=0.0))
            p = PoseStamped()
            p.header.stamp = now
            p.header.frame_id = self.base_frame
            p.pose.position.x = t.transform.translation.x
            p.pose.position.y = t.transform.translation.y
            p.pose.position.z = t.transform.translation.z
            p.pose.orientation = t.transform.rotation
            self.pub_tcp.publish(p)
        except (LookupException, ConnectivityException, ExtrapolationException):
            pass

    def _publish_slow(self):
        prog = Bool()
        prog.data = True
        self.pub_prog.publish(prog)

        if self.pub_mode is not None:
            rm = RobotMode()
            # RobotMode.RUNNING == 7 on upstream; use attr if present.
            rm.mode = getattr(RobotMode, "RUNNING", 7)
            self.pub_mode.publish(rm)
        if self.pub_safety is not None:
            sm = SafetyMode()
            sm.mode = getattr(SafetyMode, "NORMAL", 1)
            self.pub_safety.publish(sm)

        if self.pub_io is not None:
            io = IOStates()
            self.pub_io.publish(io)


def main():
    rclpy.init()
    node = SimBroadcasters()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
