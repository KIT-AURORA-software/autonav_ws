#!/usr/bin/env python3
import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool
from tf2_ros import Buffer, TransformListener


def clamp(x: float, lo: float, hi: float) -> float:
    return max(lo, min(x, hi))


class GatePControllerTF(Node):
    def __init__(self):
        super().__init__("gate_p_controller_tf")

        self.declare_parameter("goal_topic", "/gate/forward")
        self.declare_parameter("stable_topic", "/gate/stable")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")

        self.declare_parameter("target_frame", "base_link")
        self.declare_parameter("hold_sec", 1.0)

        self.declare_parameter("kp_linear", 0.5)
        self.declare_parameter("kp_angular", 2.0)
        self.declare_parameter("max_linear", 0.35)
        self.declare_parameter("max_angular", 1.0)

        self.declare_parameter("rotate_only_angle", 0.35)
        self.declare_parameter("stop_dist", 0.15)
        self.declare_parameter("alpha", 0.3)

        self.goal_topic = self.get_parameter("goal_topic").value
        self.stable_topic = self.get_parameter("stable_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value

        self.target_frame = self.get_parameter("target_frame").value
        self.hold_sec = float(self.get_parameter("hold_sec").value)

        self.kp_linear = float(self.get_parameter("kp_linear").value)
        self.kp_angular = float(self.get_parameter("kp_angular").value)
        self.max_linear = float(self.get_parameter("max_linear").value)
        self.max_angular = float(self.get_parameter("max_angular").value)

        self.rotate_only_angle = float(self.get_parameter("rotate_only_angle").value)
        self.stop_dist = float(self.get_parameter("stop_dist").value)
        self.alpha = float(self.get_parameter("alpha").value)

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # state
        self.latest_goal: Optional[PoseStamped] = None
        self.last_stable_time: Optional[rclpy.time.Time] = None

        # filtered target in base_link
        self.fx: Optional[float] = None
        self.fy: Optional[float] = None

        self.create_subscription(PoseStamped, self.goal_topic, self.cb_goal, 10)
        self.create_subscription(Bool, self.stable_topic, self.cb_stable, 10)

        self.pub_cmd = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.timer = self.create_timer(0.05, self.control_loop)

        self.get_logger().info("GatePControllerTF started")

    def cb_goal(self, msg: PoseStamped):
        # 目標は常に“最後に来たもの”を保持（途切れても last_goal は残る）
        self.latest_goal = msg

    def cb_stable(self, msg: Bool):
        if msg.data:
            self.last_stable_time = self.get_clock().now()

    def stable_held(self) -> bool:
        if self.last_stable_time is None:
            return False
        dt = (self.get_clock().now() - self.last_stable_time).nanoseconds * 1e-9
        return dt <= self.hold_sec

    def goal_in_base(self) -> Optional[Tuple[float, float]]:
        if self.latest_goal is None:
            return None
        src = self.latest_goal.header.frame_id
        if not src:
            return None

        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, src, Time(),
                timeout=rclpy.duration.Duration(seconds=0.2)
            )
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed {self.target_frame}<-{src}: {e}")
            return None

        tx = tf.transform.translation.x
        ty = tf.transform.translation.y

        q = tf.transform.rotation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        gx = self.latest_goal.pose.position.x
        gy = self.latest_goal.pose.position.y

        bx = math.cos(yaw) * gx - math.sin(yaw) * gy + tx
        by = math.sin(yaw) * gx + math.cos(yaw) * gy + ty
        return bx, by

    def stop(self):
        self.pub_cmd.publish(Twist())

    def control_loop(self):
        if not self.stable_held():
            self.stop()
            return

        p = self.goal_in_base()
        if p is None:
            # stableホールド中でも、TFが一時的に引けないなら安全に停止
            self.stop()
            return

        x, y = p

        # LPF
        if self.fx is None:
            self.fx, self.fy = x, y
        else:
            self.fx = (1.0 - self.alpha) * self.fx + self.alpha * x
            self.fy = (1.0 - self.alpha) * self.fy + self.alpha * y

        dx, dy = self.fx, self.fy
        dist = math.hypot(dx, dy)
        ang = math.atan2(dy, dx)

        cmd = Twist()

        if dist < self.stop_dist:
            self.pub_cmd.publish(cmd)
            return

        if abs(ang) > self.rotate_only_angle:
            cmd.linear.x = 0.0
            cmd.angular.z = clamp(self.kp_angular * ang, -self.max_angular, self.max_angular)
        else:
            cmd.linear.x = clamp(self.kp_linear * dist, 0.0, self.max_linear)
            cmd.angular.z = clamp(self.kp_angular * ang, -self.max_angular, self.max_angular)

        self.pub_cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = GatePControllerTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()