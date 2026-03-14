#!/usr/bin/env python3
"""
aruco_gate_detector.py

/aruco_markers (ros2_aruco_interfaces/msg/ArucoMarkers) から
左右マーカーIDを指定して取得し、target_frame(map/odom)にTF変換して

- ゲート中点 pM
- 抜け点 pF (= pM + n * forward_dist)

を publish する。

Publish:
- /gate/center   (geometry_msgs/PoseStamped)
- /gate/forward  (geometry_msgs/PoseStamped)
- /gate/stable   (std_msgs/Bool)
- /gate/debug_markers (visualization_msgs/MarkerArray)  # 任意

Notes:
- header.frame_id はカメラ座標系が多いので TF 変換必須
- pF の法線向きは「ロボット→中点方向」との内積で自動選択
"""

import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, Pose
from visualization_msgs.msg import Marker, MarkerArray

from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose  # geometry_msgs変換ユーティリティ

from ros2_aruco_interfaces.msg import ArucoMarkers
from rclpy.time import Time


def norm2(x: float, y: float) -> float:
    return math.sqrt(x * x + y * y)


def normalize(x: float, y: float) -> Tuple[float, float]:
    n = norm2(x, y)
    if n < 1e-9:
        return 0.0, 0.0
    return x / n, y / n


class ArucoGateDetector(Node):
    def __init__(self):
        super().__init__("aruco_gate_detector")

        # ===== params =====
        self.declare_parameter("left_id", 10)
        self.declare_parameter("right_id", 11)
        self.declare_parameter("target_frame", "odom")  # or "map"
        self.declare_parameter("base_frame", "base_link")  # used to choose forward direction
        self.declare_parameter("forward_dist", 1.5)  # [m]
        self.declare_parameter("stable_required_count", 8)  # consecutive frames
        self.declare_parameter("lost_hold_sec", 0.5)  # keep stable for a bit after loss
        self.declare_parameter("min_gate_width", 0.2)  # sanity
        self.declare_parameter("max_gate_width", 5.0)  # sanity
        self.declare_parameter("publish_debug_markers", True)

        self.left_id = int(self.get_parameter("left_id").value)
        self.right_id = int(self.get_parameter("right_id").value)
        self.target_frame = str(self.get_parameter("target_frame").value)
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.forward_dist = float(self.get_parameter("forward_dist").value)
        self.stable_required_count = int(self.get_parameter("stable_required_count").value)
        self.lost_hold_sec = float(self.get_parameter("lost_hold_sec").value)
        self.min_gate_width = float(self.get_parameter("min_gate_width").value)
        self.max_gate_width = float(self.get_parameter("max_gate_width").value)
        self.publish_debug_markers = bool(self.get_parameter("publish_debug_markers").value)

        # ===== tf =====
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ===== pubs/subs =====
        self.sub = self.create_subscription(ArucoMarkers, "/aruco_markers", self.cb_markers, 10)

        self.pub_center = self.create_publisher(PoseStamped, "/gate/center", 10)
        self.pub_forward = self.create_publisher(PoseStamped, "/gate/forward", 10)
        self.pub_stable = self.create_publisher(Bool, "/gate/stable", 10)

        self.pub_dbg = self.create_publisher(MarkerArray, "/gate/debug_markers", 10)

        # ===== state =====
        self.stable_count = 0
        self.last_seen_time = None  # rclpy.time.Time
        self.current_stable = False

        self.get_logger().info(
            f"Started. left_id={self.left_id}, right_id={self.right_id}, "
            f"target_frame={self.target_frame}, base_frame={self.base_frame}"
        )

    def find_pose_by_id(self, msg: ArucoMarkers, marker_id: int) -> Optional[Pose]:
        """msg.marker_ids と msg.poses を index で対応させて検索"""
        for i, mid in enumerate(msg.marker_ids):
            if int(mid) == int(marker_id):
                if i < len(msg.poses):
                    return msg.poses[i]
        return None

    def transform_pose(self, pose_in: Pose, src_frame: str, dst_frame: str, stamp) -> Optional[Pose]:
        """Pose を tf2 で dst_frame に変換（PoseStampedを介す）"""
        ps = PoseStamped()
        ps.header.frame_id = src_frame
        ps.header.stamp = stamp
        ps.pose = pose_in

        try:
            tf = self.tf_buffer.lookup_transform(
                dst_frame,
                src_frame,
                Time(),  # ← 最新のTFを使う
                timeout=rclpy.duration.Duration(seconds=0.2)
            )
            ps_out = do_transform_pose(ps, tf)
            return ps_out.pose                  # ← Pose を返すのがポイント
        except Exception as e:
            self.get_logger().warn(f"TF transform failed {src_frame}->{dst_frame}: {e}")
            return None

    def get_robot_xy(self, frame: str, stamp) -> Optional[Tuple[float, float]]:
        """target_frame上でのロボット(base_frame)位置を取得（向き決定用）"""
        try:
            tf = self.tf_buffer.lookup_transform(frame, self.base_frame, stamp, timeout=rclpy.duration.Duration(seconds=0.2))
            return float(tf.transform.translation.x), float(tf.transform.translation.y)
        except Exception as e:
            self.get_logger().warn(f"TF lookup failed {frame}<-{self.base_frame}: {e}")
            return None

    def publish_debug(self, frame_id: str, stamp, pL, pR, pM, pF):
        if not self.publish_debug_markers:
            return
        ma = MarkerArray()

        def mk_sphere(ns: str, mid: int, x: float, y: float, z: float):
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp = stamp
            m.ns = ns
            m.id = mid
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = z
            m.pose.orientation.w = 1.0
            m.scale.x = 0.15
            m.scale.y = 0.15
            m.scale.z = 0.15
            # 色はRViz側で見える程度（固定）
            m.color.a = 1.0
            return m

        # left/right/center/forward
        ma.markers.append(mk_sphere("gate", 0, pL[0], pL[1], 0.0))
        ma.markers.append(mk_sphere("gate", 1, pR[0], pR[1], 0.0))
        ma.markers.append(mk_sphere("gate", 2, pM[0], pM[1], 0.0))
        ma.markers.append(mk_sphere("gate", 3, pF[0], pF[1], 0.0))

        self.pub_dbg.publish(ma)

    def cb_markers(self, msg: ArucoMarkers):
        stamp = msg.header.stamp
        src_frame = msg.header.frame_id if msg.header.frame_id else "camera_link_optical"

        poseL = self.find_pose_by_id(msg, self.left_id)
        poseR = self.find_pose_by_id(msg, self.right_id)

        now = self.get_clock().now()

        if poseL is None or poseR is None:
            # 見失い処理（hold）
            if self.last_seen_time is not None:
                dt = (now - self.last_seen_time).nanoseconds * 1e-9
                if dt <= self.lost_hold_sec:
                    # stable状態は保持
                    self.pub_stable.publish(Bool(data=self.current_stable))
                    return

            self.stable_count = 0
            self.current_stable = False
            self.pub_stable.publish(Bool(data=False))
            return

        # TF変換して target_frame に揃える
        poseL_t = self.transform_pose(poseL, src_frame, self.target_frame, stamp)
        poseR_t = self.transform_pose(poseR, src_frame, self.target_frame, stamp)
        if poseL_t is None or poseR_t is None:
            self.pub_stable.publish(Bool(data=False))
            return

        pL = (poseL_t.position.x, poseL_t.position.y)
        pR = (poseR_t.position.x, poseR_t.position.y)

        # gate width sanity
        w = norm2(pR[0] - pL[0], pR[1] - pL[1])
        if w < self.min_gate_width or w > self.max_gate_width:
            self.get_logger().warn(f"Gate width sanity fail: {w:.2f} m (min={self.min_gate_width}, max={self.max_gate_width})")
            self.stable_count = 0
            self.current_stable = False
            self.pub_stable.publish(Bool(data=False))
            return

        # 中点
        pM = ((pL[0] + pR[0]) * 0.5, (pL[1] + pR[1]) * 0.5)

        # ゲート方向ベクトル g = R-L
        gx = pR[0] - pL[0]
        gy = pR[1] - pL[1]

        # 法線候補（±90度回転）
        nx1, ny1 = normalize(-gy, gx)
        nx2, ny2 = -nx1, -ny1

        # ロボット位置（target_frame上）
        robot_xy = self.get_robot_xy(self.target_frame, stamp)
        if robot_xy is None:
            # 取れないならとりあえずnx1側
            nx, ny = nx1, ny1
        else:
            rx, ry = robot_xy
            # ロボット→中点ベクトル
            vx = pM[0] - rx
            vy = pM[1] - ry
            # 「中点の先に進む」向きを選ぶ：内積が正になる方
            # (= ロボットから見て、中点方向と同じ側の法線)
            dot1 = vx * nx1 + vy * ny1
            dot2 = vx * nx2 + vy * ny2
            nx, ny = (nx1, ny1) if dot1 >= dot2 else (nx2, ny2)

        # 抜け点
        pF = (pM[0] + nx * self.forward_dist, pM[1] + ny * self.forward_dist)

        # 安定判定更新
        self.last_seen_time = now
        self.stable_count += 1
        self.current_stable = self.stable_count >= self.stable_required_count

        # publish PoseStamped
        center = PoseStamped()
        center.header.frame_id = self.target_frame
        center.header.stamp = stamp
        center.pose.position.x = float(pM[0])
        center.pose.position.y = float(pM[1])
        center.pose.position.z = 0.0
        center.pose.orientation.w = 1.0

        forward = PoseStamped()
        forward.header.frame_id = self.target_frame
        forward.header.stamp = stamp
        forward.pose.position.x = float(pF[0])
        forward.pose.position.y = float(pF[1])
        forward.pose.position.z = 0.0
        forward.pose.orientation.w = 1.0

        self.pub_center.publish(center)
        self.pub_forward.publish(forward)
        self.pub_stable.publish(Bool(data=self.current_stable))

        self.publish_debug(self.target_frame, stamp, pL, pR, pM, pF)

        # log (throttle)
        self.get_logger().info(
            f"gate: w={w:.2f} stable={self.current_stable} cnt={self.stable_count} "
            f"center=({pM[0]:.2f},{pM[1]:.2f}) forward=({pF[0]:.2f},{pF[1]:.2f})",
            throttle_duration_sec=1.0
        )


def main(args=None):
    rclpy.init(args=args)
    node = ArucoGateDetector()
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