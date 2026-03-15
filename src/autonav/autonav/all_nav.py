#!/usr/bin/env python3

import math
import csv
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix, Imu
from ros2_aruco_interfaces.msg import ArucoMarkers
import tf_transformations


def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quat(q) -> float:
    x, y, z, w = q.x, q.y, q.z, q.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def load_markers(csv_path: str):
    markers = []
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f, skipinitialspace=True)

        if reader.fieldnames is None:
            raise RuntimeError("CSV header not found")
        reader.fieldnames = [h.strip() for h in reader.fieldnames]

        for row in reader:
            row = {k.strip(): v for k, v in row.items()}
            markers.append({
                "aruco_id": int(str(row["aruco_id"]).strip()),
                "pair_id": int(str(row["pair_id"]).strip()),
                "lat": float(str(row["latitude_north"]).strip()),
                "lon": float(str(row["longitude_east"]).strip()),
            })
    return markers


def build_gate_midpoints_from_partner_id(markers):
    """
    pair_id は「相方aruco_id」
    A->B と B->A がそろっていたらゲート中点を作る
    """
    by_id = {m["aruco_id"]: m for m in markers}
    gates = []
    seen = set()

    for m in markers:
        a = m["aruco_id"]
        b = m["pair_id"]

        if a == b:
            continue
        if b not in by_id:
            continue

        key = (min(a, b), max(a, b))
        if key in seen:
            continue

        mb = by_id[b]
        if mb.get("pair_id") != a:
            continue

        lat_mid = 0.5 * (m["lat"] + mb["lat"])
        lon_mid = 0.5 * (m["lon"] + mb["lon"])

        gates.append({
            "pair_id": f"{key[0]}-{key[1]}",
            "gate": key,   # (a,b)
            "a": key[0],
            "b": key[1],
            "lat": lat_mid,
            "lon": lon_mid,
        })
        seen.add(key)

    gates.sort(key=lambda g: g["gate"])
    return gates


class UnifiedGateNavigator(Node):
    def __init__(self):
        super().__init__("unified_gate_navigator")

        # -------------------------
        # parameters
        # -------------------------
        self.declare_parameter("csv_path", "")
        csv_path = self.get_parameter("csv_path").get_parameter_value().string_value
        if not csv_path:
            self.get_logger().error("csv_path parameter is empty.")
            raise RuntimeError("csv_path is required")

        markers = load_markers(csv_path)
        self.gates = build_gate_midpoints_from_partner_id(markers)

        if len(self.gates) == 0:
            self.get_logger().error("No valid gates found in CSV.")
            raise RuntimeError("No valid gates")

        self.get_logger().info(f"CSV loaded: {csv_path}")
        self.get_logger().info(f"Loaded {len(markers)} markers.")
        self.get_logger().info(f"Built {len(self.gates)} gates.")

        # -------------------------
        # publishers / subscribers
        # -------------------------
        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.aruco_sub = self.create_subscription(
            ArucoMarkers,
            "/aruco_markers",
            self.marker_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )

        self.gps_sub = self.create_subscription(
            NavSatFix,
            "/fix",
            self.gps_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            "/imu",
            self.imu_callback,
            50
        )

        self.timer = self.create_timer(0.05, self.control_loop)

        # -------------------------
        # shared navigation state
        # -------------------------
        self.current_gate = 0

        # current target gate midpoint (GPS)
        self.target_lat = None
        self.target_lon = None

        # odom / local pose
        self.x = 0.0
        self.y = 0.0
        self.odom_yaw = 0.0

        # GPS / IMU
        self.current_lat = None
        self.current_lon = None
        self.imu_yaw = None
        self.last_gps_t = None

        # yaw offset calibration for GPS navigation
        self.yaw_offset = None
        self.anchor_fix = None
        self.calib_start_t = None

        # visual gate state
        self.id1_map = None
        self.id2_map = None
        self.id1_last_seen = None
        self.id2_last_seen = None
        self.initial_side = None
        self.gate_ready_count = 0
        self.gate_ready_required = 3
        self.cooldown_until = None

        # params for vision control
        self.side_eps = 0.01
        self.marker_timeout_sec = 0.5
        self.vision_slow_dist = 0.30

        # params for GPS control
        self.kp = 1.5
        self.max_w = 1.0
        self.v_forward = 0.25
        self.goal_radius = 0.8
        self.turn_only_thresh = 0.25

        # calibration behavior
        self.calib_v = 0.20
        self.calib_timeout = 8.0
        self.calib_dist = 0.5

        # safety
        self.gps_timeout_sec = 1.0

        self._dbg = 0

        # initialize first target
        self.set_current_gate_target()

        self.mode = "GPS"   # "GPS", "SEARCH", "VISION"
        self.search_start_time = None
        self.search_timeout = 8.0
        self.search_w = 0.4
        self.search_enter_radius = 2.0   # この距離に入ったら探索開始
        self.search_success_radius = 2.5 # この距離内ならVision移行を許可

    # =========================================================
    # utility
    # =========================================================
    def stop(self):
        self.cmd_pub.publish(Twist())

    def set_current_gate_target(self):
        if self.current_gate < len(self.gates):
            g = self.gates[self.current_gate]
            self.target_lat = g["lat"]
            self.target_lon = g["lon"]
            self.get_logger().info(
                f"Target gate[{self.current_gate}] pair={g['pair_id']} "
                f"mid: lat={self.target_lat:.8f}, lon={self.target_lon:.8f}"
            )

    def advance_gate(self):
        self.current_gate += 1

        self.id1_map = None
        self.id2_map = None
        self.id1_last_seen = None
        self.id2_last_seen = None
        self.initial_side = None
        self.gate_ready_count = 0

        now_sec = self.get_clock().now().nanoseconds / 1e9
        self.cooldown_until = now_sec + 2.0

        if self.current_gate >= len(self.gates):
            self.stop()
            self.get_logger().info("全てのゲートを通過しました！")
            return

        self.mode = "GPS"
        self.search_start_time = None
        self.set_current_gate_target()

    def get_current_gate_ids(self):
        g = self.gates[self.current_gate]
        return g["a"], g["b"]

    def get_pose(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q_x = msg.pose.pose.orientation.x
        q_y = msg.pose.pose.orientation.y
        q_z = msg.pose.pose.orientation.z
        q_w = msg.pose.pose.orientation.w
        (_, _, yaw) = tf_transformations.euler_from_quaternion(
            (q_x, q_y, q_z, q_w)
        )
        return x, y, yaw

    def compute_side(self, ax, ay, bx, by, rx, ry):
        return (bx - ax) * (ry - ay) - (by - ay) * (rx - ax)

    def sign_with_deadband(self, v, eps):
        if v > eps:
            return 1
        elif v < -eps:
            return -1
        return 0

    def markers_visible_for_current_gate(self):
        now_sec = self.get_clock().now().nanoseconds / 1e9

        if self.id1_last_seen is None or self.id2_last_seen is None:
            return False

        if (now_sec - self.id1_last_seen) > self.marker_timeout_sec:
            self.id1_map = None
            self.id1_last_seen = None

        if (now_sec - self.id2_last_seen) > self.marker_timeout_sec:
            self.id2_map = None
            self.id2_last_seen = None

        return self.id1_map is not None and self.id2_map is not None

    # =========================================================
    # callbacks
    # =========================================================
    def odom_callback(self, msg):
        self.x, self.y, self.odom_yaw = self.get_pose(msg)

    def imu_callback(self, msg: Imu):
        self.imu_yaw = yaw_from_quat(msg.orientation)

    def gps_callback(self, msg: NavSatFix):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.last_gps_t = self.get_clock().now().nanoseconds * 1e-9

        if self.current_gate >= len(self.gates):
            return

        # yaw_offset calibration
        if self.yaw_offset is None and self.imu_yaw is not None:
            if self.anchor_fix is None:
                self.anchor_fix = (self.current_lat, self.current_lon)
                return

            lat0, lon0 = self.anchor_fix
            lat0_rad = math.radians(lat0)
            dn = (self.current_lat - lat0) * 111320.0
            de = (self.current_lon - lon0) * 111320.0 * math.cos(lat0_rad)
            dist = math.hypot(dn, de)

            if dist < self.calib_dist:
                return

            course = math.atan2(dn, de)  # ENU
            self.yaw_offset = wrap_pi(course - self.imu_yaw)

            self.get_logger().info(
                f"Yaw offset calibrated: off={self.yaw_offset:.3f} rad "
                f"(dist={dist:.2f}m, dn={dn:.2f}, de={de:.2f}, "
                f"course={course:.2f}, yaw={self.imu_yaw:.2f})"
            )

    def marker_callback(self, msg: ArucoMarkers):
        if self.current_gate >= len(self.gates):
            return

        id1, id2 = self.get_current_gate_ids()
        now_sec = self.get_clock().now().nanoseconds / 1e9

        # current gate marker 1
        if id1 in msg.marker_ids:
            i1 = msg.marker_ids.index(id1)
            p1 = msg.poses[i1].position

            cam_x = p1.x
            cam_z = p1.z

            map_x = self.x + cam_z * math.cos(self.odom_yaw) + cam_x * math.sin(self.odom_yaw)
            map_y = self.y + cam_z * math.sin(self.odom_yaw) - cam_x * math.cos(self.odom_yaw)

            self.id1_map = (map_x, map_y)
            self.id1_last_seen = now_sec

        # current gate marker 2
        if id2 in msg.marker_ids:
            i2 = msg.marker_ids.index(id2)
            p2 = msg.poses[i2].position

            cam_x = p2.x
            cam_z = p2.z

            map_x = self.x + cam_z * math.cos(self.odom_yaw) + cam_x * math.sin(self.odom_yaw)
            map_y = self.y + cam_z * math.sin(self.odom_yaw) - cam_x * math.cos(self.odom_yaw)

            self.id2_map = (map_x, map_y)
            self.id2_last_seen = now_sec

    # =========================================================
    # control modes
    # =========================================================
    def control_by_vision(self):
        twist = Twist()

        x1, y1 = self.id1_map
        x2, y2 = self.id2_map

        center_map_x = (x1 + x2) / 2.0
        center_map_y = (y1 + y2) / 2.0

        dx = center_map_x - self.x
        dy = center_map_y - self.y

        dist = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)

        yaw_error = wrap_pi(target_yaw - self.odom_yaw)

        current_side = self.compute_side(x1, y1, x2, y2, self.x, self.y)
        gate_width = math.hypot(x2 - x1, y2 - y1)

        if 5.0 <= gate_width <= 7.5 and abs(current_side) > self.side_eps:
            self.gate_ready_count += 1
        else:
            self.gate_ready_count = 0

        if self.initial_side is None:
            if self.gate_ready_count >= self.gate_ready_required:
                self.initial_side = current_side
                self.get_logger().info(f"[VISION] initial_side={self.initial_side:.4f}")
            else:
                twist.linear.x = 0.05
                twist.angular.z = max(min(0.6 * yaw_error, 0.5), -0.5)
                self.cmd_pub.publish(twist)
                self.get_logger().info("[VISION] waiting gate stable")
                return

        initial_sign = self.sign_with_deadband(self.initial_side, self.side_eps)
        current_sign = self.sign_with_deadband(current_side, self.side_eps)

        crossed = False
        if initial_sign != 0 and current_sign != 0 and initial_sign != current_sign:
            crossed = True

        if abs(current_side) < 0.01 and dist < 0.2:
            crossed = True

        if crossed:
            self.get_logger().info(f"[VISION] gate[{self.current_gate}] passed")
            self.cmd_pub.publish(Twist())
            self.advance_gate()
            return

        if dist < self.vision_slow_dist:
            twist.linear.x = min(0.15, dist * 0.5)
            twist.angular.z = max(min(0.5 * yaw_error, 0.2), -0.2)
        else:
            twist.linear.x = min(0.25, dist * 0.4)
            twist.angular.z = max(min(yaw_error, 0.4), -0.4)

        self.cmd_pub.publish(twist)

        self.get_logger().info(
            f"[VISION] gate[{self.current_gate}] dist={dist:.2f}, "
            f"yaw_error={math.degrees(yaw_error):.2f}deg, "
            f"side={current_side:.4f}, gate_width={gate_width:.2f}"
        )

    def control_by_gps(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        # GPS timeout
        if self.last_gps_t is None or (now - self.last_gps_t) > self.gps_timeout_sec:
            self.stop()
            self.get_logger().warn("[GPS] timeout -> STOP")
            return

        if (self.current_lat is None or self.current_lon is None or
                self.target_lat is None or self.target_lon is None or
                self.imu_yaw is None):
            return

        # calibration mode
        if self.yaw_offset is None:
            if self.calib_start_t is None:
                self.calib_start_t = now
                self.anchor_fix = (self.current_lat, self.current_lon)
                self.get_logger().info(
                    f"[GPS] Calibrating yaw_offset: driving forward... (need {self.calib_dist:.2f}m)"
                )

            cmd = Twist()
            cmd.linear.x = self.calib_v
            cmd.angular.z = 0.0
            self.cmd_pub.publish(cmd)

            if (now - self.calib_start_t) > self.calib_timeout:
                self.stop()
                self.get_logger().warn("[GPS] calibration timeout: resetting anchor...")
                self.calib_start_t = None
                self.anchor_fix = None
            return

        lat0_rad = math.radians(self.current_lat)
        dn = (self.target_lat - self.current_lat) * 111320.0
        de = (self.target_lon - self.current_lon) * 111320.0 * math.cos(lat0_rad)
        dist = math.hypot(dn, de)

        # GPSでは「中点近傍まで来た」だけでは次ゲートにしない。
        # ArUcoが見えたら vision に移行して通過判定する。
        # ただし本当にArUcoが見えないケースのための保険として very close なら進める
        if dist < self.goal_radius * 0.5 and not self.markers_visible_for_current_gate():
            self.get_logger().warn(
                f"[GPS] very close to gate[{self.current_gate}] midpoint but markers not visible. "
                f"Fallback advance."
            )
            self.advance_gate()
            return

        desired = math.atan2(dn, de)  # ENU
        yaw_corr = wrap_pi(self.imu_yaw + self.yaw_offset)
        err = wrap_pi(desired - yaw_corr)

        w = max(-self.max_w, min(self.max_w, self.kp * err))

        cmd = Twist()
        cmd.angular.z = w
        cmd.linear.x = 0.0 if abs(err) > self.turn_only_thresh else self.v_forward
        self.cmd_pub.publish(cmd)

        self._dbg += 1
        if self._dbg % 10 == 0:
            g = self.gates[self.current_gate]
            self.get_logger().info(
                f"[GPS] gate[{self.current_gate}] pair={g['pair_id']} "
                f"dist={dist:.2f} dn={dn:.2f} de={de:.2f} "
                f"yaw={self.imu_yaw:.2f} off={self.yaw_offset:.2f} yawC={yaw_corr:.2f} "
                f"des={desired:.2f} err={err:.2f} w={w:.2f}"
            )

    # =========================================================
    # main control loop
    # =========================================================
    def control_loop(self):
        if self.current_gate >= len(self.gates):
            self.stop()
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9

        if self.cooldown_until is not None and now_sec < self.cooldown_until:
            self.stop()
            return

        visible = self.markers_visible_for_current_gate()

        # マーカが見えたらいつでもVISIONへ
        if visible:
            self.mode = "VISION"

        if self.mode == "VISION":
            self.control_by_vision()
            return

        # GPS距離を計算（SEARCH遷移判断用）
        gps_dist = None
        if self.current_lat is not None and self.current_lon is not None \
        and self.target_lat is not None and self.target_lon is not None:
            lat0_rad = math.radians(self.current_lat)
            dn = (self.target_lat - self.current_lat) * 111320.0
            de = (self.target_lon - self.current_lon) * 111320.0 * math.cos(lat0_rad)
            gps_dist = math.hypot(dn, de)

        if self.mode == "GPS":
            # ゲート近傍に来たら探索モードへ
            if gps_dist is not None and gps_dist < self.search_enter_radius:
                self.mode = "SEARCH"
                self.search_start_time = now_sec
                self.stop()
                self.get_logger().info(
                    f"[SEARCH] enter search mode for gate[{self.current_gate}] dist={gps_dist:.2f}"
                )
                return

            self.gate_ready_count = 0
            self.initial_side = None
            self.control_by_gps()
            return

        if self.mode == "SEARCH":
            # 見つかったら冒頭でVISIONに移るのでここでは探索だけ
            if self.search_start_time is None:
                self.search_start_time = now_sec

            elapsed = now_sec - self.search_start_time

            # 一定時間見つからなければ次へ進む or GPSに戻る
            if elapsed > self.search_timeout:
                self.get_logger().warn(
                    f"[SEARCH] timeout at gate[{self.current_gate}] -> fallback advance"
                )
                self.advance_gate()
                return

            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = self.search_w
            self.cmd_pub.publish(twist)

            self.get_logger().info(
                f"[SEARCH] rotating... elapsed={elapsed:.1f}s"
            )
            return


def main(args=None):
    rclpy.init(args=args)
    node = UnifiedGateNavigator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()