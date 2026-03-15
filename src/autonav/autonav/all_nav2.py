#!/usr/bin/env python3

import math
import csv
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
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
    pair_id は相方の aruco_id とみなす
    A->B と B->A の相互参照からゲート中点を作る
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
            "gate": key,
            "a": key[0],
            "b": key[1],
            "lat": lat_mid,
            "lon": lon_mid,
        })
        seen.add(key)

    gates.sort(key=lambda g: g["gate"])
    return gates


class IntegratedGateNavigator(Node):
    def __init__(self):
        super().__init__("integrated_gate_navigator")

        # =====================================================
        # parameter
        # =====================================================
        self.declare_parameter("csv_path", "")
        csv_path = self.get_parameter("csv_path").get_parameter_value().string_value

        if not csv_path:
            self.get_logger().error("csv_path parameter is empty.")
            raise RuntimeError("csv_path is required")

        markers = load_markers(csv_path)
        self.gates = build_gate_midpoints_from_partner_id(markers)

        self.get_logger().info(f"CSV loaded: {csv_path}")
        self.get_logger().info(f"Loaded {len(markers)} markers.")
        self.get_logger().info(f"Built {len(self.gates)} gate midpoints.")

        if len(self.gates) == 0:
            self.get_logger().error("No valid gates.")
            raise RuntimeError("No valid gates")

        # =====================================================
        # pub/sub
        # =====================================================
        self.sub_gps = self.create_subscription(
            NavSatFix, "/fix", self.gps_callback, 10
        )
        self.sub_imu = self.create_subscription(
            Imu, "/imu", self.imu_callback, 50
        )
        self.sub_odom = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )
        self.sub_aruco = self.create_subscription(
            ArucoMarkers, "/aruco_markers", self.aruco_callback, 10
        )

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.mode_pub = self.create_publisher(Int32, "/mode", 10)

        self.timer = self.create_timer(0.05, self.control_loop)

        # =====================================================
        # upper navigation state
        # =====================================================
        self.STATE_CALIB = "CALIB"
        self.STATE_GPS_NAV = "GPS_NAV"
        self.STATE_VISION_NAV = "VISION_NAV"
        self.STATE_DONE = "DONE"

        self.state = self.STATE_CALIB

        # =====================================================
        # lower drive mode
        # mode=1: opposite_phase
        # mode=2: pivot
        # =====================================================
        self.MODE_OPPOSITE_PHASE = 1
        self.MODE_PIVOT = 2
        self.drive_mode = self.MODE_OPPOSITE_PHASE

        # =====================================================
        # current target
        # =====================================================
        self.current_gate = 0
        self.target_lat = self.gates[0]["lat"]
        self.target_lon = self.gates[0]["lon"]

        # =====================================================
        # sensor state
        # =====================================================
        self.current_lat = None
        self.current_lon = None
        self.last_gps_t = None

        self.yaw_imu = None
        self.yaw_offset = None

        self.x = 0.0
        self.y = 0.0
        self.yaw_odom = 0.0

        # calib
        self.anchor_fix = None
        self.calib_start_t = None

        # aruco map positions
        self.id1_map = None
        self.id2_map = None

        # gate crossing
        self.initial_side = None
        self.gate_ready_count = 0
        self.gate_ready_required = 3
        self.cooldown_until = None

        # =====================================================
        # parameters
        # =====================================================
        # GPS navigation
        self.kp_gps = 1.5
        self.max_w_gps = 0.8
        self.v_forward_gps = 0.25
        self.goal_radius = 0.8
        self.turn_only_thresh = 0.25
        self.vision_switch_dist = 3.0

        # yaw calibration
        self.calib_v = 0.20
        self.calib_timeout = 8.0
        self.calib_dist = 0.5

        # safety
        self.gps_timeout_sec = 1.0

        # mode switching hysteresis
        self.pivot_enter_thresh = math.radians(25.0)
        self.pivot_exit_thresh = math.radians(12.0)

        # vision control
        self.vision_search_w = 0.2
        self.vision_slow_dist = 0.30

        # gate width check
        self.gate_width_min = 5.0
        self.gate_width_max = 7.5

        # debug
        self._dbg = 0

        # visiontime
        self.vision_startt_time = None
        self.vision_timeout_sec = 20.0

    # =====================================================
    # utility
    # =====================================================
    def publish_cmd_mode(self, linear_x: float, angular_z: float, mode_value: int):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z

        mode_msg = Int32()
        mode_msg.data = mode_value

        self.cmd_pub.publish(twist)
        self.mode_pub.publish(mode_msg)

    def stop(self):
        self.publish_cmd_mode(0.0, 0.0, self.MODE_OPPOSITE_PHASE)

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

    def get_current_gate_info(self):
        if self.current_gate >= len(self.gates):
            return None
        return self.gates[self.current_gate]

    def set_next_gate_target(self):
        gate = self.get_current_gate_info()
        if gate is None:
            return
        self.target_lat = gate["lat"]
        self.target_lon = gate["lon"]

    def reset_vision_state(self):
        self.id1_map = None
        self.id2_map = None
        self.initial_side = None
        self.gate_ready_count = 0
        self.vision_start_time = None

    def advance_gate(self):
        self.current_gate += 1
        self.reset_vision_state()
        self.cooldown_until = None

        if self.current_gate >= len(self.gates):
            self.state = self.STATE_DONE
            self.stop()
            self.get_logger().info("All gates reached. Stopping.")
            return

        self.set_next_gate_target()
        gate = self.get_current_gate_info()
        self.state = self.STATE_GPS_NAV
        self.get_logger().info(
            f"Next gate[{self.current_gate}] pair_id={gate['pair_id']} "
            f"mid: lat={self.target_lat:.8f}, lon={self.target_lon:.8f}"
        )

    def compute_side(self, ax, ay, bx, by, rx, ry):
        return (bx - ax) * (ry - ay) - (by - ay) * (rx - ax)

    def sign_with_deadband(self, v, eps):
        if v > eps:
            return 1
        elif v < -eps:
            return -1
        return 0

    def calc_relative_gps_target(self):
        lat0_rad = math.radians(self.current_lat)
        dn = (self.target_lat - self.current_lat) * 111320.0
        de = (self.target_lon - self.current_lon) * 111320.0 * math.cos(lat0_rad)
        dist = math.hypot(dn, de)
        return dn, de, dist

    def select_drive_mode_from_yaw_error(self, yaw_error):
        abs_err = abs(yaw_error)

        if self.drive_mode != self.MODE_PIVOT and abs_err > self.pivot_enter_thresh:
            self.drive_mode = self.MODE_PIVOT
        elif self.drive_mode == self.MODE_PIVOT and abs_err < self.pivot_exit_thresh:
            self.drive_mode = self.MODE_OPPOSITE_PHASE

    # =====================================================
    # callbacks
    # =====================================================
    def imu_callback(self, msg: Imu):
        self.yaw_imu = yaw_from_quat(msg.orientation)

    def odom_callback(self, msg: Odometry):
        self.x, self.y, self.yaw_odom = self.get_pose(msg)

    def gps_callback(self, msg: NavSatFix):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.last_gps_t = self.get_clock().now().nanoseconds * 1e-9

        # yaw_offset calibration
        if self.yaw_offset is None and self.yaw_imu is not None:
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

            course = math.atan2(dn, de)
            self.yaw_offset = wrap_pi(course - self.yaw_imu)
            self.state = self.STATE_GPS_NAV

            gate = self.get_current_gate_info()
            if gate is not None:
                self.get_logger().info(
                    f"Yaw offset calibrated: off={self.yaw_offset:.3f} rad, "
                    f"start gate[{self.current_gate}] pair_id={gate['pair_id']}"
                )

    def aruco_callback(self, msg: ArucoMarkers):
        gate = self.get_current_gate_info()
        if gate is None:
            return

        id1 = gate["a"]
        id2 = gate["b"]

        if id1 in msg.marker_ids:
            i1 = msg.marker_ids.index(id1)
            p1 = msg.poses[i1].position

            cam_x = p1.x
            cam_z = p1.z

            map_x = self.x + cam_z * math.cos(self.yaw_odom) + cam_x * math.sin(self.yaw_odom)
            map_y = self.y + cam_z * math.sin(self.yaw_odom) - cam_x * math.cos(self.yaw_odom)

            self.id1_map = (map_x, map_y)

        if id2 in msg.marker_ids:
            i2 = msg.marker_ids.index(id2)
            p2 = msg.poses[i2].position

            cam_x = p2.x
            cam_z = p2.z

            map_x = self.x + cam_z * math.cos(self.yaw_odom) + cam_x * math.sin(self.yaw_odom)
            map_y = self.y + cam_z * math.sin(self.yaw_odom) - cam_x * math.cos(self.yaw_odom)

            self.id2_map = (map_x, map_y)

    # =====================================================
    # state controls
    # =====================================================
    def control_calib(self, now):
        if self.current_lat is None or self.current_lon is None or self.yaw_imu is None:
            return

        if self.calib_start_t is None:
            self.calib_start_t = now
            self.anchor_fix = (self.current_lat, self.current_lon)
            self.get_logger().info(
                f"Calibrating yaw_offset: driving forward... need {self.calib_dist:.2f}m"
            )

        # calibration中は前進なので opposite_phase
        self.drive_mode = self.MODE_OPPOSITE_PHASE
        self.publish_cmd_mode(self.calib_v, 0.0, self.drive_mode)

        if (now - self.calib_start_t) > self.calib_timeout:
            self.stop()
            self.get_logger().warn("Calibration timeout: resetting anchor...")
            self.calib_start_t = None
            self.anchor_fix = None

    def control_gps_nav(self, now):
        if self.last_gps_t is None or (now - self.last_gps_t) > self.gps_timeout_sec:
            self.stop()
            self.get_logger().warn("GPS timeout -> STOP")
            return

        if (
            self.current_lat is None or
            self.current_lon is None or
            self.target_lat is None or
            self.target_lon is None or
            self.yaw_imu is None or
            self.yaw_offset is None
        ):
            return

        dn, de, dist = self.calc_relative_gps_target()
        desired = math.atan2(dn, de)
        yaw_corr = wrap_pi(self.yaw_imu + self.yaw_offset)
        yaw_error = wrap_pi(desired - yaw_corr)

        gate = self.get_current_gate_info()
        if gate is None:    
            self.state = self.STATE_DONE
            self.stop()
            return

        # 近づいたら vision へ
        if dist < self.vision_switch_dist:
            self.state = self.STATE_VISION_NAV
            self.reset_vision_state()
            self.vision_start_time = now
            self.drive_mode = self.MODE_OPPOSITE_PHASE
            self.get_logger().info(
                f"Switch to VISION_NAV for gate[{self.current_gate}] pair={gate['pair_id']} "
                f"(gps_dist={dist:.2f}m)"
            )
            return

        self.select_drive_mode_from_yaw_error(yaw_error)

        if self.drive_mode == self.MODE_PIVOT:
            linear_x = 0.0
            angular_z = max(min(1.0 * yaw_error, self.max_w_gps), -self.max_w_gps)

        else:
            angular_z = max(min(self.kp_gps * yaw_error, self.max_w_gps), -self.max_w_gps)
            linear_x = 0.0 if abs(yaw_error) > self.turn_only_thresh else self.v_forward_gps

        self.publish_cmd_mode(linear_x, angular_z, self.drive_mode)

        self._dbg += 1
        if self._dbg % 10 == 0:
            self.get_logger().info(
                f"[GPS] gate[{self.current_gate}] pair={gate['pair_id']} "
                f"dist={dist:.2f} dn={dn:.2f} de={de:.2f} "
                f"yaw={self.yaw_imu:.2f} off={self.yaw_offset:.2f} yawC={yaw_corr:.2f} "
                f"des={desired:.2f} err={yaw_error:.2f} mode={self.drive_mode}"
            )

        # 保険としてGPSだけで到達扱い
        if dist < self.goal_radius:
            self.get_logger().info(
                f"Gate[{self.current_gate}] reached by GPS radius only. Advance gate."
            )
            self.advance_gate()

    def control_vision_nav(self, now):
        
        if self.cooldown_until is not None and now < self.cooldown_until:
            self.publish_cmd_mode(0.0, 0.0, self.MODE_OPPOSITE_PHASE)
            return
        
        if self.vision_start_time is None:
            self.vision_start_time = now

        if self.id1_map is None or self.id2_map is None:
            # マーカが片方/両方見えないときは探索
            elapsed = now - self.vision_start_time

            if elapsed > self.vision_timeout_sec:
                self.get_logger().warn("見つかりませんでした。次に進みます")
                self.advance_gate()
                return
            

            self.drive_mode = self.MODE_PIVOT
            self.publish_cmd_mode(0.0, self.vision_search_w, self.drive_mode)
            self.get_logger().info("[VISION] searching markers...")
            return

        x1, y1 = self.id1_map
        x2, y2 = self.id2_map

        center_map_x = (x1 + x2) / 2.0
        center_map_y = (y1 + y2) / 2.0

        dx = center_map_x - self.x
        dy = center_map_y - self.y

        dist = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)
        yaw_error = wrap_pi(target_yaw - self.yaw_odom)

        current_side = self.compute_side(x1, y1, x2, y2, self.x, self.y)

        side_eps = 0.01
        gate_width = math.hypot(x2 - x1, y2 - y1)

        if self.gate_width_min <= gate_width <= self.gate_width_max and abs(current_side) > side_eps:
            self.gate_ready_count += 1
        else:
            self.gate_ready_count = 0

        if self.initial_side is None:
            if self.gate_ready_count >= self.gate_ready_required:
                self.initial_side = current_side
                self.get_logger().info(
                    f"[VISION] gate stable. initial_side={self.initial_side:.4f}"
                )
            else:
                # 安定するまでは少しずつ寄せる
                self.select_drive_mode_from_yaw_error(yaw_error)

                if self.drive_mode == self.MODE_PIVOT:
                    linear_x = 0.0
                    angular_z = max(min(0.8 * yaw_error, 0.5), -0.5)
                else:
                    linear_x = 0.05
                    angular_z = max(min(0.6 * yaw_error, 0.4), -0.4)

                self.publish_cmd_mode(linear_x, angular_z, self.drive_mode)
                self.get_logger().info("[VISION] waiting gate stable")
                return

        initial_sign = self.sign_with_deadband(self.initial_side, side_eps)
        current_sign = self.sign_with_deadband(current_side, side_eps)

        crossed = False
        if initial_sign != 0 and current_sign != 0 and initial_sign != current_sign:
            crossed = True

        if abs(current_side) < 0.01 and dist < 0.2:
            crossed = True

        if crossed:
            self.get_logger().info(
                f"[VISION] Gate passed! gate_index={self.current_gate}"
            )
            self.publish_cmd_mode(0.0, 0.0, self.MODE_OPPOSITE_PHASE)
            self.cooldown_until = now + 1.5
            self.advance_gate()
            return

        # 通過前の追従
        self.select_drive_mode_from_yaw_error(yaw_error)

        if self.drive_mode == self.MODE_PIVOT:
            linear_x = 0.0
            angular_z = max(min(1.0 * yaw_error, 0.5), -0.5)

        else:
            if dist < self.vision_slow_dist:
                linear_x = min(0.15, dist * 0.5)
                angular_z = max(min(0.5 * yaw_error, 0.2), -0.2)
            else:
                linear_x = min(0.25, dist * 0.4)
                angular_z = max(min(0.8 * yaw_error, 0.4), -0.4)

        self.publish_cmd_mode(linear_x, angular_z, self.drive_mode)

        self.get_logger().info(
            f"[VISION] gate[{self.current_gate}] dist={dist:.2f}, "
            f"yaw_error={math.degrees(yaw_error):.2f} deg, "
            f"side={current_side:.4f}, gate_width={gate_width:.2f}, "
            f"mode={self.drive_mode}"
        )

    # =====================================================
    # main control
    # =====================================================
    def control_loop(self):
        now = self.get_clock().now().nanoseconds * 1e-9

        if self.state == self.STATE_DONE:
            self.stop()
            return

        if self.state == self.STATE_CALIB:
            self.control_calib(now)

        elif self.state == self.STATE_GPS_NAV:
            self.control_gps_nav(now)

        elif self.state == self.STATE_VISION_NAV:
            self.control_vision_nav(now)


def main(args=None):
    rclpy.init(args=args)
    node = IntegratedGateNavigator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()