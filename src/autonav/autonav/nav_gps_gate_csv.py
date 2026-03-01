import math
import csv
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Twist


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
        reader = csv.DictReader(f, skipinitialspace=True)  # ←これが重要

        # ヘッダ名を強制的にstripして正規化
        if reader.fieldnames is None:
            raise RuntimeError("CSV header not found")
        reader.fieldnames = [h.strip() for h in reader.fieldnames]

        for row in reader:
            # 行のキーもstrip（念のため）
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
    markers: [{"aruco_id": int, "pair_id": int, "lat": float, "lon": float}, ...]
    pair_id は「相方aruco_id」とみなす。
    A->B と B->A をペアにして中点を作る。
    """
    by_id = {m["aruco_id"]: m for m in markers}

    gates = []
    seen = set()  # (min_id, max_id)

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
        # 相互参照になっているかチェック（任意だが安全）
        if mb.get("pair_id") != a:
            # 片方向だけの定義でも中点を作りたいならこのチェックは外してOK
            continue

        lat_mid = 0.5 * (m["lat"] + mb["lat"])
        lon_mid = 0.5 * (m["lon"] + mb["lon"])

        gates.append({
            "pair_id": f"{key[0]}-{key[1]}",
            "gate": key,          # (a,b) のペア
            "a": a,
            "b": b,
            "lat": lat_mid,
            "lon": lon_mid,
            "visited": False,
        })
        seen.add(key)

    # 走行順を安定させたいならソート（例：小さいID順）
    gates.sort(key=lambda g: g["gate"])
    return gates

class NavGPSGateCSV(Node):
    def __init__(self):
        super().__init__("nav_gps_gate_csv")

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
            self.get_logger().error("No valid gates (need 2 markers per pair_id).")
            raise RuntimeError("No valid gates")

        # subs/pubs
        self.sub_gps = self.create_subscription(NavSatFix, "/fix", self.gps_callback, 10)
        self.sub_imu = self.create_subscription(Imu, "/imu", self.imu_callback, 50)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # state
        self.current_lat = None
        self.current_lon = None
        self.yaw = None
        self.last_gps_t = None

        # yaw offset calibration
        self.yaw_offset = None
        self.anchor_fix = None
        self.calib_start_t = None

        # navigation
        self.idx = 0
        self.target_lat = None
        self.target_lon = None

        # params
        self.kp = 1.5
        self.max_w = 1.0
        self.v_forward = 0.25

        self.goal_radius = 0.8          # ゲート中点に近づいた判定（シミュなら0.5〜1.0）
        self.turn_only_thresh = 0.25

        # calibration behavior
        self.calib_v = 0.25
        self.calib_timeout = 8.0
        self.calib_dist = 0.5

        # safety
        self.gps_timeout_sec = 1.0

        self._dbg = 0
        self.timer = self.create_timer(0.05, self.control_loop)

    def stop(self):
        self.pub.publish(Twist())

    def imu_callback(self, msg: Imu):
        self.yaw = yaw_from_quat(msg.orientation)

    def gps_callback(self, msg: NavSatFix):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.last_gps_t = self.get_clock().now().nanoseconds * 1e-9
        # t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        # self.last_gps_t = t

        # 初回：最初のゲート中点をセット
        if self.target_lat is None:
            g = self.gates[self.idx]
            self.target_lat = g["lat"]
            self.target_lon = g["lon"]
            self.get_logger().info(
                f"Target gate[{self.idx}] pair_id={g['pair_id']} mid: lat={self.target_lat:.8f}, lon={self.target_lon:.8f}"
            )

        # yaw_offset未確定なら、アンカー方式で course からオフセット決定
        if self.yaw_offset is None and self.yaw is not None:
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

            course = math.atan2(dn, de)  # ENU: atan2(N, E)
            self.yaw_offset = wrap_pi(course - self.yaw)

            self.get_logger().info(
                f"Yaw offset calibrated: off={self.yaw_offset:.3f} rad "
                f"(dist={dist:.2f}m, dn={dn:.2f}, de={de:.2f}, course={course:.2f}, yaw={self.yaw:.2f})"
            )

    def control_loop(self):
        # safety: GPS timeout
        now = self.get_clock().now().nanoseconds * 1e-9
        if self.last_gps_t is None or (now - self.last_gps_t) > self.gps_timeout_sec:
            self.stop()
            self.get_logger().warn("GPS timeout -> STOP")
            return

        if (self.current_lat is None or self.current_lon is None or
                self.target_lat is None or self.target_lon is None or
                self.yaw is None):
            return

        # calibration mode
        if self.yaw_offset is None:
            if self.calib_start_t is None:
                self.calib_start_t = now
                self.anchor_fix = (self.current_lat, self.current_lon)
                self.get_logger().info(
                    f"Calibrating yaw_offset: driving forward... (need {self.calib_dist:.2f}m)"
                )

            cmd = Twist()
            cmd.linear.x = self.calib_v
            cmd.angular.z = 0.0
            self.pub.publish(cmd)

            if (now - self.calib_start_t) > self.calib_timeout:
                self.stop()
                self.get_logger().warn("Calibration timeout: resetting anchor...")
                self.calib_start_t = None
                self.anchor_fix = None
            return

        # navigation mode
        lat0_rad = math.radians(self.current_lat)
        dn = (self.target_lat - self.current_lat) * 111320.0
        de = (self.target_lon - self.current_lon) * 111320.0 * math.cos(lat0_rad)
        dist = math.hypot(dn, de)

        if dist < self.goal_radius:
            # 次のゲートへ
            self.idx += 1
            if self.idx >= len(self.gates):
                self.stop()
                self.get_logger().info("All gates reached. Stopping.")
                return

            g = self.gates[self.idx]
            self.target_lat = g["lat"]
            self.target_lon = g["lon"]
            self.get_logger().info(
                f"Reached gate. Next gate[{self.idx}] pair_id={g['pair_id']} mid: lat={self.target_lat:.8f}, lon={self.target_lon:.8f}"
            )
            return

        desired = math.atan2(dn, de)  # ENU
        yaw_corr = wrap_pi(self.yaw + self.yaw_offset)
        err = wrap_pi(desired - yaw_corr)

        w = max(-self.max_w, min(self.max_w, self.kp * err))

        cmd = Twist()
        cmd.angular.z = w
        cmd.linear.x = 0.0 if abs(err) > self.turn_only_thresh else self.v_forward
        self.pub.publish(cmd)

        self._dbg += 1
        if self._dbg % 10 == 0:
            g = self.gates[self.idx]
            self.get_logger().info(
                f"gate[{self.idx}] pair={g['pair_id']} dist={dist:.2f} dn={dn:.2f} de={de:.2f} "
                f"yaw={self.yaw:.2f} off={self.yaw_offset:.2f} yawC={yaw_corr:.2f} "
                f"des={desired:.2f} err={err:.2f} w={w:.2f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = NavGPSGateCSV()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()