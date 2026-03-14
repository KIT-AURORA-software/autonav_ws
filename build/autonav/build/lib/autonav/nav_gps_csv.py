import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import Twist
import csv

def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quat(q) -> float:
    # geometry_msgs/Quaternion -> yaw
    x, y, z, w = q.x, q.y, q.z, q.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)

def load_targets(csv_path: str):
    targets = []
    with open(csv_path, newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            aruco_id = int(row["aruco_id"].strip())
            pair_id  = int(row["pair_id"].strip())
            lat      = float(row["latitude_north"].strip())
            lon      = float(row["longitude_east"].strip())
            targets.append({"aruco_id": aruco_id, "pair_id": pair_id, "lat": lat, "lon": lon, "visited": False})
    return targets


class NavGPS(Node):
    """
    /fix (NavSatFix) の局所近似(ENU)で目標方向を計算し、
    /imu の yaw から heading を作って cmd_vel を出す。

    重要：IMU yaw と GPS ENU の向きが合わないので、
          起動直後に「少し直進→GPSの進行方位」から yaw_offset を自動校正する。
          10Hz GPSだと連続差分は小さすぎるので、アンカー方式で距離が溜まってから校正する。
    """

    def __init__(self):
        super().__init__("nav_gps")

        self.declare_parameter("csv_path", "")
        csv_path = self.get_parameter("csv_path").get_parameter_value().string_value

        self.targets = load_targets(csv_path)
        self.get_logger().info(f"Loaded {len(self.targets)} targets from CSV.")

        # subs/pubs
        self.sub_gps = self.create_subscription(NavSatFix, "/fix", self.gps_callback, 10)
        self.sub_imu = self.create_subscription(Imu, "/imu", self.imu_callback, 50)
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # state
        self.current_latitude = None
        self.current_longitude = None
        self.yaw = None  # imu yaw
        self.yaw_offset = None  # GPS(ENU)に合わせる補正

        # target
        self.target_lat = None
        self.target_lon = None

        # calibration anchor
        self.anchor_fix = None  # (lat, lon) アンカー（更新しない）
        self.anchor_t = None
        self.calib_start_t = None

        # params
        self.kp = 1.5
        self.max_w = 1.0
        self.v_forward = 0.2

        self.goal_radius = 0.5         # [m] 到達判定
        self.turn_only_thresh = 0.25   # [rad] これより誤差が大きければ回転のみ

        # calibration behavior
        self.calib_v = 0.25            # [m/s] 校正中の直進速度
        self.calib_timeout = 8.0       # [s]   校正が決まらないときの停止タイムアウト
        self.calib_dist = 0.5          # [m]   アンカーからこの距離進んだら course を計算

        # debug
        self._dbg_count = 0

        # control loop
        self.timer = self.create_timer(0.05, self.control_loop)  # 20Hz

    # --- callbacks ---

    def imu_callback(self, msg: Imu):
        # Gazebo IMUは orientation が入るので、それからyawを直接取得
        self.yaw = yaw_from_quat(msg.orientation)

    # def read_goal_from_csv(self, aruco_id: int):
    #     """
    #     fix.csv から指定 aruco_id の (lat, lon) を返す。
    #     見つからなければ None を返す。
    #     """
    #     with open("/home/fuga1129/autonav_ws/src/autonav/targets.csv", "r", encoding="ms932", newline="") as f:
    #         reader = csv.DictReader(f, skipinitialspace=True)

    #         for row in reader:
    #             try:
    #                 rid = int(str(row["aruco_id"]).strip().replace('"', ''))
    #                 if rid != aruco_id:
    #                     continue

    #                 lat_s = str(row["latitude_north"]).strip().replace('"', '')
    #                 lon_s = str(row["longitude_east"]).strip().replace('"', '')
    #                 lat = float(lat_s)
    #                 lon = float(lon_s)
    #                 return lat, lon

    #             except (KeyError, ValueError) as e:
    #                 # 壊れた行はスキップ
    #                 self.get_logger().warn(f"Skip bad row: {row} ({e})")

    #     return None

    def gps_callback(self, msg: NavSatFix):
        self.current_latitude = msg.latitude
        self.current_longitude = msg.longitude
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # ターゲット設定：とりあえず北へ約1m（緯度増加）
        # ターゲット設定：CSVから読む
        if self.target_lat is None:
            # goal = self.read_goal_from_csv(aruco_id=0)  # ←目的のIDに変更

            # if goal is None:
            #     self.get_logger().error("Goal not found in fix.csv.")
            #     return
            # else:
            #     self.target_lat, self.target_lon = goal

            if len(self.targets) == 0:
                self.get_logger().error("No targets in CSV")
                return
            
            first = self.targets[0]
            self.target_lat = first["lat"]
            self.target_lon = first["lon"]
            self.get_logger().info(
                f"Target set from CSV: lat={self.target_lat:.10f}, "
                f"lon={self.target_lon:.10f}"
            )

        # yaw_offset未確定なら、アンカー方式で course からオフセット決定
        if self.yaw_offset is None and self.yaw is not None:
            # アンカー未設定ならここで固定
            if self.anchor_fix is None:
                self.anchor_fix = (self.current_latitude, self.current_longitude)
                self.anchor_t = t
                return

            lat0, lon0 = self.anchor_fix

            # 変位(m)  (近距離近似)
            lat0_rad = math.radians(lat0)
            dn = (self.current_latitude - lat0) * 111320.0
            de = (self.current_longitude - lon0) * 111320.0 * math.cos(lat0_rad)
            dist = math.hypot(dn, de)

            # まだ距離が溜まってないなら待つ（アンカーは更新しない）
            if dist < self.calib_dist:
                return

            # ENUの進行方位（East=x, North=y）
            # yaw=0 を +x(East) に合わせる扱いで統一：atan2(N, E)
            course = math.atan2(dn, de)

            # IMU yaw に対して、GPS方位へ合わせるオフセット
            self.yaw_offset = wrap_pi(course - self.yaw)

            self.get_logger().info(
                f"Yaw offset calibrated: off={self.yaw_offset:.3f} rad "
                f"(dist={dist:.2f}m, dn={dn:.2f}, de={de:.2f}, course={course:.2f}, yaw={self.yaw:.2f})"
            )

    # --- helpers ---

    def stop(self):
        self.pub.publish(Twist())

    # --- main loop ---

    def control_loop(self):
        if (self.current_latitude is None or self.current_longitude is None or
                self.target_lat is None or self.target_lon is None or
                self.yaw is None):
            return

        # --- calibration mode ---
        if self.yaw_offset is None:
            now = self.get_clock().now().nanoseconds * 1e-9

            if self.calib_start_t is None:
                self.calib_start_t = now
                # ここでアンカーを取り直す（止まってる間にズレてもOK）
                self.anchor_fix = (self.current_latitude, self.current_longitude)
                self.anchor_t = now
                self.get_logger().info(
                    f"Calibrating yaw_offset: driving forward... (need {self.calib_dist:.2f}m)"
                )

            # 校正のため直進（nav_gpsが /cmd_vel を上書きするので、手動操作するならこのノードを止める）
            cmd = Twist()
            cmd.linear.x = self.calib_v
            cmd.angular.z = 0.0
            self.pub.publish(cmd)

            # タイムアウト → 一旦止めてアンカー取り直し
            if (now - self.calib_start_t) > self.calib_timeout:
                self.stop()
                self.get_logger().warn(
                    "Calibration timeout: yaw_offset not set yet. Resetting anchor..."
                )
                self.calib_start_t = None
                self.anchor_fix = None
                self.anchor_t = None
            return

        # --- navigation mode ---

        # lat/lon -> meters (local approx, ENU)
        lat0_rad = math.radians(self.current_latitude)
        dn = (self.target_lat - self.current_latitude) * 111320.0
        de = (self.target_lon - self.current_longitude) * 111320.0 * math.cos(lat0_rad)
        dist = math.hypot(dn, de)

        if dist < self.goal_radius:
            self.stop()
            self.get_logger().info("Reached target. Stopping.")
            return

        # desired heading in ENU (East=x, North=y): atan2(N, E)
        desired_heading = math.atan2(dn, de)

        # yaw corrected into ENU heading
        yaw_corr = wrap_pi(self.yaw + self.yaw_offset)

        # heading error
        error = wrap_pi(desired_heading - yaw_corr)

        # angular control
        w = self.kp * error
        w = max(-self.max_w, min(self.max_w, w))

        cmd = Twist()
        if abs(error) > self.turn_only_thresh:
            cmd.linear.x = 0.0
        else:
            cmd.linear.x = self.v_forward
        cmd.angular.z = w
        self.pub.publish(cmd)

        self._dbg_count += 1
        if self._dbg_count % 10 == 0:
            self.get_logger().info(
                f"dist={dist:.2f}m dn={dn:.2f} de={de:.2f} "
                f"yaw={self.yaw:.2f} off={self.yaw_offset:.2f} yawC={yaw_corr:.2f} "
                f"des={desired_heading:.2f} err={error:.2f} w={w:.2f}"
            )


def main(args=None):
    rclpy.init(args=args)
    node = NavGPS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()