#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf_transformations import euler_from_quaternion
from ros2_aruco_interfaces.msg import ArucoMarkers
import tf_transformations
from collections import deque

# def yaw_from_quaternion(q):
#     x = q.x
#     y = q.y
#     z = q.z
#     w = q.w
#     siny_cosp = 2.0 * (w*z + x*y)
#     cosy_cosp = 1.0 - 2.0 * ( y * y + z * z)
#     return math.atan2(siny_cosp, cosy_cosp)



class GateFollower(Node):

    def __init__(self):
        super().__init__("gate_follower")

        self.cmd_pub = self.create_publisher(Twist,"/cmd_vel",10)

        self.aruco_sub = self.create_subscription(
            ArucoMarkers,
            "/aruco_markers",
            self.marker_map_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )

        self.timer = self.create_timer(0.1, self.control_loop)

        # idマーカ設定
        self.id1 = 2
        self.id2 = 3
        
        # 中点の初期値
        # self.center_x = None
        # self.center_z = None

        # idマーカの保存場所
        self.gates = [
            (2,3),
            (4,5),
            (6,7),
        ]
        self.id1_map = None
        self.id2_map = None
        self.current_gate = 0

        # self.id1_hist = deque(maxlen=5)
        # self.id2_hist = deque(maxlen=5)

        self.gate_ready_count = 0
        self.gate_ready_required = 3

        # odom設定
        self.x, self.y,self.yaw = 0.0, 0.0, 0.0

        self.initial_side = None
        # self.passed_gate = False
        self.cooldown_until = None

    def get_pose(self,msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q_x = msg.pose.pose.orientation.x
        q_y = msg.pose.pose.orientation.y
        q_z = msg.pose.pose.orientation.z
        q_w = msg.pose.pose.orientation.w
        (roll, pitch, yaw) = tf_transformations.euler_from_quaternion(
            (q_x, q_y, q_z, q_w))
        
        return x, y, yaw

    def odom_callback(self,msg):
        self.x, self.y, self.yaw = self.get_pose(msg) 

    # def update_marker_with_jump_check(self, current, new_map, name, max_jump=2.0):
    #     if current is None:
    #         return new_map

    #     d = math.hypot(new_map[0] - current[0], new_map[1] - current[1])

    #     if d > max_jump:
    #         self.get_logger().warn(
    #             f"{name} jump rejected: {current} -> {new_map}, d={d:.2f}"
    #         )
    #         return current

    #     return new_map   
    
    def marker_map_callback(self,msg):
        if self.id1 in msg.marker_ids:
            i1 = msg.marker_ids.index(self.id1)
            p1 = msg.poses[i1].position

            cam_x = p1.x
            cam_z = p1.z

            map_x = self.x + cam_z * math.cos(self.yaw) + cam_x * math.sin(self.yaw)
            map_y = self.y + cam_z * math.sin(self.yaw) - cam_x * math.cos(self.yaw)

            # new_map = (map_x, map_y)
            # new_map = self.update_marker_with_jump_check(
            #     self.id1_map,
            #     new_map,
            #     "id1"
            # )
            # self.id1_hist.append(new_map)

            # avg_x = sum(p[0] for p in self.id1_hist) / len(self.id1_hist)
            # avg_y = sum(p[1] for p in self.id1_hist) / len(self.id1_hist)

            self.id1_map = (map_x, map_y)

            self.get_logger().info(f"id1_map: {self.id1_map}")
            self.get_logger().info(
                f"id1 raw: cam_x={cam_x:.3f}, cam_z={cam_z:.3f}, "
                f"robot=({self.x:.3f},{self.y:.3f},{math.degrees(self.yaw):.1f}deg), "
                f"map=({map_x:.3f},{map_y:.3f})"
            )
        if self.id2 in msg.marker_ids:
            i2 = msg.marker_ids.index(self.id2)
            p2 = msg.poses[i2].position

            cam_x = p2.x
            cam_z = p2.z

            map_x = self.x + cam_z * math.cos(self.yaw) + cam_x * math.sin(self.yaw)
            map_y = self.y + cam_z * math.sin(self.yaw) - cam_x * math.cos(self.yaw)

            # new_map = (map_x, map_y)
            # new_map = self.update_marker_with_jump_check(
            #     self.id1_map,
            #     new_map,
            #     "id1"
            # )
            # self.id1_hist.append(new_map)

            # avg_x = sum(p[0] for p in self.id1_hist) / len(self.id1_hist)
            # avg_y = sum(p[1] for p in self.id1_hist) / len(self.id1_hist)

            # self.id1_map = (avg_x, avg_y)


            self.id2_map = (map_x, map_y)
            self.get_logger().info(f"id2_map: {self.id2_map}")
            self.get_logger().info(
                f"id2 raw: cam_x={cam_x:.3f}, cam_z={cam_z:.3f}, "
                f"robot=({self.x:.3f},{self.y:.3f},{math.degrees(self.yaw):.1f}deg), "
                f"map=({map_x:.3f},{map_y:.3f})"
            )
        

    # def marker_callback(self,msg):

    #     # 両方のマーカが見えたとき
    #     if self.id1 in msg.marker_ids and self.id2 in msg.marker_ids:

    #         i1 = msg.marker_ids.index(self.id1)
    #         i2 = msg.marker_ids.index(self.id2)

    #         p1 = msg.poses[i1].position
    #         p2 = msg.poses[i2].position

    #         x1 = p1.x
    #         z1 = p1.z

    #         x2 = p2.x
    #         z2 = p2.z

    #         self.center_x = (x1+x2)/2
    #         self.center_z = (z1+z2)/2

    #     else:
    #         self.center_x = None
    #         self.center_z = None

    def compute_side(self, ax, ay, bx, by, rx, ry):
        return (bx - ax) * (ry -ay) - (by - ay) * (rx - ax)
    
    def sign_with_deadband(self, v, eps):
        if v > eps:
            return 1
        elif v < -eps:
            return -1
        return 0


    def control_loop(self):

        twist = Twist()
        now_sec = self.get_clock().now().nanoseconds / 1e9

        if self.current_gate >= len(self.gates):
            self.get_logger().info("全てのゲートを通過しました！")
            return
        self.id1, self.id2 = self.gates[self.current_gate]

        if self.cooldown_until is not None and now_sec < self.cooldown_until:
            self.cmd_pub.publish(twist)
            return

        if self.id1_map is not None and self.id2_map is not None:
            x1, y1 = self.id1_map
            x2, y2 = self.id2_map

            center_map_x = (x1 + x2) / 2.0
            center_map_y = (y1 + y2) / 2.0

            dx = center_map_x - self.x
            dy = center_map_y - self.y

            dist = math.hypot(dx, dy)
            target_yaw = math.atan2(dy, dx)

            yaw_error = target_yaw - self.yaw
            yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

            current_side = self.compute_side(x1, y1, x2, y2, self.x, self.y)

            side_eps = 0.01
            gate_width = math.hypot(x2 - x1, y2 - y1)

            if 5.0 <= gate_width <= 7.5 and abs(current_side) > side_eps:
                self.gate_ready_count += 1
            else:
                self.gate_ready_count = 0

            if self.initial_side is None :
                if self.gate_ready_count >= self.gate_ready_required:
                    self.initial_side = current_side
                    self.get_logger().info(f"initial_side: {self.initial_side:.4f}")
                else:
                    self.get_logger().info("waiting gate stable")


            initial_sign = self.sign_with_deadband(self.initial_side, side_eps) if self.initial_side is not None else 0
            current_sign = self.sign_with_deadband(current_side, side_eps)

            crossed = False
            if initial_sign != 0 and current_sign != 0 and initial_sign != current_sign:
                crossed = True

            if abs(current_side) < 0.01 and dist < 0.2:
                crossed = True

            if crossed :
                self.current_gate += 1
                self.get_logger().info(f"ゲートを通過しました！ waypoint: {self.current_gate}")

                twist.linear.x = 0.0
                twist.angular.z = 0.0

                self.id1_map = None
                self.id2_map = None
                self.initial_side = None
                self.cooldown_until = now_sec + 2.0
                self.gate_ready_count = 0

            else:
                # stop_dist = 0.10
                slow_dist = 0.30
                # gate_width = math.hypot(x2 - x1, y2 - y1)


                # if dist < stop_dist:
                #     twist.linear.x = 0.0
                #     twist.angular.z = 0.0
                #     self.get_logger().info("ゴールに到達しました！")

                if self.initial_side is None:
                    twist.linear.x = 0.05
                    twist.angular.z = max(min(0.6 * yaw_error, 0.5), -0.5)
                    self.get_logger().info("進行中")

                else:

                    if dist < slow_dist:
                        twist.linear.x = min(0.15, dist * 0.5)
                        twist.angular.z = max(min(0.5 * yaw_error, 0.2), -0.2)
                        self.get_logger().info("進行中")

                    else:
                        twist.linear.x = min(0.25, dist * 0.4)
                        twist.angular.z = max(min(yaw_error, 0.4), -0.4)

                    self.get_logger().info(
                        f"dist={dist:.2f}, yaw_error={math.degrees(yaw_error):.2f} deg, side={current_side:.4f}, gate_width={gate_width:.2f}"
                    )

        else:
            self.gate_ready_count = 0
            # self.passed_gate = False
            twist.linear.x = 0.0
            twist.angular.z = 0.2

        self.cmd_pub.publish(twist)


        # if self.center_x is not None :
        #     twist.linear.x = min(0.4 ,self.center_z*0.5)
        #     twist.angular.z = max(min(-2.0*self.center_x, 1.0), -1.0)

        # else:
        #     twist.linear.x = 0.05
        #     twist.angular.z = 0.2
        
        # self.cmd_pub.publish(twist)


def main():

    rclpy.init()
    node = GateFollower()
    rclpy.spin(node)


if __name__ == "__main__":
    main()