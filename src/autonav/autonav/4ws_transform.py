import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int32MultiArray, Float32MultiArray, Int32
import math

class TransformPublisher(Node):
        def clip_angle(self, angle):
            return max(-45, min(45, angle))
        def __init__(self):
            super().__init__('transform_publisher')

            self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_callback, 10)
            self.mode_sub = self.create_subscription(Int32, 'mode', self.mode_callback, 10)
            self.omega_pub = self.create_publisher(Float32MultiArray, 'omega', 10)
            self.steer_pub = self.create_publisher(Int32MultiArray, 'steer', 10)

            # 機体の物理パラメータ
            self.wheel_radius = 0.3
            self.wheel_base = 0.82
            self.track_width = 0.83

            # 送る値の初期値
            self.rw = 0.0
            self.lw = 0.0
            self.lfs = 0.0
            self.rfs = 0.0
            self.lrs = 0.0
            self.rrs = 0.0

            # モード選択
            self.mode = 1

        def mode_callback(self, msg):
            self.mode = msg.data
            self.get_logger().info(f"現在のモードは{self.mode}です")
            

        def cmd_callback(self, msg):
            vx = msg.linear.x
            wz = msg.angular.z
            if self.mode == 1:
                self.publish_opposite_phase(vx, wz)
            elif self.mode == 2:
                self.publish_pivot_turn(wz)
            else:
                self.publish_stop()

        def publish_opposite_phase(self,vx,wz):

            if vx > 0:
                sign = 1
            elif vx < 0:
                sign = -1
            else:
                sign = 0

            v_right = sign*math.hypot(vx + wz*self.track_width/2, wz*self.wheel_base/2)
            v_left = sign*math.hypot(vx - wz*self.track_width/2, wz*self.wheel_base/2)

            self.rw = round(v_right / self.wheel_radius, 3)
            self.lw = round(v_left / self.wheel_radius, 3)

            lfs_rad = math.atan2(wz*self.wheel_base, 2*vx - wz*self.track_width)
            rfs_rad = math.atan2(wz*self.wheel_base, 2*vx + wz*self.track_width)
            lrs_rad = -lfs_rad
            rrs_rad = -rfs_rad

            self.lfs = int(round(self.clip_angle(math.degrees(lfs_rad))))
            self.rfs = int(round(self.clip_angle(math.degrees(rfs_rad))))
            self.lrs = int(round(self.clip_angle(math.degrees(lrs_rad))))
            self.rrs = int(round(self.clip_angle(math.degrees(rrs_rad))))

            omega_msg = Float32MultiArray()
            omega_msg.data = [self.rw, self.lw]
            self.omega_pub.publish(omega_msg)

            steer_msg = Int32MultiArray()
            steer_msg.data = [self.lfs, self.rfs, self.lrs, self.rrs]
            self.steer_pub.publish(steer_msg)
        
        def publish_pivot_turn(self, wz):

            v_right = -wz
            v_left = wz
            self.rw = round(v_right / self.wheel_radius, 3)
            self.lw = round(v_left / self.wheel_radius, 3)

            pivot_angle = math.degrees(math.atan2(self.wheel_base, self.track_width))
            self.lfs = int(round(self.clip_angle(-pivot_angle)))
            self.rfs = int(round(self.clip_angle(pivot_angle)))
            self.lrs = int(round(self.clip_angle(pivot_angle)))
            self.rrs = int(round(self.clip_angle(-pivot_angle)))

            omega_msg = Float32MultiArray()
            omega_msg.data = [self.rw, self.lw]
            self.omega_pub.publish(omega_msg)   

            steer_msg = Int32MultiArray()
            steer_msg.data = [self.lfs, self.rfs, self.lrs, self.rrs]
            self.steer_pub.publish(steer_msg)
        
        def publish_stop(self):
            self.rw = 0.0
            self.lw = 0.0
            self.lfs = 0
            self.rfs = 0
            self.lrs = 0
            self.rrs = 0

            omega_msg = Float32MultiArray()
            omega_msg.data = [self.rw, self.lw]
            self.omega_pub.publish(omega_msg)   

            steer_msg = Int32MultiArray()
            steer_msg.data = [self.lfs, self.rfs, self.lrs, self.rrs]
            self.steer_pub.publish(steer_msg)

def main(args=None):
    rclpy.init(args=args)
    transform_publisher = TransformPublisher()
    rclpy.spin(transform_publisher)
    transform_publisher.destroy_node()
    rclpy.shutdown()

        