import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Int32MultiArray
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler

class OdomPublisher(Node):
    def __init__(self):
        super().__init__('odom_publisher')

        self.encorder_sub = self.create_subscription(Int32MultiArray, 'encoder', self.read_robot_state, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.vx = 0.0
        self.wz = 0.0

        self.wheel_radius = 0.3
        self.wheel_base = 0.82

        self.rw = 0.0
        self.lw = 0.0
        self.fs = 0.0
        self.rs = 0.0


        self.prev_rw = None
        self.prev_lw = None

        self.last_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.update)

    def read_robot_state(self, msg):
        self.rw = msg.data[0] / 1000.0
        self.lw = msg.data[1] / 1000.0
        self.fs = msg.data[2] / 1000.0
        self.rs = msg.data[3] / 1000.0

        pass

    def update(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanodeconds * 1e-9
        self.last_time = current_time

        if self.prec_rw is None or self.prec_lw is None:
            self.prec_rw = self.rw
            self.prev_lw = self.lw
            return 
        
        delta_rw = self.rw - self.prev_rw
        delta_lw = self.lw - self.prev_lw

        self.prev_rw = self.rw
        self.prev_lw = self.lw

        # 距離差分
        ds_r = delta_rw * 2 * math.pi * self.wheel_radius
        ds_l = delta_lw * 2 * math.pi * self.wheel_radius

        # 距離速度、速度
        ds = (ds_r + ds_l) / 2.0
        self.vx = ds / dt

        # 角速度
        self.wz = self.vx * {
            math.tan(self.fs) - math.tan((self.rs))
        } / self.wheel_base
        
        # オドメトリ積分
        self.yaw += self.wz * dt
        self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))

        self.x += self.vx * math.cos(self.yaw) * dt
        self.y += self.vx * math.sin(self.yaw) * dt

        q = quaternion_from_euler(0, 0, self.yaw)

        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.angular.z = self.wz

        self.odom_pub.publish(odom)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = current_time.to_msg()
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_link'

        tf_msg.pose.pose.position.x = self.x
        tf_msg.pose.pose.position.y = self.y
        tf_msg.pose.pose.position.z = 0.0
        tf_msg.pose.pose.orientation.x = q[0]
        tf_msg.pose.pose.orientation.y = q[1]
        tf_msg.pose.pose.orientation.z = q[2]
        tf_msg.pose.pose.orientation.w = q[3]

        tf_msg.twist.twist.linear.x = self.vx
        tf_msg.twist.twist.angular.z = self.wz

        self.tf_broadcaster.sendTransform(tf_msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



        
        
        



        