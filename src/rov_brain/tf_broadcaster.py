#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PointStamped, TransformStamped
from tf2_ros import TransformBroadcaster

class RovTfBroadcaster(Node):
    def __init__(self):
        super().__init__('rov_tf_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)

        # Mở khóa toàn bộ không gian 3D
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        self.create_subscription(PointStamped, '/sensor/depth', self.gps_cb, 10)
        self.create_subscription(Imu, '/imu', self.imu_cb, 10)

    def gps_cb(self, msg):
        # Lấy đầy đủ X, Y, Z từ Webots GPS
        self.current_x = msg.point.x
        self.current_y = msg.point.y
        self.current_z = msg.point.z

    def imu_cb(self, msg):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'world'       
        # Bắn mũi tên tọa độ vào CÁI BÓNG thay vì xác phàm
        t.child_frame_id = 'base_footprint' 

        t.transform.translation.x = self.current_x
        t.transform.translation.y = self.current_y
        t.transform.translation.z = self.current_z
        t.transform.rotation = msg.orientation

        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = RovTfBroadcaster()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
