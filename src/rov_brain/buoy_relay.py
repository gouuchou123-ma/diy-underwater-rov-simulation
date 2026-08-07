#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time
import random

class BuoyRelay(Node):
    def __init__(self):
        super().__init__('buoy_acoustic_relay')
        self.get_logger().info("[TRẠM PHAO] Đã khởi động! Bắt đầu bóp nghẹt băng thông...")

        # --- CẤU HÌNH SÓNG ÂM ---
        self.ACOUSTIC_DELAY = 1.5  # Trễ 1.5 giây
        self.PACKET_LOSS = 0.2     # Rớt mạng 20%
        self.packet_queue = []     # Hàng đợi ngâm dữ liệu

        # 1. CÁI TAI LẮNG NGHE TỪ BỜ (Lấy tín hiệu từ teleop)
        self.create_subscription(Float64, '/shore/pid/target_depth', self.depth_callback, 10)
        self.create_subscription(Float64, '/shore/joint_thruster_1/cmd_vel', lambda msg: self.motor_callback(msg, 'joint_thruster_1'), 10)
        self.create_subscription(Float64, '/shore/joint_thruster_2/cmd_vel', lambda msg: self.motor_callback(msg, 'joint_thruster_2'), 10)
        self.create_subscription(Float64, '/shore/joint_thruster_3/cmd_vel', lambda msg: self.motor_callback(msg, 'joint_thruster_3'), 10)
        self.create_subscription(Float64, '/shore/joint_thruster_7/cmd_vel', lambda msg: self.motor_callback(msg, 'joint_thruster_7'), 10)

        # 2. CÁI MIỆNG PHÁT XUỐNG MẠNG RX ĐÁY BIỂN (Cho rov_bridge nghe)
        self.pub_depth = self.create_publisher(Float64, '/subsea_rx/pid/target_depth', 10)
        self.pubs_motor = {
            'joint_thruster_1': self.create_publisher(Float64, '/subsea_rx/joint_thruster_1/cmd_vel', 10),
            'joint_thruster_2': self.create_publisher(Float64, '/subsea_rx/joint_thruster_2/cmd_vel', 10),
            'joint_thruster_3': self.create_publisher(Float64, '/subsea_rx/joint_thruster_3/cmd_vel', 10),
            'joint_thruster_7': self.create_publisher(Float64, '/subsea_rx/joint_thruster_7/cmd_vel', 10),
        }

        # 3. NHỊP TIM KIỂM TRA HÀNG ĐỢI (0.05s)
        self.create_timer(0.05, self.process_queue)

    # --- PHẦN NÃO: LOGIC GIAM GIỮ DỮ LIỆU ---
    def depth_callback(self, msg):
        self.inject_to_water('depth', msg)

    def motor_callback(self, msg, motor_name):
        self.inject_to_water(motor_name, msg)

    def inject_to_water(self, topic_type, msg):
        # Thuật toán đánh rớt gói tin
        if random.random() < self.PACKET_LOSS: return 
        
        # Đóng gói và gắn mác thời gian giao hàng (hiện tại + 1.5s)
        delivery_time = time.time() + self.ACOUSTIC_DELAY
        self.packet_queue.append({'deliver_at': delivery_time, 'type': topic_type, 'data': msg})

    def process_queue(self):
        current_time = time.time()
        # Vớt những gói tin đã ngâm đủ 1.5 giây ra
        ready_packets = [p for p in self.packet_queue if p['deliver_at'] <= current_time]
        
        for packet in ready_packets:
            if packet['type'] == 'depth':
                self.pub_depth.publish(packet['data'])
            else:
                self.pubs_motor[packet['type']].publish(packet['data'])
            
            # Quăng xuống biển xong thì xóa khỏi hàng đợi
            self.packet_queue.remove(packet)

# --- PHẦN ĐUÔI: ĐỘNG CƠ VĨNH CỬU ---
def main(args=None):
    rclpy.init(args=args)
    node = BuoyRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
