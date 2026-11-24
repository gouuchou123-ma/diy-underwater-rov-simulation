#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import time
import random

class BuoyRelay(Node):
    def __init__(self):
        super().__init__('buoy_acoustic_relay')
        self.get_logger().info("Đã khởi động! Bắt đầu bóp băng thông...")

        self.ACOUSTIC_DELAY = 1.5  
        self.PACKET_LOSS = 0.2     
        self.packet_queue = []     


        self.create_subscription(Float64, '/shore/pid/target_depth', self.depth_callback, 10)
        self.create_subscription(Float64, '/shore/joint_thruster_1/cmd_vel', lambda msg: self.motor_callback(msg, 'joint_thruster_1'), 10)
        self.create_subscription(Float64, '/shore/joint_thruster_2/cmd_vel', lambda msg: self.motor_callback(msg, 'joint_thruster_2'), 10)
        self.create_subscription(Float64, '/shore/joint_thruster_3/cmd_vel', lambda msg: self.motor_callback(msg, 'joint_thruster_3'), 10)
        self.create_subscription(Float64, '/shore/joint_thruster_7/cmd_vel', lambda msg: self.motor_callback(msg, 'joint_thruster_7'), 10)

        self.pub_depth = self.create_publisher(Float64, '/subsea_rx/pid/target_depth', 10)
        self.pubs_motor = {
            'joint_thruster_1': self.create_publisher(Float64, '/subsea_rx/joint_thruster_1/cmd_vel', 10),
            'joint_thruster_2': self.create_publisher(Float64, '/subsea_rx/joint_thruster_2/cmd_vel', 10),
            'joint_thruster_3': self.create_publisher(Float64, '/subsea_rx/joint_thruster_3/cmd_vel', 10),
            'joint_thruster_7': self.create_publisher(Float64, '/subsea_rx/joint_thruster_7/cmd_vel', 10),
        }

        self.create_timer(0.05, self.process_queue)

    def depth_callback(self, msg):
        self.inject_to_water('depth', msg)

    def motor_callback(self, msg, motor_name):
        self.inject_to_water(motor_name, msg)

    def inject_to_water(self, topic_type, msg):

        if random.random() < self.PACKET_LOSS: return 
        
        delivery_time = time.time() + self.ACOUSTIC_DELAY
        self.packet_queue.append({'deliver_at': delivery_time, 'type': topic_type, 'data': msg})

    def process_queue(self):
        current_time = time.time()

        ready_packets = [p for p in self.packet_queue if p['deliver_at'] <= current_time]
        
        for packet in ready_packets:
            if packet['type'] == 'depth':
                self.pub_depth.publish(packet['data'])
            else:
                self.pubs_motor[packet['type']].publish(packet['data'])
            
            self.packet_queue.remove(packet)

def main(args=None):
    rclpy.init(args=args)
    node = BuoyRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
