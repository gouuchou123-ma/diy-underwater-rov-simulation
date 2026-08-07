#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, Bool 

class RovTeleop(Node):
    def __init__(self):
        super().__init__('rov_teleop')
        self.get_logger().info("BỘ ĐIỀU KHIỂN TỪ XA HMI ĐÃ SẴN SÀNG")
        
        self.cable_connected = True
        self.target_depth = -0.0
        self.t1 = 0.0; self.t2 = 0.0; self.t3 = 0.0; self.t7 = 0.0
        
        # --- BIẾN KIỂM SOÁT POSEIDON ---
        self.poseidon_on = False
        self.poseidon_intensity = 20.0
        self.last_dpad_y = 0.0 
        
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        
        # Các kênh phát tín hiệu ROS 2
        self.pub_t1 = self.create_publisher(Float64, '/joint_thruster_1/cmd_vel', 10)
        self.pub_t2 = self.create_publisher(Float64, '/joint_thruster_2/cmd_vel', 10)
        self.pub_t3 = self.create_publisher(Float64, '/joint_thruster_3/cmd_vel', 10)
        self.pub_t7 = self.create_publisher(Float64, '/joint_thruster_7/cmd_vel', 10)
        
        self.pub_target_depth = self.create_publisher(Float64, '/subsea/pid/target_depth', 10)
        self.pub_cable_status = self.create_publisher(Bool, '/cable_status', 10)
        self.pub_poseidon_on = self.create_publisher(Bool, '/poseidon/enable', 10)
        self.pub_poseidon_int = self.create_publisher(Float64, '/poseidon/intensity', 10)
        
        self.timer = self.create_timer(0.1, self.timer_callback)

    def joy_callback(self, msg):
        try:
            # 1. TRỤC DI CHUYỂN
            surge = msg.axes[1] * 50.0  
            sway = msg.axes[0] * -100.0 # Vẫn giữ mức 100 để thắng lực cản ngang
            yaw = msg.axes[3] * -50.0   

            self.get_logger().info(f"Lệnh: Surge:{surge: .1f} | Sway:{sway: .1f} | Yaw:{yaw: .1f}")

            # 2. MA TRẬN PHÂN BỔ LỰC ĐẨY (Đã sửa lỗi đồng bộ Vector Vật lý)
            self.t7 = surge + sway + yaw  # Động cơ Trước - Trái (Đã đảo dấu Sway và Yaw)
            self.t3 = surge - sway + yaw  # Động cơ Sau - Trái
            self.t2 = surge - sway - yaw  # Động cơ Trước - Phải
            self.t1 = surge + sway - yaw  # Động cơ Sau - Phải

            # 2. ĐỘ SÂU (Nút vai: RB=5, LB=4)
            if msg.buttons[5] == 1:   # RB: Nổi
                self.target_depth += 0.05
                if self.target_depth > 0.0: self.target_depth = 0.0
            elif msg.buttons[4] == 1: # LB: Lặn
                self.target_depth -= 0.05

            # 3. QUẢN LÝ CÁP (Nút: B=1, A=0)
            if msg.buttons[1] == 1:   # B: Cắt cáp
                if self.cable_connected:
                    self.get_logger().error("ĐÃ CẮT CÁP QUANG TỪ BỜ!")
                    self.cable_connected = False
            elif msg.buttons[0] == 1: # A: Nối cáp
                if not self.cable_connected:
                    self.get_logger().info("ĐÃ NỐI LẠI CÁP QUANG!")
                    self.cable_connected = True

            # 4. TRIỆU HỒI POSEIDON (Nút: Y=3, X=2)
            if msg.buttons[3] == 1:   # Y: Khởi động bão
                if not self.poseidon_on:
                    self.get_logger().warn("Đã kích hoạt Poseidon")
                    self.poseidon_on = True
            elif msg.buttons[2] == 1: # X: Ngừng bão
                if self.poseidon_on:
                    self.get_logger().info("Đã tắt Poseidon.")
                    self.poseidon_on = False

            # 5. CƯỜNG ĐỘ BÃO (D-pad: axes 7)
            current_dpad = msg.axes[7]
            if current_dpad > 0.5 and self.last_dpad_y <= 0.5:
                self.poseidon_intensity += 5.0
                self.get_logger().warn(f"TĂNG cấp độ bão ngầm: {self.poseidon_intensity}")
            elif current_dpad < -0.5 and self.last_dpad_y >= -0.5:
                self.poseidon_intensity -= 5.0
                if self.poseidon_intensity < 0.0: self.poseidon_intensity = 0.0
                self.get_logger().warn(f"GIẢM cấp độ bão ngầm: {self.poseidon_intensity}")
            self.last_dpad_y = current_dpad

        except IndexError:
            pass

    def timer_callback(self):
        # Phát trạng thái hệ thống cho các node khác (Foxglove, PID, Webots)
        self.pub_cable_status.publish(Bool(data=self.cable_connected))
        self.pub_poseidon_on.publish(Bool(data=self.poseidon_on))
        self.pub_poseidon_int.publish(Float64(data=self.poseidon_intensity))

        # Xuất lệnh ra Motor
        if self.cable_connected:
            self.pub_target_depth.publish(Float64(data=self.target_depth))
            self.pub_t1.publish(Float64(data=self.t1))
            self.pub_t2.publish(Float64(data=self.t2))
            self.pub_t3.publish(Float64(data=self.t3))
            self.pub_t7.publish(Float64(data=self.t7))
        else:
            self.pub_target_depth.publish(Float64(data=0.0))
            self.pub_t1.publish(Float64(data=0.0))
            self.pub_t2.publish(Float64(data=0.0))
            self.pub_t3.publish(Float64(data=0.0))
            self.pub_t7.publish(Float64(data=0.0))

def main():
    rclpy.init()
    node = RovTeleop()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
