#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64

class AutoDepthPID(Node):
    def __init__(self):
        super().__init__('auto_depth_pid')
        self.get_logger().info("[PID] Hệ thống Lái tự động Đã Khởi Động!")

        # 1. KÊNH LẮNG NGHE (Đã cập nhật để nghe sóng âm dưới đáy biển)
        self.create_subscription(PointStamped, '/sensor/depth', self.depth_callback, 10)
        self.create_subscription(Float64, '/subsea/pid/target_depth', self.target_callback, 10)        
        
        # 2. KÊNH PHÁT LỆNH (Trực tiếp xuống Webots, loại bỏ namespace /shore/ rác)
        self.pub_t5 = self.create_publisher(Float64, '/joint_thruster_5/cmd_vel', 10)
        self.pub_t6 = self.create_publisher(Float64, '/joint_thruster_6/cmd_vel', 10)

        # 3. THÔNG SỐ PID (Có thể tinh chỉnh sau)
        self.Kp = 15.0  # Lực kéo lò xo (Mạnh/Yếu)
        self.Ki = 0.5   # Bù trừ lực nổi (Tích lũy)
        self.Kd = 5.0  # Phanh giảm xóc (Chống lố)

        # 4. BỘ NHỚ PID
        self.target_depth = -2.0  # Mục tiêu mặc định: Neo tàu ở độ sâu 2 mét
        self.current_depth = 0.0
        self.integral_error = 0.0
        self.previous_error = 0.0

        # Vòng lặp PID chạy liên tục 20 lần/giây (0.05s)
        self.timer = self.create_timer(0.05, self.pid_compute)


    def depth_callback(self, msg):
        # Trục Z của GPS Webots thường âm khi chìm xuống nước
        self.current_depth = msg.point.z 

    def target_callback(self, msg):
        self.target_depth = msg.data
        self.get_logger().info(f"[PID] Đã nhận lệnh thay đổi độ sâu mục tiêu: {self.target_depth} mét")

    def pid_compute(self):
        # Bước 1: Tính toán sai số (Error)
        error = self.target_depth - self.current_depth

        # Bước 2: Tích phân (Integral) - Cộng dồn sai số theo thời gian
        self.integral_error += error * 0.05
        
        # Chống tràn bộ nhớ Integral (Windup Guard)
        if self.integral_error > 50.0: self.integral_error = 50.0
        if self.integral_error < -50.0: self.integral_error = -50.0

        # Bước 3: Đạo hàm (Derivative) - Tính tốc độ thay đổi của sai số
        derivative = (error - self.previous_error) / 0.05
        self.previous_error = error

        # Bước 4: TỔNG HỢP LỰC PID
        # Output = (P) + (I) + (D)
        control_output = (self.Kp * error) + (self.Ki * self.integral_error) + (self.Kd * derivative)

        # Giới hạn lực động cơ tối đa (Bảo vệ motor)
        if control_output > 100.0: control_output = 100.0
        if control_output < -100.0: control_output = -100.0

        # Bước 5: Bơm ga cho 2 động cơ lặn nổi
        self.pub_t5.publish(Float64(data=control_output))
        self.pub_t6.publish(Float64(data=control_output))

def main(args=None):
    rclpy.init(args=args)
    node = AutoDepthPID()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
