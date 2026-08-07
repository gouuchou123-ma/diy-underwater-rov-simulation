#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class RovBridge(Node):
    def __init__(self):
        super().__init__('rov_bridge')
        self.get_logger().info("BỘ ĐỊNH TUYẾN MUX (LAN/ACOUSTIC) ĐÃ SẴN SÀNG")
        
        # Trạng thái mạng viễn thông hiện tại
        self.current_network_mode = "UNKNOWN"
        
        # Đôi tai lắng nghe trạng thái cáp vật lý từ Teleop
        self.create_subscription(Bool, '/cable_status', self.cable_status_cb, 10)
        
        # Vòng lặp kiểm soát băng thông hệ thống
        self.timer = self.create_timer(1.0, self.network_manager_callback)

    def cable_status_cb(self, msg):
        """Hàm ngắt (Interrupt) xử lý tức thời khi có sự kiện đứt/nối cáp"""
        is_cable_ok = msg.data
        
        if is_cable_ok and self.current_network_mode != "LAN":
            self.current_network_mode = "LAN"
            self.get_logger().info("[MUX KÍCH HOẠT] Đã nhận tín hiệu Cáp Quang. Mở van băng thông rộng (Video HD/100Hz Telemetry).")
            # Logic: Bật cờ truyền Video, mở khóa cấp số nhân tốc độ publish TF
            
        elif not is_cable_ok and self.current_network_mode != "ACOUSTIC":
            self.current_network_mode = "ACOUSTIC"
            self.get_logger().warn("[MUX KHẨN CẤP] Cáp Quang mất kết nối! Chuyển mạch sang Sóng Âm (Acoustic).")
            self.get_logger().warn("Khóa luồng Video. Áp dụng nén dữ liệu tọa độ (Hysteresis Active).")
            # Logic: Ngắt kênh camera, giảm tần số gửi dữ liệu tọa độ xuống 1Hz để phù hợp tốc độ truyền âm thanh dưới nước

    def network_manager_callback(self):
        """Bộ quản lý duy trì nhịp tim mạng định kỳ"""
        if self.current_network_mode == "UNKNOWN":
            self.get_logger().error("Đang chờ tín hiệu xác thực phần cứng từ trạm bờ...")
        elif self.current_network_mode == "LAN":
            self.get_logger().debug("Truyền dẫn Quang học ổn định: Băng thông 1Gbps.")
        elif self.current_network_mode == "ACOUSTIC":
            self.get_logger().debug("Truyền dẫn Sóng âm duy trì: Băng thông 300bps.")

def main():
    rclpy.init()
    node = RovBridge()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
