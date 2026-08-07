#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from unetpy import UnetSocket, DatagramReq, Services

class BuoyBridge(Node):
    def __init__(self):
        super().__init__('buoy_acoustic_bridge')
        
        # 1. KẾT NỐI MODEM UNETSTACK THẬT
        try:
            self.socket = UnetSocket('localhost', 1101)
            self.get_logger().info("[PHAO] Đã cắm Modem UnetStack (Port 1101) thành công!")
        except Exception as e:
            self.get_logger().error(f"Không thể kết nối Modem UnetStack: {e}")

        # 2. BỘ NHỚ LỆNH (Mảng 5 phần tử: t1, t2, t3, t7, depth)
        self.cmds = [0.0, 0.0, 0.0, 0.0, -2.0]

        # 3. LẮNG NGHE TÍN HIỆU TỪ TRẠM BỜ (Đúng kênh RF, đúng chuẩn MultiArray)
        self.create_subscription(Float64MultiArray, '/shore/rf_to_buoy', self.rf_callback, 10)

        # 4. NHỊP TIM SÓNG ÂM (5 giây/lần)
        self.timer = self.create_timer(5.0, self.send_acoustic_data)

    def rf_callback(self, msg):
        # Khi có sóng RF từ bờ truyền ra, lập tức cập nhật toàn bộ mảng lệnh
        if len(msg.data) == 5:
            self.cmds = msg.data

    def send_acoustic_data(self):
        # Rút dữ liệu từ mảng ra để đóng gói
        payload = f"{self.cmds[0]:.2f},{self.cmds[1]:.2f},{self.cmds[2]:.2f},{self.cmds[3]:.2f},{self.cmds[4]:.2f}"
        
        try:
            router = self.socket.gw.agentForService(Services.ROUTING)
            datagram = self.socket.gw.agentForService(Services.DATAGRAM)
            agent = router if router else datagram
            
            if not agent:
                self.get_logger().error("X-QUANG: LÕI MẠNG CHẾT!")
                return
                
            req = DatagramReq(to=31, protocol=0)
            req.data = list(payload.encode('utf-8'))
            req.recipient = agent
            
            res = self.socket.gw.request(req, 2000)
            
            if res is None:
                self.get_logger().error("X-QUANG: TIMEOUT!")
            elif str(res.perf) == 'AGREE':
                self.get_logger().info(f"[PHAO -> SÓNG ÂM] Bắn thành công: {payload}")
            else:
                self.get_logger().error(f"X-QUANG: BỊ TỪ CHỐI")
                
        except Exception as e:
            pass

def main():
    rclpy.init()
    node = BuoyBridge()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if hasattr(node, 'socket'): node.socket.close()
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == '__main__':
    main()
