#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from unetpy import UnetSocket, DatagramReq, Services


class BuoyBridge(Node):
    def __init__(self):
        super().__init__('buoy_acoustic_bridge')

        # 1. Connect to UnetStack acoustic modem (Port 1101)
        try:
            self.socket = UnetSocket('localhost', 1101)
            self.get_logger().info('[BUOY] Connected to UnetStack modem on localhost:1101.')
        except Exception as e:
            self.get_logger().error(f'[BUOY] Failed to connect to UnetStack modem: {e}')

        # 2. Latest command buffer [t1, t2, t3, t7, target_depth]
        self.cmds = [0.0, 0.0, 0.0, 0.0, -2.0]

        # 3. Subscribe to shore RF relay topic
        self.create_subscription(Float64MultiArray, '/shore/rf_to_buoy', self.rf_callback, 10)

        # 4. Acoustic transmission heartbeat (0.2 Hz / every 5.0 s)
        self.timer = self.create_timer(5.0, self.send_acoustic_data)

    def rf_callback(self, msg):
        if len(msg.data) == 5:
            self.cmds = msg.data

    def send_acoustic_data(self):
        payload = (
            f'{self.cmds[0]:.2f},{self.cmds[1]:.2f},'
            f'{self.cmds[2]:.2f},{self.cmds[3]:.2f},{self.cmds[4]:.2f}'
        )

        try:
            router = self.socket.gw.agentForService(Services.ROUTING)
            datagram = self.socket.gw.agentForService(Services.DATAGRAM)
            agent = router if router else datagram

            if not agent:
                self.get_logger().error('[ACOUSTIC] No ROUTING or DATAGRAM agent available.')
                return

            req = DatagramReq(to=31, protocol=0)
            req.data = list(payload.encode('utf-8'))
            req.recipient = agent

            res = self.socket.gw.request(req, 2000)

            if res is None:
                self.get_logger().error('[ACOUSTIC] Transmission timeout (2000 ms).')
            elif str(res.perf) == 'AGREE':
                self.get_logger().info(f'[ACOUSTIC] Transmitted payload: {payload}')
            else:
                self.get_logger().error(f'[ACOUSTIC] Transmission rejected: {res.perf}')
        except Exception:
            pass


def main():
    rclpy.init()
    node = BuoyBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'socket'):
            node.socket.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
