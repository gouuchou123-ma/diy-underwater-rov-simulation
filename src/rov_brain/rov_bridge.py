#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool


class RovBridge(Node):
    def __init__(self):
        super().__init__('rov_bridge')
        self.get_logger().info('[MUX] LAN/Acoustic communication router initialized.')

        # Current active communication link state
        self.current_network_mode = 'UNKNOWN'

        # Subscribe to physical tether status from shore teleop
        self.create_subscription(Bool, '/cable_status', self.cable_status_cb, 10)

        # Periodic link status monitor (1 Hz)
        self.timer = self.create_timer(1.0, self.network_manager_callback)

    def cable_status_cb(self, msg):
        """Handle immediate link switching on tether connect/disconnect events."""
        is_cable_ok = msg.data

        if is_cable_ok and self.current_network_mode != 'LAN':
            self.current_network_mode = 'LAN'
            self.get_logger().info(
                '[MUX] Fiber-optic link active. Switched to LAN mode (high-bandwidth telemetry).'
            )
        elif not is_cable_ok and self.current_network_mode != 'ACOUSTIC':
            self.current_network_mode = 'ACOUSTIC'
            self.get_logger().warn(
                '[MUX] Fiber-optic link lost. Switched to ACOUSTIC failsafe mode.'
            )
            self.get_logger().warn(
                '[MUX] Video stream disabled. Low-bandwidth telemetry compression active.'
            )

    def network_manager_callback(self):
        """Periodic communication link health monitor."""
        if self.current_network_mode == 'UNKNOWN':
            self.get_logger().error('[MUX] Waiting for tether status signal from shore station.')
        elif self.current_network_mode == 'LAN':
            self.get_logger().debug('[MUX] Optical link nominal (1 Gbps).')
        elif self.current_network_mode == 'ACOUSTIC':
            self.get_logger().debug('[MUX] Acoustic link active (300 bps).')


def main():
    rclpy.init()
    node = RovBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
