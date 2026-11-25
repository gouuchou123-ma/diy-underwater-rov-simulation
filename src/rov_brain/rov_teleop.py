#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Float64, Bool


class RovTeleop(Node):
    def __init__(self):
        super().__init__('rov_teleop')
        self.get_logger().info('[TELEOP] HMI teleoperation node initialized.')

        self.cable_connected = True
        self.target_depth = 0.0
        self.t1 = 0.0
        self.t2 = 0.0
        self.t3 = 0.0
        self.t7 = 0.0

        # Poseidon disturbance control state
        self.poseidon_on = False
        self.poseidon_intensity = 20.0
        self.last_dpad_y = 0.0

        self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # ROS 2 command and status publishers
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
            # 1. Motion axes mapping
            surge = msg.axes[1] * 50.0
            sway = msg.axes[0] * -100.0  # Higher gain to overcome lateral drag
            yaw = msg.axes[3] * -50.0

            self.get_logger().info(
                f'[TELEOP] Cmd | Surge={surge:.1f} | Sway={sway:.1f} | Yaw={yaw:.1f}'
            )

            # 2. Thruster Allocation Matrix (TAM)
            self.t7 = surge + sway + yaw  # Front-Left thruster
            self.t3 = surge - sway + yaw  # Rear-Left thruster
            self.t2 = surge - sway - yaw  # Front-Right thruster
            self.t1 = surge + sway - yaw  # Rear-Right thruster

            # 3. Target depth control (Shoulder buttons: RB=5, LB=4)
            if msg.buttons[5] == 1:    # RB: Ascend
                self.target_depth += 0.05
                if self.target_depth > 0.0:
                    self.target_depth = 0.0
            elif msg.buttons[4] == 1:  # LB: Descend
                self.target_depth -= 0.05

            # 4. Tether failsafe simulation (Buttons: B=1, A=0)
            if msg.buttons[1] == 1:    # B: Disconnect tether
                if self.cable_connected:
                    self.get_logger().error('[TELEOP] Fiber-optic tether disconnected.')
                    self.cable_connected = False
            elif msg.buttons[0] == 1:  # A: Reconnect tether
                if not self.cable_connected:
                    self.get_logger().info('[TELEOP] Fiber-optic tether reconnected.')
                    self.cable_connected = True

            # 5. Poseidon disturbance toggle (Buttons: Y=3, X=2)
            if msg.buttons[3] == 1:    # Y: Enable disturbance
                if not self.poseidon_on:
                    self.get_logger().warn('[TELEOP] Poseidon disturbance enabled.')
                    self.poseidon_on = True
            elif msg.buttons[2] == 1:  # X: Disable disturbance
                if self.poseidon_on:
                    self.get_logger().info('[TELEOP] Poseidon disturbance disabled.')
                    self.poseidon_on = False

            # 6. Disturbance intensity adjustment (D-pad vertical: axis 7)
            current_dpad = msg.axes[7]
            if current_dpad > 0.5 and self.last_dpad_y <= 0.5:
                self.poseidon_intensity += 5.0
                self.get_logger().warn(
                    f'[TELEOP] Poseidon intensity increased: {self.poseidon_intensity:.1f}'
                )
            elif current_dpad < -0.5 and self.last_dpad_y >= -0.5:
                self.poseidon_intensity -= 5.0
                if self.poseidon_intensity < 0.0:
                    self.poseidon_intensity = 0.0
                self.get_logger().warn(
                    f'[TELEOP] Poseidon intensity decreased: {self.poseidon_intensity:.1f}'
                )
            self.last_dpad_y = current_dpad

        except IndexError:
            pass

    def timer_callback(self):
        # Broadcast system status to downstream nodes
        self.pub_cable_status.publish(Bool(data=self.cable_connected))
        self.pub_poseidon_on.publish(Bool(data=self.poseidon_on))
        self.pub_poseidon_int.publish(Float64(data=self.poseidon_intensity))

        # Publish thruster and depth commands when tethered; neutralize when disconnected
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
