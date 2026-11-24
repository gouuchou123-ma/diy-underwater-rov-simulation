#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64


class AutoDepthPID(Node):
    def __init__(self):
        super().__init__('auto_depth_pid')
        self.get_logger().info('[PID] Anti-oscillation depth controller initialized.')

        # 1. Subscribers
        self.create_subscription(PointStamped, '/sensor/depth', self.depth_callback, 10)
        self.create_subscription(Float64, '/subsea/pid/target_depth', self.target_callback, 10)
        self.create_subscription(Float64, '/rov/height_correction', self.correction_callback, 10)

        # 2. Publishers
        self.pub_t5 = self.create_publisher(Float64, '/joint_thruster_5/cmd_vel', 10)
        self.pub_t6 = self.create_publisher(Float64, '/joint_thruster_6/cmd_vel', 10)
        self.pub_effective_target = self.create_publisher(
            Float64, '/subsea/pid/effective_target_depth', 10
        )

        # 3. Tuned PID gains (reduced Kd to eliminate phase-lag oscillation)
        self.Kp = 2.0
        self.Ki = 0.05
        self.Kd = 4.0

        # 4. Controller state variables
        self.target_depth = -2.0
        self.height_correction = 0.0
        self.smooth_dh = 0.0
        self.current_depth = 0.0
        self.prev_depth = 0.0
        self.has_depth = False  # Prevents derivative kick on initial sample
        self.smooth_d = 0.0     # Low-pass filtered derivative term
        self.integral_error = 0.0
        self.dt = 0.05

        # 20 Hz closed-loop timer
        self.timer = self.create_timer(self.dt, self.pid_compute)

    def depth_callback(self, msg):
        if not self.has_depth:
            self.prev_depth = msg.point.z
            self.has_depth = True
        self.current_depth = msg.point.z

    def target_callback(self, msg):
        self.target_depth = msg.data

    def correction_callback(self, msg):
        dh = max(-3.0, min(3.0, msg.data))
        self.height_correction = dh

    def pid_compute(self):
        if not self.has_depth:
            return

        # Step 0: Apply EMA low-pass filter to height correction (alpha = 0.08)
        self.smooth_dh = 0.92 * self.smooth_dh + 0.08 * self.height_correction
        effective_target = self.target_depth + self.smooth_dh
        self.pub_effective_target.publish(Float64(data=effective_target))

        # Step 1: Compute depth error
        error = effective_target - self.current_depth

        # Step 2: Integral term with anti-windup clamp
        self.integral_error += error * self.dt
        self.integral_error = max(-5.0, min(5.0, self.integral_error))

        # Step 3: Filtered Derivative-on-Measurement
        raw_d = -(self.current_depth - self.prev_depth) / self.dt
        self.prev_depth = self.current_depth
        self.smooth_d = 0.7 * self.smooth_d + 0.3 * raw_d

        # Step 4: Compute clamped PID output
        control_output = (
            (self.Kp * error)
            + (self.Ki * self.integral_error)
            + (self.Kd * self.smooth_d)
        )
        control_output = max(-35.0, min(35.0, control_output))

        # Step 5: Publish vertical thruster commands
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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
