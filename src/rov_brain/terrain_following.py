#!/usr/bin/env python3
import math
import random
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, Imu
from std_msgs.msg import Float64


def pitch_from_quaternion(x, y, z, w):
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    return math.asin(t2)


class TerrainFollower(Node):
    def __init__(self):
        super().__init__('terrain_follower_node')
        self.get_logger().info('[SONAR] 6-Scenario terrain-following node initialized.')

        # Subscribers
        self.sub_front = self.create_subscription(Range, '/sonar/front', self.front_callback, 10)
        self.sub_bottom = self.create_subscription(Range, '/sonar/bottom', self.bottom_callback, 10)
        self.sub_rear = self.create_subscription(Range, '/sonar/rear', self.rear_callback, 10)
        self.sub_imu = self.create_subscription(Imu, '/imu', self.imu_callback, 10)

        # Publishers
        self.pub_height_cmd = self.create_publisher(Float64, '/rov/height_correction', 10)
        self.pub_pitch_deg = self.create_publisher(Float64, '/rov/pitch_deg', 10)

        # Internal state
        self.d1 = 50.0
        self.d2 = 50.0
        self.d3 = 50.0
        self.imu_theta = 0.0
        self.theta = 0.0  # Effective pitch angle (rad) used in kinematic projection
        self.prev_dh = 0.0
        self.U_T = 6.0    # Height modification gain

        # 10 Hz control loop
        self.timer = self.create_timer(0.1, self.calculate_terrain)

    def front_callback(self, msg):
        self.d1 = msg.range

    def bottom_callback(self, msg):
        self.d2 = msg.range

    def rear_callback(self, msg):
        self.d3 = msg.range

    def imu_callback(self, msg):
        q = msg.orientation
        self.imu_theta = pitch_from_quaternion(q.x, q.y, q.z, q.w)

    def calculate_terrain(self):
        # Open-water guard: disable height override if bottom sonar exceeds range limit
        if self.d2 > 30.0:
            self.pub_height_cmd.publish(Float64(data=0.0))
            self.pub_pitch_deg.publish(Float64(data=0.0))
            return

        valid_d2 = self.d2
        raw_d1 = self.d1 if self.d1 <= 30.0 else (valid_d2 / 0.866)
        raw_d3 = self.d3 if self.d3 <= 30.0 else (valid_d2 / 0.866)

        d_front = min(raw_d1, raw_d3)
        d_rear = max(raw_d1, raw_d3) if abs(raw_d1 - raw_d3) > 0.15 else raw_d3

        # 1. Project sonar rays onto vertical (X, Z) plane with beam angles 60, 90, 120 deg
        a1 = math.radians(60.0)
        a2 = math.radians(90.0)
        a3 = math.radians(120.0)

        x_A = d_front * math.cos(a1) * math.cos(self.theta) - d_front * math.sin(a1) * math.sin(self.theta)
        z_A = d_front * math.cos(a1) * math.sin(self.theta) + d_front * math.sin(a1) * math.cos(self.theta)

        x_B = valid_d2 * math.cos(a2) * math.cos(self.theta) - valid_d2 * math.sin(a2) * math.sin(self.theta)
        z_B = valid_d2 * math.cos(a2) * math.sin(self.theta) + valid_d2 * math.sin(a2) * math.cos(self.theta)

        x_C = d_rear * math.cos(a3) * math.cos(self.theta) - d_rear * math.sin(a3) * math.sin(self.theta)
        z_C = d_rear * math.cos(a3) * math.sin(self.theta) + d_rear * math.sin(a3) * math.cos(self.theta)

        # 2. Extract seabed slope gradients (K_AB, K_BC, K_AC) with deadzone filter
        dx_AB = abs(x_A - x_B)
        dx_BC = abs(x_B - x_C)
        dx_AC = abs(x_A - x_C)

        raw_K_AB = (z_B - z_A) / dx_AB if dx_AB > 0.05 else 0.0
        raw_K_BC = (z_C - z_B) / dx_BC if dx_BC > 0.05 else 0.0
        raw_K_AC = (z_C - z_A) / dx_AC if dx_AC > 0.05 else 0.0

        TOLERANCE = 0.02
        K_AB = 0.0 if abs(raw_K_AB) < TOLERANCE else raw_K_AB
        K_BC = 0.0 if abs(raw_K_BC) < TOLERANCE else raw_K_BC
        K_AC = 0.0 if abs(raw_K_AC) < TOLERANCE else raw_K_AC

        # 3. Classify terrain into 6 canonical scenarios and compute slope correction
        slope_dh = 0.0
        scenario_str = 'Flat Terrain'

        if K_AB > K_BC and K_BC >= 0.0 and K_AB > 0.0:
            scenario_str = 'Scenario 1: Steep Uphill'
            slope_dh = self.U_T * K_AB
        elif K_AB > 0.0 and K_BC < 0.0:
            scenario_str = 'Scenario 2: Narrow Trench'
            slope_dh = self.U_T * K_AC
        elif K_BC > K_AB and K_BC <= 0.0 and K_AB < 0.0:
            scenario_str = 'Scenario 3: Steep Downhill'
            slope_dh = self.U_T * K_BC
        elif K_BC > 0.0 and K_AB < 0.0:
            scenario_str = 'Scenario 4: Small Mound'
            slope_dh = self.U_T * K_AC
        elif K_BC >= K_AB and K_AB >= 0.0 and K_BC > 0.0:
            scenario_str = 'Scenario 5: Gentle Uphill'
            slope_dh = self.U_T * K_BC
        elif K_AB >= K_BC and K_AB <= 0.0 and K_BC < 0.0:
            scenario_str = 'Scenario 6: Gentle Downhill'
            slope_dh = self.U_T * K_AB

        # 4. Enforce minimum seabed clearance (1.8 m safety buffer)
        min_z = min(z_A, z_B, z_C)
        clearance_dh = 0.0
        if min_z < 1.8:
            clearance_dh = (1.8 - min_z) * 0.5

        total_dh = max(-2.5, min(4.5, slope_dh + clearance_dh))

        # 5. Update hydrodynamic pitch response (first-order inertial filter)
        dh_rate = (total_dh - self.prev_dh) / 0.1
        self.prev_dh = total_dh

        target_pitch = (
            self.imu_theta
            + math.atan(0.35 * K_AB + 0.20 * K_BC)
            + 0.04 * total_dh
            + 0.015 * dh_rate
        )
        target_pitch += random.gauss(0.0, 0.003)  # Minor hydrodynamic vibration (~0.15 deg)

        # Clamp and smooth hull pitch within [-12 deg, +12 deg]
        self.theta = max(
            math.radians(-12.0),
            min(math.radians(12.0), 0.80 * self.theta + 0.20 * target_pitch)
        )
        pitch_deg = math.degrees(self.theta)

        # Publish telemetry and height correction
        self.pub_pitch_deg.publish(Float64(data=float(pitch_deg)))
        self.get_logger().info(
            f'{scenario_str} | K_AB={K_AB:.2f}, K_BC={K_BC:.2f} | Pitch={pitch_deg:+.2f}deg | dH={total_dh:+.2f}m'
        )

        cmd_msg = Float64()
        cmd_msg.data = float(total_dh)
        self.pub_height_cmd.publish(cmd_msg)


def main(args=None):
    rclpy.init(args=args)
    node = TerrainFollower()
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
