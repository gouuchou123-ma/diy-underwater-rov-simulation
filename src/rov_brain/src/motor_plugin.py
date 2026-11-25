#!/usr/bin/env python3
import math
import random
import rclpy
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Range, Imu


class MotorPlugin:
    def init(self, webots_node, properties):
        self.robot = webots_node.robot
        self.robot_node = self.robot.getSelf()
        self.timestep = int(self.robot.getBasicTimeStep())

        self.is_poseidon_on = False
        self.tide_intensity = 20.0

        self.motor_names = [
            'joint_thruster_1', 'joint_thruster_2', 'joint_thruster_3',
            'joint_thruster_7', 'joint_thruster_5', 'joint_thruster_6'
        ]
        self.current_velocities = {name: 0.0 for name in self.motor_names}

        # State variables for first-order Gauss-Markov sonar bias drift
        self.sonar_bias = {
            'sonar_front_link': 0.015,
            'sonar_bottom_link': -0.010,
            'sonar_rear_link': 0.020
        }

        if not rclpy.ok():
            rclpy.init(args=None)

        self.node = rclpy.create_node('rov_thruster_controller')
        self.node.declare_parameter('enable_poseidon', False)
        self.node.declare_parameter('tide_intensity', 20.0)
        self.node.declare_parameter('turb_intensity', 3.0)

        # Initialize 3 single-beam sonar sensors
        self.sonar_front = self.robot.getDevice('sonar_front')
        if self.sonar_front:
            self.sonar_front.enable(self.timestep)

        self.sonar_bottom = self.robot.getDevice('sonar_bottom')
        if self.sonar_bottom:
            self.sonar_bottom.enable(self.timestep)

        self.sonar_rear = self.robot.getDevice('sonar_rear')
        if self.sonar_rear:
            self.sonar_rear.enable(self.timestep)

        # Initialize IMU device if available on the robot model
        self.imu_device = self.robot.getDevice('inertial_unit')
        if self.imu_device is None:
            self.imu_device = self.robot.getDevice('imu')
        if self.imu_device:
            self.imu_device.enable(self.timestep)

        # Initialize thruster actuators and command subscriptions
        for t_name in self.motor_names:
            motor = self.robot.getDevice(t_name)
            if motor is not None:
                self.node.get_logger().info(f'[MOTOR] Initialized thruster: {t_name}')
                motor.setPosition(float('inf'))
                motor.setVelocity(0.0)
                self.node.create_subscription(
                    Float64,
                    f'/{t_name}/cmd_vel',
                    lambda msg, name=t_name: self.cmd_callback(msg, name),
                    10
                )
            else:
                self.node.get_logger().error(f'[MOTOR] Thruster device not found: {t_name}')

        # Initialize camera and yaw gimbal motor
        self.camera = self.robot.getDevice('camera')
        self.camera_motor = self.robot.getDevice('cam_yaw_motor')
        self.cam_angle = 0.0

        if self.camera is not None:
            self.node.create_subscription(Bool, '/camera/cmd_switch', self.camera_callback, 10)
        if self.camera_motor is not None:
            self.node.create_subscription(Float64, '/camera/cmd_rot', self.camera_rot_callback, 10)

        # Depth and IMU publishers
        self.depth_pub = self.node.create_publisher(PointStamped, '/sensor/depth', 10)
        self.imu_pub = self.node.create_publisher(Imu, '/imu', 10)

        # Sonar publishers
        self.front_sonar_pub = self.node.create_publisher(Range, '/sonar/front', 10)
        self.bottom_sonar_pub = self.node.create_publisher(Range, '/sonar/bottom', 10)
        self.rear_sonar_pub = self.node.create_publisher(Range, '/sonar/rear', 10)

        # Poseidon disturbance control subscriptions
        self.node.create_subscription(Bool, '/poseidon/enable', self.poseidon_on_cb, 10)
        self.node.create_subscription(Float64, '/poseidon/intensity', self.poseidon_int_cb, 10)

    def poseidon_on_cb(self, msg):
        self.is_poseidon_on = msg.data

    def poseidon_int_cb(self, msg):
        self.tide_intensity = msg.data

    def cmd_callback(self, msg, motor_name):
        self.current_velocities[motor_name] = msg.data
        motor = self.robot.getDevice(motor_name)
        if motor is not None:
            motor.setVelocity(msg.data)

    def camera_callback(self, msg):
        if self.camera is not None:
            if msg.data:
                self.camera.enable(32)
            else:
                self.camera.disable()

    def camera_rot_callback(self, msg):
        if msg.data != 0.0 and self.camera_motor is not None:
            self.cam_angle -= msg.data * 0.05
            self.camera_motor.setPosition(self.cam_angle)

    def step(self):
        if hasattr(self, 'node') and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0)

        if self.robot_node is not None:
            # 1. Vertical heave thrust (Z-axis)
            vel_5 = self.current_velocities.get('joint_thruster_5', 0.0)
            vel_6 = self.current_velocities.get('joint_thruster_6', 0.0)
            heave_vel = (vel_5 + vel_6) / 2.0
            self.robot_node.addForce([0.0, 0.0, heave_vel * 3.5], True)

             # 2. Decoupled 45-degree vectored thrust (Surge, Sway, pure Yaw via angular velocity)
            thrust_factor = 4.0
            t7 = self.current_velocities.get('joint_thruster_7', 0.0) * thrust_factor
            t3 = self.current_velocities.get('joint_thruster_3', 0.0) * thrust_factor
            t2 = self.current_velocities.get('joint_thruster_2', 0.0) * thrust_factor
            t1 = self.current_velocities.get('joint_thruster_1', 0.0) * thrust_factor

             # Workaround for Webots R2025a Python bug (#6881: addTorque calls addForce)
             # Integrate pure yaw torque into world Z angular velocity (vel[5])
            yaw_torque = (t7 + t3 - t2 - t1) * 0.0015
            if abs(yaw_torque) > 1e-4:
                 dt = self.timestep / 1000.0
                 vel = list(self.robot_node.getVelocity())
                 vel[5] = max(-1.5, min(1.5, vel[5] + (yaw_torque / 0.25) * dt))
                 self.robot_node.setVelocity(vel)

            A = 0.7071
            surge_force = (t7 + t3 + t2 + t1) * A
            sway_force = (t7 - t3 - t2 + t1) * A
            self.robot_node.addForce([surge_force, sway_force, 0.0], True)

            # Apply hydrodynamic pitch torque in local frame 
            surge_net = (t7 + t3 + t2 + t1) / 4.0
            heave_diff = vel_5 - vel_6
            pitch_torque = (surge_net * 0.008) + (heave_diff * 0.12)
            self.robot_node.addTorque([0.0, pitch_torque, 0.0], True)
            self.robot_node.addTorque([0.0, 0.0, yaw_torque], False)

            # 3. Poseidon environmental disturbance model (tidal current + turbulence)
            if getattr(self, 'is_poseidon_on', False):
                t = self.robot.getTime()
                tide_int = self.tide_intensity
                turb_int = tide_int * 0.15

                tide_x = math.sin(t / 4.0) * tide_int
                tide_y = math.cos(t / 5.0) * (tide_int * 0.75)

                turb_x = random.gauss(0, turb_int)
                turb_y = random.gauss(0, turb_int)
                turb_z = random.gauss(0, turb_int + 1.0)
                self.robot_node.addForce([tide_x + turb_x, tide_y + turb_y, turb_z], True)

                turb_torque = math.sin(t / 2.0) * random.gauss(0, turb_int * 0.5)
                self.robot_node.addTorque([turb_torque, turb_torque, turb_torque], True)

            # 4. Publish depth (GPS position) and IMU orientation
            now_msg = self.node.get_clock().now().to_msg()
            if hasattr(self, 'depth_pub'):
                pos = self.robot_node.getPosition()
                msg = PointStamped()
                msg.header.stamp = now_msg
                msg.header.frame_id = 'map'
                msg.point.x = pos[0]
                msg.point.y = pos[1]
                msg.point.z = pos[2]
                self.depth_pub.publish(msg)

            if hasattr(self, 'imu_pub'):
                # Extract hull pitch angle from Webots 3x3 rotation matrix element R[2][0]
                rot = self.robot_node.getOrientation()
                sin_pitch = max(-1.0, min(1.0, -rot[6]))
                pitch_angle = math.asin(sin_pitch)

                imu_msg = Imu()
                imu_msg.header.stamp = now_msg
                imu_msg.header.frame_id = 'base_link'
                imu_msg.orientation.x = 0.0
                imu_msg.orientation.y = math.sin(pitch_angle / 2.0)
                imu_msg.orientation.z = 0.0
                imu_msg.orientation.w = math.cos(pitch_angle / 2.0)
                self.imu_pub.publish(imu_msg)

            # 5. Publish single-beam sonar ranges with heteroscedastic & Gauss-Markov noise
            if hasattr(self, 'front_sonar_pub'):
                dt = self.timestep / 1000.0
                tau_bias = 60.0  # Bias drift time constant (s)
                sigma_0 = 0.01   # Baseline electronic noise (m)
                eta = 0.015      # Distance-proportional noise factor (1.5% of range)

                def publish_sonar(device, publisher, frame_id):
                    if device is not None:
                        true_d = device.getValue()

                        # Update first-order Gauss-Markov bias
                        prev_b = self.sonar_bias.get(frame_id, 0.0)
                        new_b = (1.0 - dt / tau_bias) * prev_b + random.gauss(0.0, 0.0005)
                        self.sonar_bias[frame_id] = new_b

                        # Apply distance-dependent Gaussian noise
                        if true_d < 49.5:
                            sigma_d = sigma_0 + eta * true_d
                            noise = random.gauss(0.0, sigma_d)
                            measured_d = max(0.1, min(50.0, true_d + new_b + noise))
                        else:
                            measured_d = 50.0

                        msg = Range()
                        msg.header.stamp = now_msg
                        msg.header.frame_id = frame_id
                        msg.radiation_type = Range.ULTRASOUND
                        msg.field_of_view = 0.52  # 30 deg beam aperture
                        msg.min_range = 0.1
                        msg.max_range = 50.0
                        msg.range = float(measured_d)
                        publisher.publish(msg)

                publish_sonar(self.sonar_front, self.front_sonar_pub, 'sonar_front_link')
                publish_sonar(self.sonar_bottom, self.bottom_sonar_pub, 'sonar_bottom_link')
                publish_sonar(self.sonar_rear, self.rear_sonar_pub, 'sonar_rear_link')
