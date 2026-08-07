#!/usr/bin/env python3
import rclpy
import random
from std_msgs.msg import Float64, Bool
from geometry_msgs.msg import PointStamped

class MotorPlugin:
    def init(self, webots_node, properties):
        self.robot = webots_node.robot
        self.robot_node = self.robot.getSelf() 
        
        self.motor_names = ['joint_thruster_1', 'joint_thruster_2', 'joint_thruster_3', 'joint_thruster_7', 'joint_thruster_5', 'joint_thruster_6']
        self.current_velocities = {name: 0.0 for name in self.motor_names}

        if not rclpy.ok(): rclpy.init(args=None)
        self.node = rclpy.create_node('rov_thruster_controller')

        self.node.declare_parameter('enable_poseidon', False) # Mặc định tắt (Nước lặng)
        self.node.declare_parameter('tide_intensity', 20.0)   # Cường độ hải lưu
        self.node.declare_parameter('turb_intensity', 3.0)    # Cường độ sóng dăm

        for t_name in self.motor_names:
            motor = self.robot.getDevice(t_name)
            if motor is not None:
                motor.setPosition(float('inf'))
                motor.setVelocity(0.0)
                self.node.create_subscription(Float64, f'/{t_name}/cmd_vel', lambda msg, name=t_name: self.cmd_callback(msg, name), 10)

        # Cấu hình Camera 
        self.camera = self.robot.getDevice('camera')
        self.camera_motor = self.robot.getDevice('cam_yaw_motor')
        self.cam_angle = 0.0 
        if self.camera is not None: self.node.create_subscription(Bool, '/camera/cmd_switch', self.camera_callback, 10)
        if self.camera_motor is not None: self.node.create_subscription(Float64, '/camera/cmd_rot', self.camera_rot_callback, 10)

        # Kênh phát GPS Tự Chế (Cấp God Mode)
        self.depth_pub = self.node.create_publisher(PointStamped, '/sensor/depth', 10)
        
        self.is_poseidon_on = False
        self.tide_intensity = 20.0
        self.node.create_subscription(Bool, '/poseidon/enable', self.poseidon_on_cb, 10)
        self.node.create_subscription(Float64, '/poseidon/intensity', self.poseidon_int_cb, 10)

    def poseidon_on_cb(self, msg):
        self.is_poseidon_on = msg.data

    def poseidon_int_cb(self, msg):
        self.tide_intensity = msg.data

    def cmd_callback(self, msg, motor_name):
        self.current_velocities[motor_name] = msg.data
        motor = self.robot.getDevice(motor_name)
        if motor is not None: motor.setVelocity(msg.data)
        self.node.get_logger().info(f"🤖 [WEBOTS] Động cơ {motor_name} đã nhận lệnh: {msg.data}")    

    def camera_callback(self, msg):
        if self.camera is not None:
            if msg.data: self.camera.enable(32)
            else: self.camera.disable()

    def camera_rot_callback(self, msg):
        if msg.data != 0.0 and self.camera_motor is not None:
            self.cam_angle -= msg.data * 0.05
            self.camera_motor.setPosition(self.cam_angle)

    def step(self):
        if hasattr(self, 'node') and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0)
            
        if self.robot_node is not None:
            # 1. Đội Lặn Nổi (Trục Z)
            vel_5 = self.current_velocities.get('joint_thruster_5', 0.0)
            vel_6 = self.current_velocities.get('joint_thruster_6', 0.0)
            lan_noi = (vel_5 + vel_6) / 2.0
            
            # Nhân 500.0 để bù lực nổi Fluid
            self.robot_node.addForce([0.0, 0.0, lan_noi * 20.0], True)

            # 2. Đội Vector 45 Độ Thực Thụ
            thrust_factor = 25.0
            t7 = self.current_velocities.get('joint_thruster_7', 0.0) * thrust_factor
            t3 = self.current_velocities.get('joint_thruster_3', 0.0) * thrust_factor
            t2 = self.current_velocities.get('joint_thruster_2', 0.0) * thrust_factor
            t1 = self.current_velocities.get('joint_thruster_1', 0.0) * thrust_factor

            A = 0.7071 

            self.robot_node.addForceWithOffset([t7 * A, -t7 * A, 0.0], [0.2, 0.2, 0.0], True)   # Trước Trái
            self.robot_node.addForceWithOffset([t3 * A, t3 * A, 0.0], [-0.2, 0.2, 0.0], True)   # Sau Trái
            self.robot_node.addForceWithOffset([t2 * A, t2 * A, 0.0], [0.2, -0.2, 0.0], True)   # Trước Phải
            self.robot_node.addForceWithOffset([t1 * A, -t1 * A, 0.0], [-0.2, -0.2, 0.0], True) # Sau Phải

            # ====================================================
            # 🌊 POSEIDON NODE: MÔ PHỎNG SÓNG NGẦM & HẢI LƯU ĐỘNG
            # ====================================================
            # SỬ DỤNG BIẾN THỜI GIAN THỰC THAY VÌ ĐỌC THAM SỐ CỐ ĐỊNH
            if self.is_poseidon_on:
                t = self.robot.getTime() 
                tide_int = self.tide_intensity
                # Sóng dăm (Turbulence) tự động tỷ lệ thuận với Hải lưu (Tide)
                turb_int = tide_int * 0.15 
                
                # Hải lưu (Sóng lớn)
                tide_x = math.sin(t / 4.0) * tide_int
                tide_y = math.cos(t / 5.0) * (tide_int * 0.75)
                
                # Nhiễu loạn (Sóng nhỏ)
                turb_x = random.gauss(0, turb_int)
                turb_y = random.gauss(0, turb_int)
                turb_z = random.gauss(0, turb_int + 1.0)
                
                self.robot_node.addForce([tide_x + turb_x, tide_y + turb_y, turb_z], True)
                
                xoay_ngam = math.sin(t / 2.0) * random.gauss(0, turb_int * 0.5)
                self.robot_node.addTorque([xoay_ngam, xoay_ngam, xoay_ngam], True)
            # ====================================================
            # ====================================================

            # --- ĐOẠN ĐỌC TỌA ĐỘ GPS (GOD MODE) ---
            pos = self.robot_node.getPosition()
            
            msg = PointStamped()
            msg.header.frame_id = 'map'
       
            # Tạm thời ta lấy cả 3 trục ném lên mạng để theo dõi
            msg.point.x = pos[0]
            msg.point.y = pos[1]
            msg.point.z = pos[2] 
            
            self.depth_pub.publish(msg)
