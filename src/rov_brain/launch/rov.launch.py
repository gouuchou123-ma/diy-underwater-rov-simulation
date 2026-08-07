from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import AnyLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_controller import WebotsController 
import os

def generate_launch_description():
    # 1. QUYỀN LỰC TỐI CAO: Ép mọi Node phải nhìn thấy thư mục chứa file motor_plugin.py
    set_env = SetEnvironmentVariable(
        name='PYTHONPATH',
        value=['/home/nhu/rov_ws/src/rov_brain/src:', os.environ.get('PYTHONPATH', '')]
    )

    urdf_file_path = '/home/nhu/rov_ws/src/asem3/urdf/asem3.urdf'
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    jsp_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    webots_driver = WebotsController(
        robot_name='asem3',
        parameters=[
            {'robot_description': urdf_file_path, 'use_sim_time': True} 
        ]
    )

    brain_node = Node(
        package='rov_brain',
        executable='rov_brain_node',
        output='screen'
    )

    foxglove_bridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(get_package_share_directory('foxglove_bridge'), 'launch', 'foxglove_bridge_launch.xml')
        )
    )

    tf_node = Node(
        package='rov_brain',
        executable='tf_broadcaster.py', # Thêm .py nếu CMake của bạn giữ nguyên đuôi
        output='screen'
    )
    
    return LaunchDescription([
        set_env, # Đã kích hoạt lệnh ép buộc
        rsp_node,
        jsp_node,
        webots_driver,
        brain_node,
        foxglove_bridge,
        tf_node,
    ])


