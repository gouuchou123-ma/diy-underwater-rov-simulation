import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch_ros.actions import Node
from webots_ros2_driver.webots_controller import WebotsController


def generate_launch_description():
    # 1. Add motor_plugin.py directory to PYTHONPATH
    plugin_dir = os.path.expanduser('~/rov_ws/src/rov_brain/src')
    set_env = SetEnvironmentVariable(
        name='PYTHONPATH',
        value=[plugin_dir + ':', os.environ.get('PYTHONPATH', '')]
    )

    urdf_file_path = os.path.expanduser('~/rov_ws/src/asem3/urdf/asem3.urdf')
    with open(urdf_file_path, 'r') as infp:
        robot_desc = infp.read()

    # 2. URDF Publishers & Webots Driver
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
        parameters=[{'robot_description': robot_desc, 'use_sim_time': True}]
    )

    webots_driver = WebotsController(
        robot_name='asem3',
        parameters=[{'robot_description': urdf_file_path, 'use_sim_time': True}]
    )

    # 3. Foxglove Studio WebSocket Bridge (Port 8765)
    foxglove_bridge = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('foxglove_bridge'),
                'launch',
                'foxglove_bridge_launch.xml'
            )
        )
    )

    # 4. Gamepad & Shore Teleoperation Nodes
    joy_node = Node(
        package='joy',
        executable='joy_node',
        output='screen'
    )

    teleop_node = Node(
        package='rov_brain',
        executable='rov_teleop.py',
        output='screen'
    )

    # 5. Acoustic Bridge & LAN/Acoustic Multiplexer Nodes
    buoy_node = Node(
        package='rov_brain',
        executable='buoy_bridge.py',
        output='screen'
    )

    mux_node = Node(
        package='rov_brain',
        executable='rov_bridge.py',
        output='screen'
    )

    # 6. Autonomous Terrain-Following, Depth PID & TF Broadcaster Nodes
    terrain_node = Node(
        package='rov_brain',
        executable='terrain_following.py',
        output='screen'
    )

    pid_node = Node(
        package='rov_brain',
        executable='depth_pid.py',
        output='screen'
    )

    tf_node = Node(
        package='rov_brain',
        executable='tf_broadcaster.py',
        output='screen'
    )

    return LaunchDescription([
        set_env,
        rsp_node,
        jsp_node,
        webots_driver,
        foxglove_bridge,
        joy_node,
        teleop_node,
        buoy_node,
        mux_node,
        terrain_node,
        pid_node,
        tf_node
    ])
