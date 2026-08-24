import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('tufan_v2_ws')
    model_path = os.path.join(pkg_share, 'models', 'Target26last.pt')

    camera_index = LaunchConfiguration('camera_index')
    turret_serial_port = LaunchConfiguration('turret_serial_port')
    video_target_ip = LaunchConfiguration('video_target_ip')
    video_target_port = LaunchConfiguration('video_target_port')

    declare_camera_index_cmd = DeclareLaunchArgument(
        'camera_index', default_value='0',
        description='Turret (hedef takip) kamerasinin /dev/videoN indeksi'
    )
    declare_turret_serial_port_cmd = DeclareLaunchArgument(
        'turret_serial_port', default_value='/dev/ttyACM0',
        description='Turret Arduino seri portu'
    )
    declare_video_target_ip_cmd = DeclareLaunchArgument(
        'video_target_ip', default_value='10.40.64.48',
        description='Islenmis turret goruntusunun UDP ile gonderilecegi IP'
    )
    declare_video_target_port_cmd = DeclareLaunchArgument(
        'video_target_port', default_value='5000',
        description='Turret goruntu UDP portu'
    )

    turret_node = Node(
        package='tufan_v2_ws',
        executable='turret_node.py',
        name='turret_node',
        output='screen',
        parameters=[{
            'camera_index': camera_index,
            'serial_port': turret_serial_port,
            'model_path': model_path,
            'video_target_ip': video_target_ip,
            'video_target_port': video_target_port,
        }],
    )

    return LaunchDescription([
        declare_camera_index_cmd,
        declare_turret_serial_port_cmd,
        declare_video_target_ip_cmd,
        declare_video_target_port_cmd,
        turret_node,
    ])
