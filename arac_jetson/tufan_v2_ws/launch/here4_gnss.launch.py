"""Here4 GNSS (u-blox ZED-F9P tabanli) rover node'unu baslatir.

Bagimsiz/ayri bir launch dosyasi -- ana tufan_mppi.launch.py'ye KASITLI OLARAK
eklenmedi, boylece donanim/baglanti dogrulanana kadar calisan sistemi
etkilemez. Test icin:

    ros2 launch tufan_v2_ws here4_gnss.launch.py

Basariliysa /fix (sensor_msgs/NavSatFix) topic'inde konum gorunur:

    ros2 topic echo /fix
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('tufan_v2_ws'), 'config', 'here4_gnss.yaml')

    return LaunchDescription([
        Node(
            package='ublox_gps',
            executable='ublox_gps_node',
            name='ublox_gps_node',
            output='screen',
            parameters=[config],
        ),
    ])
