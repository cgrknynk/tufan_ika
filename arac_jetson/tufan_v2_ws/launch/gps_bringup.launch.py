"""GPS/RTK (Cube Orange + Here4 GNSS + TUSAGA-Aktif) icin tum bringup'i
tek yerden baslatir. Ana otonomluk launch dosyasindan (tufan_mppi.launch.py)
KASITLI OLARAK AYRI tutuldu - donanim/baglanti her zaman takili/dogrulanmis
olmayabilir, ana sistemi etkilememeli. konum_birlestirici.py GPS topic'leri
hic gelmese BILE saf dead-reckoning ile calismaya devam eder (bkz. o
dosyanin basi).

Ikisini BIRLIKTE calistirmak icin:
    ros2 launch tufan_v2_ws tufan_mppi.launch.py
    ros2 launch tufan_v2_ws gps_bringup.launch.py   # ayri terminal

Icerir:
  1. cube_orange_mavros.launch.py (mavros, Cube Orange'a otomatik port
     tespitiyle baglanir - bkz. o dosyanin basi)
  2. fix_qos_bridge.py (mavros'un BEST_EFFORT /mavros/global_position/
     raw/fix'ini arayuz/GCS'nin bekledigi RELIABLE /fix'e cevirir - TASINDI,
     eskiden workspace disinda elle calistiriliyordu, bkz. o dosyanin basi)
  3. gps_hedef_donusturucu.py (operatorun /gps_hedef'e (lat/lon) yayinladigi
     hedefi konum_birlestirici.py'nin anchor/hizalamasini kullanarak
     /ugv_goal'a (odom cercevesi) cevirir - bkz. o dosyanin basi)

RTCM (TUSAGA-Aktif) enjeksiyonu VE konum_birlestirici.py'nin GPS-tabanli
surunme duzeltmesi burada DEGIL, ayri calisir: ilki arayuz Jetson'unda
(internet olan taraf) elle/ayri yurutulen bir surec (ntrip_rtcm_bridge.py,
bu workspace'in DISINDA), ikincisi ana tufan_mppi.launch.py icindeki
konum_birlestirici_node zaten /mavros/... topic'lerine DOGRUDAN abone
(bu launch dosyasindan BAGIMSIZ calisir, sadece topic'lerin var olmasina
ihtiyac duyar).
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('tufan_v2_ws')

    cube_orange_mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'cube_orange_mavros.launch.py')
        ),
    )

    fix_qos_bridge_node = Node(
        package='tufan_v2_ws',
        executable='fix_qos_bridge.py',
        name='fix_qos_bridge',
        output='screen',
    )

    gps_hedef_donusturucu_node = Node(
        package='tufan_v2_ws',
        executable='gps_hedef_donusturucu.py',
        name='gps_hedef_donusturucu',
        output='screen',
    )

    return LaunchDescription([
        cube_orange_mavros_launch,
        fix_qos_bridge_node,
        gps_hedef_donusturucu_node,
    ])
