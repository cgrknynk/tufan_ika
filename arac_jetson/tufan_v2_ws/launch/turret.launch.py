import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('tufan_v2_ws')
    # KESIN COZUM (2026-09-01): hedefv2.pt (PyTorch, CPU-agir) yerine
    # hedefv3-seg.engine (TensorRT) - bkz. tufan_mppi.launch.py'deki tabela
    # icin AYNI tarihli not (CPU asiri yuk -> rf2o gecikmesi -> odometri
    # kaymasi + Nav2 lifecycle bond timeout). imgsz=800 bu engine icin
    # DOGRULANDI (izole testte basarili, imgsz=640 HATA verdi) - asagidaki
    # declare_model_imgsz_cmd default'u da 800'e cekildi.
    varsayilan_model_path = os.path.join(pkg_share, 'models', 'hedefv3-seg.engine')

    camera_index = LaunchConfiguration('camera_index')
    turret_serial_port = LaunchConfiguration('turret_serial_port')
    video_target_ip = LaunchConfiguration('video_target_ip')
    video_target_port = LaunchConfiguration('video_target_port')
    model_path = LaunchConfiguration('model_path')
    model_imgsz = LaunchConfiguration('model_imgsz')

    declare_camera_index_cmd = DeclareLaunchArgument(
        'camera_index', default_value='0',
        description='Turret (hedef takip) kamerasinin /dev/videoN indeksi'
    )
    declare_turret_serial_port_cmd = DeclareLaunchArgument(
        'turret_serial_port', default_value='/dev/ttyACM0',
        description='Turret Arduino seri portu'
    )
    declare_video_target_ip_cmd = DeclareLaunchArgument(
        'video_target_ip', default_value='10.40.64.44',
        description='Islenmis turret goruntusunun UDP ile gonderilecegi IP'
    )
    declare_video_target_port_cmd = DeclareLaunchArgument(
        'video_target_port', default_value='5000',
        description='Turret goruntu UDP portu'
    )
    declare_model_path_cmd = DeclareLaunchArgument(
        'model_path', default_value=varsayilan_model_path,
        description='YOLO model dosyasi (.pt/.engine/.onnx) - farkli bir model icin mutlak yol verilebilir'
    )
    declare_model_imgsz_cmd = DeclareLaunchArgument(
        'model_imgsz', default_value='800',
        description='Model giris boyutu (piksel) - TensorRT (.engine) export edilen modeller icin export sirasindaki boyutla AYNI olmali'
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
            'model_imgsz': model_imgsz,
            'video_target_ip': video_target_ip,
            'video_target_port': video_target_port,
        }],
        # OTOMATIK RESPAWN (2026-09-09): silah kamerasi/taret gorev
        # kritik. Sureç olurse ya da saglik_bekcisi 'ayakta ama
        # sessiz' diye oldururse launch geri getirir (bkz.
        # tufan_mppi.launch.py'deki OTOMATIK RESPAWN notu).
        respawn=True,
        respawn_delay=3.0,
    )

    return LaunchDescription([
        declare_camera_index_cmd,
        declare_turret_serial_port_cmd,
        declare_video_target_ip_cmd,
        declare_video_target_port_cmd,
        declare_model_path_cmd,
        declare_model_imgsz_cmd,
        turret_node,
    ])
