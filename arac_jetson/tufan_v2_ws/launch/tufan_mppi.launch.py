import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg_share = get_package_share_directory('tufan_v2_ws')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    xacro_file = os.path.join(pkg_share, 'urdf', 'robot.xacro')
    nav2_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    ekf_params_file = os.path.join(pkg_share, 'config', 'ekf.yaml')
    rviz_config_file = os.path.join(pkg_share, 'config', 'nav2_view.rviz')
    bno055_params_file = os.path.join(
        get_package_share_directory('bno055'), 'config', 'bno055_params.yaml'
    )

    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    use_sim_time = False  # Gercek donanim, simulasyon saati kullanilmiyor

    # --- Launch argumanlari (donanim baglantisina gore degistirilebilir) ---
    lidar_serial_port = LaunchConfiguration('lidar_serial_port')
    declare_lidar_serial_port_cmd = DeclareLaunchArgument(
        'lidar_serial_port', default_value='/dev/ttyUSB0',
        description='SLLIDAR S3 seri port'
    )

    tabela_camera_index = LaunchConfiguration('tabela_camera_index')
    declare_tabela_camera_index_cmd = DeclareLaunchArgument(
        'tabela_camera_index', default_value='0',
        description='Tabela (trafik levhasi) algilama kamerasinin /dev/videoN indeksi'
    )
    tabela_video_target_ip = LaunchConfiguration('tabela_video_target_ip')
    declare_tabela_video_target_ip_cmd = DeclareLaunchArgument(
        'tabela_video_target_ip', default_value='192.168.1.20',
        description='Isaretlenmis tabela goruntusunun UDP ile gonderilecegi IP'
    )
    tabela_video_target_port = LaunchConfiguration('tabela_video_target_port')
    declare_tabela_video_target_port_cmd = DeclareLaunchArgument(
        'tabela_video_target_port', default_value='5001',
        description='Tabela goruntu UDP portu (turret ile cakismasin diye farkli)'
    )

    # 1. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': use_sim_time}],
    )

    # 1b. Teker joint yayinlayici: left/right_wheel_joint (continuous) icin
    #     joint_states hic gelmiyordu, bu yuzden robot_state_publisher bu
    #     tekerlerin TF'ini uretemiyor ve RViz'de RobotModel "no transform"
    #     hatasi veriyordu (LIDAR/IMU fixed joint oldugu icin etkilenmiyordu).
    teker_joint_node = Node(
        package='tufan_v2_ws',
        executable='teker_joint_yayinlayici.py',
        name='teker_joint_yayinlayici',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # 2. IMU (BNO055) - /bno055/imu olarak yayinlar, /imu/data'ya remap ediyoruz
    imu_node = Node(
        package='bno055',
        executable='bno055',
        name='bno055',
        parameters=[bno055_params_file, {
            'use_sim_time': use_sim_time,
            'frame_id': 'imu_link',  # URDF'teki imu_link ile eslesmeli (yoksa TF bulunamaz)
        }],
        remappings=[('bno055/imu', 'imu/data')],
    )

    # 4. LIDAR (SLLIDAR S3) - ciktisi 'scan_raw'a remap edildi, asil '/scan'i
    #    scan_front_filter uretiyor (bkz. asagida) - govdedeki 3D baski
    #    kapatmasi nedeniyle yan/arka acilar surekli sahte yakin-mesafe uretiyordu.
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': lidar_serial_port,
            'serial_baudrate': 1000000,
            'frame_id': 'lidar_link',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'DenseBoost',
            'use_sim_time': use_sim_time,
        }],
        remappings=[('scan', 'scan_raw')],
        output='screen',
    )

    # 4b. Scan on-koni filtresi: LIDAR govdesindeki 3D baski sadece onu acik
    #     birakiyor (yan/arka kapali) - bu dugum sadece dogrulanmis acik koniyi
    #     (ham scan acisi |aci|>=140 derece, TF'e gore gercek on) birakip
    #     gerisini 'inf' yapar, boylece kapali acilardaki sahte yakin-mesafe
    #     yansimalari costmap'e hic girmez (temizlenemeyen "iz" sorununun kaynagi).
    scan_front_filter_node = Node(
        package='tufan_v2_ws',
        executable='scan_front_filter.py',
        name='scan_front_filter',
        parameters=[{'acik_esik_derece': 140.0, 'use_sim_time': use_sim_time}],
    )

    # 4b2. Scan-matcher icin GENIS ACILI filtre (OTONOM ODOMETRI DUZELTMESI):
    #      laser_scan_matcher, costmap'in dar (~80 derece) on-koni filtresini
    #      (yukaridaki scan_front_filter_node) paylasiyordu - ICP scan-matching
    #      360 derecenin sadece ~%21'i ile calisinca, ozellikle donuslerde
    #      referans noktalari hizla goruş disina cikip biriken pozisyon hatasina
    #      (surus sirasinda kayma) yol aciyordu. Bu dugum, aci kisitlamasi
    #      OLMADAN, sadece govdeye/baskiya GERCEKTEN bitisik (<0.20m) sahte
    #      yansimalari temizler - costmap'in kendi filtresine (guvenlik
    #      davranisi) HIC DOKUNULMADI, sadece scan-matcher'a ayri, cok daha
    #      zengin bir veri kaynagi verildi (bkz. scan_matcher_filter.py).
    scan_matcher_filter_node = Node(
        package='tufan_v2_ws',
        executable='scan_matcher_filter.py',
        name='scan_matcher_filter',
        parameters=[{'near_range_m': 0.20, 'use_sim_time': use_sim_time}],
    )

    # NOT: rf2o lazer odometrisi (+ 3 farkli gurultu azaltma denemesi) arac
    # DURURKEN bile RViz'de kayma/drift uretmeye devam etti - kesin olarak
    # kaldirildi. Gercek teker enkoderi/GPS de yok. Bunun yerine once acik
    # dongu (open-loop) tahmin denendi (komut edilen hizi entegre etmek) -
    # calisti ama YAKLASIK bir tahmindi (gercek teker kaymasi hesaba
    # katilmadigi icin). acik_dongu_odom.py hala kaynak kodda duruyor
    # (yedek/karsilastirma icin), ama artik varsayilan olarak calismiyor.
    #
    # 4c. laser_scan_matcher (CSM/ICP tabanli scan matching, rf2o'dan FARKLI
    # bir algoritma): bagimsiz testte rf2o'dan cok daha stabil cikti - 10sn
    # durgunken max 1.1cm sapma / 0.14 derece yaw sapmasi, ve gercek kisa bir
    # surus sirasinda dogru mesafe yakalayip sonrasinda ~58sn boyunca 2mm
    # gurultu bandinda sabit kaldi. Şu an ana odometri kaynagi budur.
    laser_scan_matcher_node = Node(
        package='ros2_laser_scan_matcher',
        executable='laser_scan_matcher',
        name='laser_scan_matcher',
        output='screen',
        parameters=[{
            'publish_odom': '/odom_lsm',
            'publish_tf': False,
            'base_frame': 'base_link',
            'odom_frame': 'odom_lsm_internal',
            'laser_frame': 'lidar_link',
            'use_sim_time': use_sim_time,
        }],
        # GERI ALINDI (canli testte dogrulandi): scan_matcher_filtered (tam
        # 360 derece, sadece <0.20m temizlenmis) ICP'yi TAMAMEN BOZDU ("ICP
        # failed for some reason", ~150 eslesme, /odom_lsm 15s+ hic yayin
        # yapmadi) - dar on-koni ('scan') ile calisan PL-ICP, cok daha genis
        # ve heterojen bir nokta bulutuyla eslesme bulamadi. bkz.
        # scan_matcher_filter_node yorumundaki devam eden arastirma notu.
    )

    # 4d. rf2o_laser_odometry: bu projede once pozisyon kaynagi olarak
    #     denenip reddedilmisti (durgunken bile RViz'de kayiyordu). Simdi
    #     SADECE lineer hiz (twist.linear.x) icin tekrar etkinlestirildi -
    #     laser_scan_matcher.cpp'nin hiz=pozisyon_farki/wall-clock_dt
    #     hesabinin Jetson'da CPU paylasimi altinda urettigi FIZIKSEL
    #     IMKANSIZ hiz sicramalarina (canli testte: 2-3 m/s, gercek max
    #     0.4 m/s) karsi arastirma sonrasi bulunan cozum: rf2o kendi ic
    #     zaman farkini LIDAR'in donanim zaman damgasindan (wall-clock
    #     DEGIL) alir ve son 4 orneğin ortalamasiyla zaten yumusatilmis
    #     cikar (bkz. konum_birlestirici.py dosya basi notu). publish_tf
    #     KAPALI - konum_birlestirici.py zaten odom->base_footprint TF'ini
    #     kendi yayinliyor, iki yayinci ayni TF'te CAKISMASIN diye.
    rf2o_node = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic': 'scan',
            'odom_topic': '/odom_rf2o',
            'publish_tf': False,
            'base_frame_id': 'base_footprint',
            'odom_frame_id': 'odom_rf2o_internal',
            'init_pose_from_topic': '',
            'freq': 10.0,
            'use_sim_time': use_sim_time,
        }],
    )

    # 5. Konum birlestirici: robot_localization/EKF denendi ama ilginc bir
    #    sorun cikti (EKF SUREKLI KAYDI, atlandi). Sonra /odom_lsm (ICP)
    #    konum icin kullanildi ama canli testte "donus iyi, ileri-geri hic
    #    yok" seklinde basarisiz oldu (dar-FOV aperture problem, bkz.
    #    konum_birlestirici.py dosya basi notu). ARTIK konum(x,y) ICP
    #    DEGIL, rf2o hizi + IMU yonu zaman icinde entegre edilerek
    #    (dead-reckoning) hesaplaniyor; /odom_lsm HIC KULLANILMIYOR (yine de
    #    referans/karsilastirma icin laser_scan_matcher_node calismaya
    #    devam ediyor).
    konum_birlestirici_node = Node(
        package='tufan_v2_ws',
        executable='konum_birlestirici.py',
        name='konum_birlestirici',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # 7. Nav2 navigasyon yigini (controller/planner/smoother/behavior/bt_navigator/
    #    velocity_smoother + lifecycle_manager). Harita/AMCL YOK - haritasiz calisiyoruz.
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': str(use_sim_time),
            'params_file': nav2_params_file,
            'autostart': 'true',

        }.items(),
    )

    # 7b. OTONOM KONTROL - Otonom hedef yoneticisi: Nav2'nin rolling-window
    #     costmap'i uzak bir hedefe TEK SEFERDE yol uretemedigi icin (harita
    #     yok), bu node uzak hedefi costmap penceresi icinde kalan ara
    #     "bacaklara" bolup NavigateToPose action'ini tekrar tekrar cagirir
    #     (bkz. goal_manager_node.py docstring). Simulasyonda (ros2_ws/
    #     tufan_simulation, Gazebo'da dogrulandi) test edilen otonom surus
    #     mantiginin bu araca tasinan asil parcasi budur.
    #     max_leg_distance/replan_trigger_distance, bu aracin global_costmap
    #     boyutuna (15x15m, bkz. config/nav2_params.yaml) gore olceklendi
    #     (bkz. goal_manager_node.py'deki "GERCEK ARAC UYARLAMASI" notu).
    #     respawn=True: goal_manager_watchdog.py nadir bir executor
    #     kilitlenmesinde bu process'i SIGTERM ile sonlandirir, respawn
    #     onu otomatik yeniden baslatir.
    goal_manager_node = Node(
        package='tufan_v2_ws',
        executable='goal_manager_node.py',
        name='goal_manager_node',
        output='screen',
        respawn=True,
        respawn_delay=1.0,
        parameters=[{
            'use_sim_time': use_sim_time,
            'max_leg_distance': 6.0,
            'replan_trigger_distance': 2.0,
            'goal_reached_tolerance': 0.55,
            'max_leg_retries': 8,
        }],
    )

    # 7c. goal_manager_node'un canlilik bekcisi (yukaridaki not, bkz.
    #     goal_manager_watchdog.py docstring).
    goal_manager_watchdog_node = Node(
        package='tufan_v2_ws',
        executable='goal_manager_watchdog.py',
        name='goal_manager_watchdog',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'timeout_sec': 6.0}],
    )

    # 7d. OTONOM KONTROL - Nav2/MPPI'nin urettigi /cmd_vel'i motorlara
    #     gondermeden once LiDAR ile bagimsiz bir "yol gercekten acik mi"
    #     kontrolunden gecirip /cmd_vel_gated uretir; sabit/hareketli engel
    #     ayrimi yapar (bkz. nav2_cmd_vel_gate.py docstring - bu araca ozgu
    #     LiDAR aci kalibrasyonu dahil). surus_koprusu.py OTONOM modda
    #     artik bunu dinler (asagida).
    nav2_cmd_vel_gate_node = Node(
        package='tufan_v2_ws',
        executable='nav2_cmd_vel_gate.py',
        name='nav2_cmd_vel_gate',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # 8. RViz2 - path/costmap gorsellestirme + "2D Goal Pose" ile hedef gonderme
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
    )

    # 9. Surus koprusu: SADECE OTONOM modda /cmd_vel_gated'i (Nav2 ciktisinin
    #    nav2_cmd_vel_gate.py'den gecmis hali) dinleyip /palet_hizlari
    #    (Float32MultiArray, -255..255) formatina cevirir. MANUEL modda hic
    #    yayin yapmaz - o konuyu dogrudan yer istasyonu (tufan_yer_istasyonu)
    #    yonetir, iki yayinci ayni konuda yarismasin diye.
    surus_koprusu_node = Node(
        package='tufan_v2_ws',
        executable='surus_koprusu.py',
        name='surus_koprusu',
        output='screen',
    )

    # 10. Arduino motor surucusu - /palet_hizlari'i dinleyip seri port uzerinden
    #     gercek motorlara PPM komutu gonderir. BU DOSYAYA DOKUNULMADI.
    #     Dosyada shebang olmadigi icin Node yerine ExecuteProcess ile python3'u
    #     acikca cagiriyoruz - dosya icerigine hic dokunmadan calistirmanin yolu bu.
    arduino_script_path = os.path.join(
        get_package_prefix('tufan_v2_ws'), 'lib', 'tufan_v2_ws', 'arduino_motor_kontrol.py'
    )
    motor_driver_node = ExecuteProcess(
        cmd=['python3', arduino_script_path],
        name='arduino_motor_kontrol_uydusu',
        output='screen',
    )

    # 11. Tabela (trafik levhasi) algilama - ayri bir kameradan surekli YOLO
    #     tespiti yapar, /tabela_tespit'e yayinlar, isaretli goruntuyu UDP ile gonderir.
    tabela_model_path = os.path.join(pkg_share, 'models', 'Tabela_ana.pt')
    tabela_node = Node(
        package='tufan_v2_ws',
        executable='tabela_node.py',
        name='tabela_node',
        output='screen',
        parameters=[{
            'camera_index': tabela_camera_index,
            'model_path': tabela_model_path,
            'video_target_ip': tabela_video_target_ip,
            'video_target_port': tabela_video_target_port,
        }],
    )

    return LaunchDescription([
        declare_lidar_serial_port_cmd,
        declare_tabela_camera_index_cmd,
        declare_tabela_video_target_ip_cmd,
        declare_tabela_video_target_port_cmd,
        robot_state_publisher,
        teker_joint_node,
        imu_node,
        lidar_node,
        scan_front_filter_node,
        laser_scan_matcher_node,
        rf2o_node,
        konum_birlestirici_node,
        navigation_launch,
        goal_manager_node,
        goal_manager_watchdog_node,
        nav2_cmd_vel_gate_node,
        rviz_node,
        surus_koprusu_node,
        motor_driver_node,
        tabela_node,
    ])
