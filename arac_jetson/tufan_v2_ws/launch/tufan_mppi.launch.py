import os
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def _lidar_portunu_otomatik_bul():
    """KESIN COZUM (2026-09-01, CANLI TESTTE BULUNDU): sabit /dev/ttyUSB0
    kirilgan - USB yeniden-enumerasyonunda LIDAR'in seri adaptoru ttyUSB1'e
    kayinca sllidar_node "Error, unexpected error, code: 80008004" ile
    coktu (arduino_motor_kontrol.py'nin arduino_uno_portu_bul() ve
    tabela_node.py/turret_node.py'nin kamera oto-bulma ile AYNI kok neden -
    bu projede tekrarlayan bir USB kararsizligi deseni). LIDAR'in USB-seri
    cip'i (Silicon Labs CP2102N, VID:PID=10c4:ea60 - CANLI DOGRULANDI,
    `python3 -c "import serial.tools.list_ports as p; ..."` ile) baska
    hicbir takili cihazla (2x Arduino Uno, 2x CubeOrange) CAKISMIYOR, bu
    yuzden VID:PID GUVENILIR bir birincil kimlik. Eslesme yoksa (or.
    kablo cikarilmis) launch varsayilanina (/dev/ttyUSB0) DUSER."""
    # 2026-09-09: ONCE udev SEMBOLU denenir. Bu fonksiyon launch dosyasi
    # AYRISTIRILIRKEN (bir kez, en basta) calisiyor - donen deger sabit bir
    # metin olarak node'a gidiyor. VID:PID taramasi o ANIN gercegini
    # donduruyordu; adaptor sonradan koparsa/kayarsa (9 Eylul'de 6 kez
    # oldu, ttyUSB0<->ttyUSB1) node ESKI porta yaziliyordu. /dev/tufan_lidar
    # ise cekirdek tarafindan ACILMA aninda cozuluyor, yani port kaymasi
    # tamamen ONEMSIZ hale geliyor (udev kurali: arac_jetson/udev/
    # 99-tufan.rules, VID:PID ile eslesir - seri no sarti KALDIRILDI cunku
    # hizli yeniden-numaralandirmada eslesmiyordu).
    import os as _os
    if _os.path.exists('/dev/tufan_lidar'):
        return '/dev/tufan_lidar'

    import serial.tools.list_ports
    LIDAR_VID_PID = (0x10C4, 0xEA60)
    for p in serial.tools.list_ports.comports():
        if (p.vid, p.pid) == LIDAR_VID_PID:
            return p.device
    return '/dev/ttyUSB0'


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
        # KESIN COZUM (2026-09-01): VID:PID ile OTOMATIK bulunur, bkz.
        # _lidar_portunu_otomatik_bul(). Elle farkli bir port zorlamak
        # icin: ros2 launch ... lidar_serial_port:=/dev/ttyUSBX
        'lidar_serial_port', default_value=_lidar_portunu_otomatik_bul(),
        description='SLLIDAR S3 seri port (varsayilan: VID:PID ile otomatik bulunur)'
    )

    # CANLI TESTTE BULUNDU (2026-09-01): kameralar takilinca v4l2-ctl ile
    # dogrulandi - /dev/video1 = C922 Pro Stream Webcam (silah/turret icin),
    # /dev/video2 = C270 HD WEBCAM (on kamera/tabela icin) GERCEK video-
    # capture dugumleri (video3/5/6 ayni fiziksel kameralarin BOS format
    # listeli metadata dugumleri, video4 = IKINCI bir C270, arka kamera
    # icin olabilir - bu mesajda belirtilmedi, dokunulmadi).
    tabela_camera_index = LaunchConfiguration('tabela_camera_index')
    declare_tabela_camera_index_cmd = DeclareLaunchArgument(
        'tabela_camera_index', default_value='2',
        description='Tabela (trafik levhasi) algilama kamerasinin /dev/videoN indeksi (C270)'
    )
    tabela_video_target_ip = LaunchConfiguration('tabela_video_target_ip')
    declare_tabela_video_target_ip_cmd = DeclareLaunchArgument(
        'tabela_video_target_ip', default_value='192.168.1.20',
        description='Isaretlenmis tabela goruntusunun UDP ile gonderilecegi IP'
    )
    # NOT (2026-09-01): kullanici port atamasini TERSINE CEVIRDI - onceki
    # istekte "silah 5000, tabela 5001" denmisti, bu mesajda "tabela 5000,
    # silah 5001" denildi. GUNCEL/SON istek buradaki degerler.
    tabela_video_target_port = LaunchConfiguration('tabela_video_target_port')
    declare_tabela_video_target_port_cmd = DeclareLaunchArgument(
        'tabela_video_target_port', default_value='5000',
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
        # --- OTOMATIK RESPAWN (2026-09-09) ---
        # 9 Eylul'de yasananlar: LIDAR'in USB adaptoru 6 KEZ koptu
        # (dmesg -19/ENODEV), sllidar_node OLU fd'yi tutup SESSIZCE
        # yayin kesti; silah kamerasi cap.read()'te surekli False
        # dondu. Hicbiri COKMEDI - bu yuzden launch de bir sey
        # yapmadi ve her seferinde ELLE yeniden baslatmak gerekti.
        # respawn: sureç OLURSE/OLDURULURSE launch onu geri getirir.
        # 'Ayakta ama sessiz' durumu icin saglik_bekcisi.py sureci
        # OLDURUR, boylece respawn devreye girer (ikisi birlikte
        # calisir - biri olmadan digeri yetersiz).
        respawn=True,
        respawn_delay=3.0,
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
        # 2026-09-01: 220 dereceye genisletme (esik=70) DENENDI, UYARILAN
        # "iz" sorunu CANLI VERIDE DOGRULANDI - /scan_raw'da |aci| 60-90
        # derece arasi HER IKI tarafta da SABIT 0.023-0.060m okuma (govde/
        # montaj yansimasi, 3 ayri ornekte tutarli), 90-100 derece gecis
        # bolgesi (karisik), 100 derece ve sonrasi TEMIZ (gercek ortam,
        # metrelerce). Bu sahte yakin-sabit noktalar rf2o'nun 'scan'
        # girdisini kirletip hiz tahminini SIFIRA yakin bastiriyordu - araç
        # gercekte ilerlerken RViz'de ileri-geri hareketin hic gorunmemesi
        # bu yuzdendi (bkz. konum_birlestirici.py). Esik=105 (150 derece
        # acik koni, 5 derecelik guvenlik payiyla) - 100 derecede kirlenme
        # bitiyor, orijinal 80 dereceden daha genis ama kirli bolgeye
        # girmeyen olculdugu dogrulanmis bir deger.
        #
        # GUNCELLEME (2026-09-02, kullanici istegi): kullanici LIDAR'in
        # fiziksel baskisini/braketini ayarlayip govde-yansima bolgesini
        # KUCULTTU. Filtre GECICI olarak kapatilip /scan_raw 1 derece
        # cozunurlukle YENIDEN OLCULDU (canli, ayarlama sonrasi): kirlilik
        # artik SADECE raw aci ~40 derece (pozitif tarafta 40'ta biter,
        # 41'de temiz/3m+; negatif tarafta -39/-41 civari benzer simetrik
        # sinir) - ONCEKI 100 dereceden COK daha dar. Esik=45 (270 derece
        # acik koni, 5 derecelik AYNI guvenlik payi mantigiyla) - 40
        # derecede kirlenme bitiyor, olculmus TEMIZ sinirin hemen otesinde.
        #
        # IKINCI GUNCELLEME (2026-09-02, ayni gun, kullanici LIDAR acisini
        # TEKRAR duzeltti - yukseklik/montaj icin LIDAR etrafina "citalar"
        # eklendi): TEK bir olcum guvenilir bulunmadi (LIDAR donerken
        # kare-kare gurultu / kismen o an insan/el karismis olabilir), 8
        # ardisik tarama ORTALAMASI alinip HEM gecerli-yakin (<1.0m) HEM
        # gecersiz (sensorun kendi range_min=0.05m/5cm altinda - donanimsal
        # kor nokta, kullanicinin tahmini DOGRULANDI) acilar ayri ayri
        # tespit edildi. Ana arka bant artik SADECE ~28 derece (once ~40
        # dereceydi) - Esik=33 (5 derecelik AYNI guvenlik payi, 294 derece
        # acik koni).
        #
        # BILINEN KALAN SORUN (bu filtreyle COZULEMEZ, FIZIKSEL mudahale
        # gerekir): citalarin bazilari ANA ARKA BANDIN DISINDA, IZOLE
        # noktalarda kalici kirlilik/kor-nokta uretiyor - en onemlisi
        # raw ~184-185 derece (GERCEK ON yonun hemen yaninda!) kalici
        # GECERSIZ (5cm altinda cita var) VE raw ~114 derece kalici
        # GECERLI-YAKIN (~7.7cm, gercek engel gibi isaretlenebilir). Bu
        # TEK NOKTALAR, mevcut simetrik "|aci|>=esik" filtre tasarimiyla
        # (SADECE arkayi tek blok halinde disliyor) KAPSANAMAZ - kullanici
        # bu spesifik citalari FIZIKSEL olarak kaydirmali/inceltmeli.
        # Gecersiz (184-185) rf2o/costmap tarafindan zaten otomatik
        # yok sayilir (math.isfinite kontrolu) - risksiz. Gecerli-yakin
        # (114) SAHTE ENGEL riski tasir - oncelik bu citada.
        parameters=[{'acik_esik_derece': 33.0, 'use_sim_time': use_sim_time}],
        respawn=True,
        respawn_delay=3.0,
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
    # 4c. laser_scan_matcher (CSM/ICP tabanli scan matching): onceden SADECE
    # referans/karsilastirma icin calistiriliyordu (konum(x,y) artik ondan
    # DEGIL, dead-reckoning'den geliyordu, /odom_lsm hicbir yerde tuketilmiyordu
    # - bkz. konum_birlestirici.py basi). 2026-09-01: GPS/mavros'un ana launch'a
    # dahil edilmesinden sonra Jetson'da surdurulebilir asiri yuk tespit edildi
    # (yuk ortalamasi 6 cekirdekte ~18-20, RAM tukenip swap'e dustu) - bu dugum
    # tek basina ~%35 CPU aliyordu ve fonksiyonel HICBIR ciktisi tuketilmiyordu,
    # bu yuzden TAMAMEN CIKARILDI (kullanici onayiyla). Referans/karsilastirma
    # gerekirse gecici olarak elle (ayri terminalde) tekrar calistirilabilir.

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
        respawn=True,
        respawn_delay=3.0,
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
        # GPS KONUM DUZELTMESI ve GPS YON (HEADING) KAPATILDI (2026-09-06,
        # kullanici karari: "gpsi iptal edelim cok kayma yapiyor").
        #
        # OLCUM: arac FIZIKSEL OLARAK DURURKEN /gps_katki_durumu
        # "RTK FLOAT ~5cm/sn ile duzeltiliyor (kalan fark 1034cm)" diyordu -
        # yani duzeltici, 10.34 METRE uzaktaki bir GPS konumuna dogru pozu
        # saniyede 5 cm cekiyordu. Bu, 30 saniyede ~1.5 m hayalet hareket
        # demek; costmap'e her karede baska yere damgalanan LIDAR taramasi
        # yuzunden Nav2 planlayici "plan uretemedi" hatasi veriyordu.
        # RTK FIXED degil FLOAT cozum alindigi icin GPS konumu zaten
        # gezingen; duzeltici o gezinmeyi odometriye tasiyordu.
        #
        # Esikler fix_type ile karsilastiriliyor (konum_birlestirici.py
        # satir 405/433/460: "if msg.fix_type < esik: return"), bu yuzden
        # 99 = ULASILAMAZ = ilgili islev HIC calismaz. Temiz kapatma,
        # yan etkisi yok.
        #
        # ANCHOR KASITLI OLARAK ACIK BIRAKILDI (5): GPS baslangic noktasi
        # kurulmaya devam ediyor, boylece gps_hedef_donusturucu.py'nin
        # GPS hedefi -> yerel hedef donusumu CALISMAYA DEVAM EDIYOR.
        # Kapatilan tek sey, GPS'in odometriyi SURUKLEMESI.
        #
        # GERI ACMAK ICIN: asagidaki iki 99 degerini eski hallerine dondurun
        # (correct: 5, heading: 3). Yeniden derlemeden denemek icin:
        #   ros2 param set /konum_birlestirici gps_correct_min_fix_type 5
        parameters=[{
            'use_sim_time': use_sim_time,
            'gps_correct_min_fix_type': 99,   # KAPALI (eski: 5)
            'gps_heading_min_fix_type': 99,   # KAPALI (eski: 3)
        }],
        respawn=True,
        respawn_delay=3.0,
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

    # 7a. CANLI TESTTE BULUNDU (2026-09-01): lifecycle_manager_navigation'in
    #     bond_timeout'u nav2_bringup/navigation_launch.py'de SABIT 4.0s
    #     (params_file kullanmiyor, sadece 3 inline dict: use_sim_time/
    #     autostart/node_names - nav2_params.yaml'daki lifecycle_manager
    #     ayarlari BU YUZDEN hic okunmuyordu). Sistem asiri yuklu iken
    #     (yuk ortalamasi 6 cekirdekte ~15-20) controller_server/bt_navigator
    #     bazen 4s icinde bond heartbeat'i gonderemiyor, lifecycle_manager
    #     onlari "COKMUS" sanip TUM navigasyon yiginini kapatiyordu
    #     (birkaç saniyede kendi kendine toparlaniyordu ama otonom bir
    #     gorev ortasinda olsa kesinti yaratirdi). params_file'a
    #     bond_timeout eklemek ISE YARAMIYOR (yukarida aciklandi) - runtime'da
    #     `ros2 param set` ile ayarlanip 4.0->12.0 canli DOGRULANDI (calisiyor,
    #     read-only degil).
    #
    # KESIN COZUM (2026-09-02, CANLI TESTTE BULUNDU): SABIT gecikmeli (10/
    # 20/40s) deneme YARISI KAYBETTI - "Managed nodes are active" (bond
    # timer'in gercekten baslama ani) sistem yukune gore 15-25s arasinda
    # DEGISKEN bir zamanda oluyor, bu yuzden t=10/20s denemeleri servis
    # HENUZ hazir degilken calisip SESSIZCE basarisiz oluyordu (timeout 8
    # ile) - bond_timeout hep varsayilan 4.0'da KALDI, "Have not received
    # a heartbeat" 8s sonra tetiklenip TUM yigini kapatti VE lifecycle_
    # manager kendi kendine TOPARLANAMADI (dakikalarca "Deactivating
    # waypoint_follower"de TAKILI kaldi). SABIT gecikme yerine artik
    # /lifecycle_manager_navigation/set_parameters servisi GERCEKTEN hazir
    # olur olmaz (1sn araliklarla polling) calisan bir dongu - bu, servisin
    # cok erken (lifecycle_manager node'u olusur olusmaz, "Starting managed
    # nodes bringup"tan COK once, ~1-2s icinde) hazir olmasindan
    # faydalanarak bond_timeout'u BRINGUP BASLAMADAN ONCE ayarlar, boylece
    # bond timer HICBIR ZAMAN 4.0s'lik varsayilanla olusturulmaz.
    bond_timeout_ayarla = [
        ExecuteProcess(
            cmd=['bash', '-c',
                 'until ros2 param set /lifecycle_manager_navigation '
                 'bond_timeout 12.0 >/dev/null 2>&1; do sleep 1; done'],
            output='screen',
        ),
    ]

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
    #    CANLI TESTTE BULUNDU (2026-09-01): rviz2 SIGSEGV ile coktu (asiri
    #    sistem yuku/RAM baskisi altinda - yuk ortalamasi 6 cekirdekte
    #    ~18-20, RAM neredeyse tukenmis, swap kullanimda). respawn=True
    #    YOKTU - tek bir rviz2 cokmesi, kullanicinin TUM sistemi elle
    #    Ctrl+C'lemesine ("kod kendini kapatti" hissi) ve bunun da mavros/
    #    lifecycle_manager'in SIGKILL'e kadar giden kirli kapanmasina yol
    #    acti (bkz. gecmisteki DDS/SHM sorunlari, ayni kok neden). rviz2
    #    SADECE gorsellestirme - cokmesi otonom kontrolu ETKILEMEZ, bu
    #    yuzden respawn GUVENLI: tek node kendi kendine geri gelir, geri
    #    kalan sistemi (mavros dahil) hic ETKILEMEDEN.
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
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
        respawn=True,
        respawn_delay=3.0,
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
        # ExecuteProcess de respawn destekler - motor surucusu OLURSE
        # arac komut alamaz hale gelir, geri gelmesi SART (bkz. lidar_node
        # ustundeki OTOMATIK RESPAWN notu).
        respawn=True,
        respawn_delay=3.0,
    )

    # 11. Tabela (trafik levhasi) algilama - ayri bir kameradan surekli YOLO
    #     tespiti yapar, /tabela_tespit'e yayinlar, isaretli goruntuyu UDP ile gonderir.
    # KESIN COZUM (2026-09-01): Tabela_ana.pt (PyTorch, CPU-agir) yerine
    # tabela_ana.engine (TensorRT, onceden derlenmis) kullan - CANLI TESTTE
    # BULUNDU: tabela_node .pt ile tek basina %55-66 CPU tuketiyordu, Jetson
    # load average 10-13'e cikip rf2o'nun /scan islemesini geciktiriyor
    # (rf2o "Waiting for laser_scans" ile saniyelerce takiliyordu) ve Nav2
    # lifecycle bond timeout'a girip TUM yigini beklenmedik sekilde
    # yeniden baslatiyordu - odometri "kaymalari"nin asil nedeni buydu.
    # imgsz=640 (tabela_node.py'nin kendi varsayilani) bu engine ile
    # DOGRULANDI (izole testte basarili, imgsz=800 HATA verdi).
    tabela_model_path = os.path.join(pkg_share, 'models', 'tabela_ana.engine')
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
        respawn=True,
        respawn_delay=3.0,
    )

    # 12. GOREV SEKANSI KARAR DUGUMU (2026-08-31, kullanici istegi - TAM
    #     akis): etap takibi + etap 8'de rampa cikisi + Stop tabelasinda
    #     GECICI durdurma/silah-fazina gecis + hedef vurulunca inis+5m.
    #     KALICI acil-durdurma kilidini (/arac_komut) ARTIK KULLANMAZ -
    #     kullanici istegi geregi Stop sonrasi OTOMATIK devam eder (bkz.
    #     tabela_etap_yoneticisi.py docstring).
    tabela_etap_yoneticisi_node = Node(
        package='tufan_v2_ws',
        executable='tabela_etap_yoneticisi.py',
        name='tabela_etap_yoneticisi',
        output='screen',
        respawn=True,
        respawn_delay=3.0,
    )

    # 13. Egim/rampa gecislerinde LIDAR'in zemini engel sanmasini onler -
    #     pitch-tabanli, otomatik (bkz. egim_costmap_ayarlayici.py basi).
    egim_costmap_ayarlayici_node = Node(
        package='tufan_v2_ws',
        executable='egim_costmap_ayarlayici.py',
        name='egim_costmap_ayarlayici',
        output='screen',
    )

    # 14. Arka kamera - SADECE acar/yayinlar, model YOK (bkz. dosyanin basi).
    # 2026-09-01, kullanici istegi: "arka kamerayi simdilik kullanmayalim,
    # sadece on kamera ve silah kamerasi acik olsun" - Node TANIMI asagida
    # duruyor (kod silinmedi) AMA return listesine EKLENMIYOR, yani
    # LAUNCH SIRASINDA CALISTIRILMIYOR. Geri acmak icin en alttaki
    # LaunchDescription listesine 'arka_kamera_node,' satirini eklemek
    # yeterli.
    # CANLI TESTTE BULUNDU (2026-09-01): varsayilan camera_index=2, tabela'ya
    # (C270, /dev/video2) atanan indeksle CAKISIYORDU - ikisi ayni cihazi
    # ACMAYA CALISIYORDU, "Device or resource busy" / "can't open camera by
    # index" hatasina yol aciyordu (hangisi ONCE baslarsa o kazaniyordu).
    # v4l2-ctl ile ikinci bir C270'in /dev/video4'te oldugu dogrulandi -
    # arka kamera BURAYA tasindi.
    arka_kamera_node = Node(
        package='tufan_v2_ws',
        executable='arka_kamera_node.py',
        name='arka_kamera_node',
        output='screen',
        parameters=[{
            # camera_index artik SADECE YEDEK: node once benzersiz kimlikle
            # acmayi dener (/dev/v4l/by-id -> /dev/v4l/by-path), bkz.
            # arka_kamera_node.py::_kamera_kaynagi_bul.
            'camera_index': 4,
            'kamera_arama_ismi': 'C270',
            'kamera_usb_port': '1-2.2.3',
            'kamera_fps': 5,
            'video_target_ip': '192.168.1.20',
            'video_target_port': 5002,
        }],
        respawn=True,
        respawn_delay=3.0,
    )

    # 14b. Serbest yon takipcisi (gap-following) - haritasiz/bilinmeyen
    #      parkurda S donuslerini LIDAR'a gore reaktif takip eder (bkz. o
    #      dosyanin basi). /otonom_surus_aktif=False ile PASIF baslar,
    #      tabela_etap_yoneticisi.py yonetir (2026-09-01, kullanici istegi).
    #      DEVRE DISI (2026-09-02, kullanici istegi): canli testlerde
    #      kararsizlik/titreme sorunlari yasandi - operator artik GCS
    #      radar ekranindan (tikla-surukle) veya RViz'in "2D Goal Pose"
    #      aracindan TEK TEK elle hedef veriyor (/goal_pose, bt_navigator
    #      DOGRUDAN isliyor - goal_manager_node.py'yi bypass eder, zaten
    #      calisir durumda, HICBIR SEY DEGISTIRILMEDI). Bu node aktif
    #      kalsaydi /otonom_surus_aktif tetiklenince KENDI /ugv_goal'ini
    #      gonderip operatorun manuel /goal_pose hedefleriyle AYNI Nav2
    #      action sunucusu icin YARISIRDI. Node TANIMI asagida duruyor
    #      (kod silinmedi) AMA return listesine EKLENMIYOR.
    serbest_yon_takipcisi_node = Node(
        package='tufan_v2_ws',
        executable='serbest_yon_takipcisi.py',
        name='serbest_yon_takipcisi',
        output='screen',
    )

    # 14c. Kaba/kalici uzun-engel haritasi (2026-09-02, kullanici istegi) -
    #      SADECE /surus_modu=MANUEL iken (manuel tur) uzun engelleri
    #      biriktirir, /kaba_harita (OccupancyGrid) olarak Nav2'nin
    #      static_layer'ina besler (bkz. o dosyanin basi + nav2_params.yaml).
    #      DEVRE DISI (2026-09-02, kullanici istegi): canli testte odometri
    #      kaymasinin (RTK Fixed->Float dususleri) haritayi guvenilmez
    #      kildigi goruldu - operator GCS radar ekraninda ELLE nokta atma
    #      sistemine gecildi (araca-goreceli /goal_pose, odometri kaymasindan
    #      ETKILENMIYOR). Node TANIMI asagida duruyor (kod silinmedi) AMA
    #      return listesine EKLENMIYOR - istenirse ileride tekrar denenebilir.
    kaba_harita_olusturucu_node = Node(
        package='tufan_v2_ws',
        executable='kaba_harita_olusturucu.py',
        name='kaba_harita_olusturucu',
        output='screen',
    )

    # 15. Silah (turret) alt-sistemi - onceden AYRI/bagimsiz turret.launch.py
    #     ile calistiriliyordu (elle, ayri terminal); gorev sekansinin Stop
    #     tabelasindan sonra /turret_model_aktif + /silah_modu'nu kendi
    #     yonetebilmesi icin artik ANA launch'a DAHIL edildi - kamera launch
    #     aninda acilir (turret_node.py varsayilani: model KAPALI, bkz. o
    #     dosyanin notu), model/izleme SADECE gorev sekansi komut verince baslar.
    # CANLI TESTTE BULUNDU (2026-09-01): turret.launch.py'nin video_target_ip
    # varsayilani '10.40.64.44' idi (192.168.1.20 DEGIL, turret_node.py'nin
    # KENDI ic varsayilani ise '10.40.64.48' - UCU FARKLI deger, hicbiri
    # dogru degildi) - burada HIC OVERRIDE EDILMIYORDU, yani silah goruntusu
    # 192.168.1.20'ye HIC GITMIYORDU (sadece tabela/on kamera gidiyordu).
    # PORT NOTU: kullanici son istekte tersine cevirdi - tabela 5000, silah
    # 5001 (bkz. tabela_video_target_port yorumu, ayni tarih). CAMERA_INDEX:
    # v4l2-ctl ile /dev/video1 = C922 Pro Stream Webcam oldugu dogrulandi,
    # silah kamerasi C922 olmali (kullanici istegi).
    turret_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'turret.launch.py')
        ),
        launch_arguments={
            'camera_index': '1',
            'video_target_ip': '192.168.1.20',
            'video_target_port': '5001',
        }.items(),
    )

    # 16. GPS/RTK - Cube Orange/mavros baglantisi (SADECE cube_orange_mavros.
    #     launch.py - gps_bringup.launch.py'nin TAMAMI DEGIL, fix_qos_bridge.py
    #     ve gps_hedef_donusturucu.py bilerek AYRI birakildi, kullanici
    #     istegi 2026-09-01). Onceden KASITLI OLARAK ayri tutuluyordu
    #     (donanim her zaman takili/dogrulanmis olmayabilir diye); mavros
    #     baglanamasa/GPS gelmese BILE konum_birlestirici.py sessizce saf
    #     dead-reckoning'e duser (degismedi), yani bu dahil etme ana sistemi
    #     riske atmiyor.
    cube_orange_mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'cube_orange_mavros.launch.py')
        ),
    )

    # 14d. ON BOSLUK NOKTA ATICI (2026-09-06, kullanici istegi: "arac
    #      onune noktalar atarak gitsin, lidarin bos gordugu yere") -
    #      odometri kaymasindan ETKILENMEYEN, her tick'te LIDAR'dan
    #      YENIDEN uretilen kisa menzilli hedefler. Slalom / 3m koridor
    #      ortalama / donusler TEK puanlamadan; inis-zemini-duvar-sanma,
    #      yan egim ve tirmanma icin ayri modlar; dik engel (rampa) icin
    #      duz-dik yuzey tespiti. VARSAYILAN PASIF - saha testinde
    #      /oto_nokta_aktif ile acilir (bkz. o dosyanin basi).
    on_bosluk_nokta_atici_node = Node(
        package='tufan_v2_ws',
        executable='on_bosluk_nokta_atici.py',
        name='on_bosluk_nokta_atici',
        output='screen',
        respawn=True,
        respawn_delay=3.0,
    )

    # 18. SAGLIK BEKCISI (2026-09-09) - "ayakta ama sessiz" dugumleri
    #     tespit edip SIGTERM ile sonlandirir; yukaridaki respawn=True
    #     onlari geri getirir. 9 Eylul'de LIDAR adaptoru TEK acilista 6 KEZ
    #     koptu ve sllidar_node her seferinde OLU fd ile ayakta kalip
    #     SESSIZCE sustu (log yok, %CPU normal) - respawn tek basina bu
    #     durumda ISE YARAMIYOR, cunku sureç olmuyor. Bkz. saglik_bekcisi.py
    #     dosyasinin basindaki ayrintili not.
    saglik_bekcisi_node = Node(
        package='tufan_v2_ws',
        executable='saglik_bekcisi.py',
        name='saglik_bekcisi',
        output='screen',
        parameters=[{
            # Yigin ~60sn'de aciliyor (TensorRT motorlari + Nav2 lifecycle);
            # bu sure boyunca topic'ler HAKLI olarak sessiz - pay bundan
            # UZUN olmali, yoksa bekci acilisi hic tamamlanmaz.
            'baslangic_payi_s': 75.0,
            'kontrol_araligi_s': 2.0,
            'etkin': True,
        }],
        respawn=True,
        respawn_delay=5.0,
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
        rf2o_node,
        konum_birlestirici_node,
        navigation_launch,
        *bond_timeout_ayarla,
        goal_manager_node,
        goal_manager_watchdog_node,
        nav2_cmd_vel_gate_node,
        rviz_node,
        surus_koprusu_node,
        motor_driver_node,
        tabela_node,
        # ARKA KAMERA GERI ACILDI (2026-09-09, kullanici: "arka kamerayi
        # kontrol et yayin yapiyor mu diye"). 2026-09-01'de KASITLI olarak
        # bu listeden cikarilmisti ("simdilik kullanmayalim") - yani
        # node hic BASLAMIYORDU, dolayisiyla 5002'ye HIC yayin gitmiyordu.
        # UC kameranin USB izokron bant genisligini tuketme riski
        # (2026-09-01'de "No space left on device") ucunun de MJPG'ye
        # gecirilmesiyle giderilmisti - geri acarken UCUNUN DE aktigi
        # CANLI dogrulanmalidir.        # 2026-09-09 TEKRAR KAPATILDI - CANLI OLCUMLE kanitlandi: arka
        # kamera acikken SILAH kamerasi (C922) hic yayin yapamiyor.
        # Olcum: 3 kamera=1160 KB/s, arka kapali=499, silah kapali=521
        # -> silah payi 19 KB/s, yani cap.read() surekli False donuyor
        # (klasik UVC izokron bant genisligi tukenmesi; OpenCV bunu
        # SESSIZCE yapar, log YOK - 2026-09-01'de de ayni sekilde
        # bulunmustu). Ucu de MJPG olmasi YETMIYOR. Kullanici onayiyla
        # ("gerekirse arka kamerayi durdur") arka kamera yine listeden
        # cikarildi - silah kamerasi GOREV KRITIK.
        # 2026-09-10 GERI ACILDI - bant genisligi COZULDU. Cekirdek
        # kaniti "Not enough bandwidth for altsetting 4" idi; rezervasyon
        # KARE HIZI ile orantili oldugu icin cozum FPS dusurmek oldu
        # (cozunurluk dusurmek ISE YARAMADI - 160x120 bile ayni hatayi
        # verdi). Ayrica kullanici kablolari yeniden duzenledi: silah
        # (C922) ve on kamera dogrudan Jetson tarafinda (1-2.1 / 1-2.3),
        # goruntu ISLEMEYEN arka kamera hub'da (1-2.2.3) ve DUSUK fps'te.
        # Canli dogrulandi: silah 15 + on 15 + arka 5 -> UCU DE calisiyor.
        arka_kamera_node,
        tabela_etap_yoneticisi_node,
        egim_costmap_ayarlayici_node,
        on_bosluk_nokta_atici_node,
        saglik_bekcisi_node,
        turret_launch,
        cube_orange_mavros_launch,
    ])
