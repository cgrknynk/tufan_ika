"""Cube Orange (ArduPilot) icin mavros baglantisini baslatir.

Cube Orange'in USB seri portu (/dev/ttyACM0, ttyACM1, ...) sabit degil --
Cube'un kendi USB yeniden-enumerasyonu ya da baska bir ACM cihazinin (orn.
Arduino Uno) takili/cikarilmis olmasina gore numara degisebiliyor. Bu yuzden
sabit bir /dev/ttyACMx yazmak yerine, calisma zamaninda USB'yi tarayip
Cube Orange'i (VID:PID 2DAE:1016, "Hex/ProfiCNC CubeOrange") bulup dogru
portu mavros'a veriyoruz.

Cube, ayni VID:PID ile BIRDEN FAZLA ACM arayuzu ile enumerate olabiliyor
(orn. ACM0 = USB interface 0 = MAVLink, ACM1 = interface 2 = baska bir akis).
ArduPilot'ta MAVLink/USB her zaman interface 0'dadir, bu yuzden en dusuk
interface numarali portu seciyoruz.

Kullanim:
    ros2 launch tufan_v2_ws cube_orange_mavros.launch.py
"""
import os
import serial.tools.list_ports

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

CUBE_ORANGE_VID_PID = {(0x2DAE, 0x1016)}  # Hex/ProfiCNC CubeOrange
VARSAYILAN_PORT = '/dev/ttyACM0'
BAUDRATE = 57600


def cube_orange_portu_bul():
    """Cube Orange'in MAVLink tasiyan (en dusuk USB interface numarali)
    portunu bulur. Bulamazsa None doner."""
    adaylar = []
    for p in serial.tools.list_ports.comports():
        if (p.vid, p.pid) not in CUBE_ORANGE_VID_PID:
            continue
        arayuz_no = 0
        # p.location orn: "1-2.3:1.0" -> ":" sonrasi "1.0", son eleman (0) interface no.
        if p.location and ':' in p.location:
            try:
                arayuz_no = int(p.location.split(':', 1)[1].split('.')[-1])
            except (ValueError, IndexError):
                pass
        adaylar.append((arayuz_no, p.device))
    if not adaylar:
        return None
    adaylar.sort(key=lambda x: x[0])
    return adaylar[0][1]


def launch_setup(context, *args, **kwargs):
    port = cube_orange_portu_bul()
    if port is None:
        port = VARSAYILAN_PORT
        print(f"⚠️  UYARI: Cube Orange USB'de bulunamadi, varsayilan {port} deneniyor!")
    else:
        print(f"✅ Cube Orange bulundu: {port}")

    fcu_url = f'{port}:{BAUDRATE}'

    # CANLI TESTTE BULUNDU (2026-09-01) - ASIL GPS SORUNU BUYDU: apm.launch
    # kendi 'namespace' argumanini varsayilan 'mavros' ile tanimliyor (bkz.
    # apm.launch/node.launch), AMA ROS2 launch'ta DeclareLaunchArgument/
    # LaunchConfiguration isim BAZINDA TUM launch agacinda KUERSELDIR - AYNI
    # isimde bir LaunchConfiguration DAHA ONCE BASKA BIR dosyada tanimlanmis/
    # ayarlanmissa, sonraki ayni-isimli tanimlama SESSIZCE YOK SAYILIR. Bu
    # dosya artik tufan_mppi.launch.py icinde Nav2'nin navigation_launch.py'si
    # ile AYNI agacta calisiyor (2026-09-01'de GPS ana launch'a dahil edildi)
    # - nav2_bringup/navigation_launch.py KENDI 'namespace' argumanini
    # default_value='' ile tanimliyor VE listede apm.launch'tan ONCE geliyor,
    # bu yuzden mavros'un 'namespace' varsayilani ('mavros') hic devreye
    # girmiyor, mavros KOK ad-alaninda (/state, /global_position/raw/fix,
    # /set_stream_rate - /mavros/ ONEKI OLMADAN) baslatiliyordu.
    # konum_birlestirici.py/fix_qos_bridge.py ise /mavros/... ONEKIYLE abone
    # oldugu icin GPS/RTK verisi HICBIR ZAMAN ULASMIYORDU (yayinci 0).
    # Bu, GPS'in "daha once calisiyordu" olup simdi calismamasinin tam
    # sebebi: eskiden AYRI calisan gps_bringup.launch.py'de Nav2 hic yoktu,
    # cakisma olmuyordu. Cozum: namespace'i ACIKCA 'mavros' olarak GECIR -
    # boylece Nav2'nin ne yaptigindan BAGIMSIZ, bu deger zorlanir.
    # CANLI TESTTE BULUNDU (2026-09-01) - MAVROS CPU KULLANIMI COK YUKSEKTI
    # (%70-110, tek basina): apm.launch, mavros'un varsayilan apm_pluginlists.
    # yaml'ini (56 plugin) kullaniyordu - kod tabaninda GERCEKTEN tuketilen
    # mavros topic'i SADECE IKI TANE (/mavros/global_position/raw/fix,
    # /mavros/gpsstatus/gps1/raw - bkz. konum_birlestirici.py). Cogu plugin
    # (setpoint_*, gimbal_control, terrain, adsb, optical_flow vb.) drone'a
    # ozgu veya bu projede hic kullanilmiyor. apm.launch KENDISI pluginlists_
    # yaml'i disariya bir <arg> olarak ACMIYOR (sadece fcu_url/gcs_url/
    # tgt_system/tgt_component/fcu_protocol/respawn_mavros/namespace) - bu
    # yuzden apm.launch yerine DOGRUDAN onun ic ice ICERDIGI node.launch
    # cagriliyor, ayni varsayilanlarla (gcs_url='', tgt_system='1',
    # tgt_component='1', fcu_protocol='v2.0') AMA pluginlists_yaml olarak
    # KENDI daraltilmis config/rover_pluginlists.yaml dosyamiz verilerek
    # (bkz. o dosyanin basi - hangi pluginlerin neden kapatildigi/
    # KAPATILMADIGI acikca yazili). config_yaml (apm_config.yaml, plugin
    # PARAMETRELERI - ornegin distance_sensor ayarlari) DEGISTIRILMEDI.
    pkg_share_mavros = FindPackageShare('mavros')
    rover_pluginlists_yaml = os.path.join(
        get_package_share_directory('tufan_v2_ws'), 'config', 'rover_pluginlists.yaml'
    )
    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share_mavros, 'launch', 'node.launch'])
        ),
        launch_arguments={
            'fcu_url': fcu_url,
            'gcs_url': '',
            'tgt_system': '1',
            'tgt_component': '1',
            'fcu_protocol': 'v2.0',
            'respawn_mavros': 'false',
            'namespace': 'mavros',
            'pluginlists_yaml': rover_pluginlists_yaml,
            'config_yaml': PathJoinSubstitution([pkg_share_mavros, 'launch', 'apm_config.yaml']),
        }.items(),
    )

    # CANLI TESTTE BULUNDU: mavros baglandiktan sonra /mavros/imu/data VE
    # /mavros/imu/mag (Here4'un RM3100 pusulasi dahil, ArduPilot'un kendi
    # AHRS/EKF'siyle birlestirdigi yon tahmini) VE GPSRAW (fix_type, h_acc
    # vb.) HIC YAYIN YAPMIYORDU - FCU bu mesajlarin MAVLink akisini
    # varsayilan olarak (bu ArduPilot Rover kurulumunda) DUSUK/SIFIR hizda
    # gonderiyor. /mavros/set_stream_rate servisi (stream_id=0=STREAM_ALL,
    # bkz. asagidaki not - servis adi/ad-alani zaman icinde DEGISTI) ile
    # ACIKCA istenince ANINDA akmaya basliyor.
    #
    # TEK SEFERLIK 6s GECIKMELI DENEME YETERSIZ CIKTI (canli testte
    # dogrulandi - ~1 saat sonra kontrol edildiginde GPSRAW hala HIC
    # akmiyordu, mavros baglantisi saglikliydi ama stream istegi
    # ULASMAMISTI/kacirilmisti - kesin kok neden izlenemedi, muhtemelen
    # mavros'un servis sunucusu 6.saniyede henuz tam hazir degildi ve
    # ExecuteProcess'in kendisi TEK SEFER calisip sonra tekrar denemeden
    # cikiyordu). Bu yuzden tek TimerAction yerine, artan gecikmelerle
    # BIRKAC KEZ deneyen bir dizi TimerAction kullanilir - herhangi biri
    # basarili olursa mavros zaten yeni istekleri kabul eder, fazladan
    # cagrilar ZARARSIZDIR (idempotent - sadece ayni hizi tekrar ister).
    #
    # CANLI TESTTE BULUNDU (2026-09-01, ILK TESHIS): servis adi /mavros/
    # ONEKI OLMADAN (/set_stream_rate) kayitliydi - o SIRADA mavros henuz
    # namespace='mavros' ZORLANMADAN, KOK ad-alaninda calisiyordu (bkz.
    # bu dosyanin ustundeki "ASIL GPS SORUNU BUYDU" notu).
    #
    # CANLI TESTTE BULUNDU (2026-09-01, GUNCELLEME - namespace duzeltmesi
    # SONRASI): namespace='mavros' zorlaninca (yukarida) servis KAYDI DA
    # /mavros/set_stream_rate'e TASINDI (`ros2 service list` ile
    # dogrulandi) - bu dosyadaki cagri hala ESKI /set_stream_rate adini
    # kullandigi icin servis ARTIK O ADRESTE YOKTU, cagri yine SONSUZA
    # KADAR bekliyordu (GPS kapali alanda "hic akmiyor" sanildi, aslinda
    # otomatik stream-rate istegi hic ULASMIYORDU - manuel /mavros/
    # set_stream_rate cagrisiyla ANINDA akmaya basladi, dogrulandi).
    # Duzeltildi: /mavros/set_stream_rate. `timeout` sarmalayici yine de
    # birakildi - servis adi ileride TEKRAR degisirse zombi surec
    # birikmesine karsi ucretsiz bir guvenlik agi.
    request_stream_rate = [
        TimerAction(
            period=delay,
            actions=[
                ExecuteProcess(
                    cmd=['timeout', '8',
                         'ros2', 'service', 'call', '/mavros/set_stream_rate',
                         'mavros_msgs/srv/StreamRate',
                         '{stream_id: 0, message_rate: 10, on_off: true}'],
                    output='screen',
                ),
            ],
        )
        for delay in (6.0, 15.0, 30.0, 60.0)
    ]
    return [mavros_launch, *request_stream_rate]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
