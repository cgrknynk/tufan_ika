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
import serial.tools.list_ports

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

    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('mavros'), 'launch', 'apm.launch'])
        ),
        launch_arguments={'fcu_url': fcu_url}.items(),
    )

    # CANLI TESTTE BULUNDU: mavros baglandiktan sonra /mavros/imu/data VE
    # /mavros/imu/mag (Here4'un RM3100 pusulasi dahil, ArduPilot'un kendi
    # AHRS/EKF'siyle birlestirdigi yon tahmini) HIC YAYIN YAPMIYORDU - FCU
    # bu mesajlarin MAVLink akisini varsayilan olarak (bu ArduPilot Rover
    # kurulumunda) DUSUK/SIFIR hizda gonderiyor. /mavros/set_stream_rate
    # servisi (stream_id=0=STREAM_ALL) ile ACIKCA istenince ANINDA 74
    # mesaj/8s akmaya basladi. mavros baglantisinin TAM oturmasi icin
    # birkac saniye beklenip (baglanmadan once cagirmak sessizce kaybolur)
    # bu istek OTOMATIK yapiliyor - elle "ros2 service call" calistirmaya
    # gerek kalmasin diye.
    request_stream_rate = TimerAction(
        period=6.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'service', 'call', '/mavros/set_stream_rate',
                     'mavros_msgs/srv/StreamRate',
                     '{stream_id: 0, message_rate: 10, on_off: true}'],
                output='screen',
            ),
        ],
    )
    return [mavros_launch, request_stream_rate]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
