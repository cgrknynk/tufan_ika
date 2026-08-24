#!/usr/bin/env python3
"""Nav2/MPPI'nin ürettiği hız komutunu motorlara göndermeden ÖNCE, LiDAR
verisiyle bağımsız bir "ileri yol açık mı" kontrolünden geçirir.

GERCEK ARAC PORTU (kaynak: ros2_ws/src/tufan_simulation/scripts/
nav2_cmd_vel_gate.py). Algoritma AYNEN korunuyor; GERCEK ARACA OZGU UC
uyarlama yapildi (bkz. asagida "GERCEK ARAC UYARLAMASI" notlari):
  1) Konum kaynagi /odometry/filtered (EKF) yerine /odom (konum_birlestirici.py).
  2) LiDAR giris konusu /scan_filtered (simulasyona ozgu vibration_filter_node.py
     ciktisi, bu araca tasinmadi) yerine dogrudan 'scan' (bu aracta zaten
     Nav2 costmap'inin kullandigi, scan_front_filter.py ciktisi olan asil konu).
  3) *** ONEMLI: lidar_angle_offset_rad (YENI parametre, simulasyonda yoktu) ***
     Bu aractaki LiDAR'in HAM aci referansi, govde onune gore 180 derece
     donuk monte/yorumlaniyor (bkz. scan_front_filter.py docstring: "ham
     scan'in +-180 derece civari GERCEK ONDUR; 0 derece civari GERCEK
     ARKADIR"). Simulasyondaki orijinal kod, ham aciyi DOGRUDAN "govde onu
     x ekseni = aci 0" kabul ediyordu (sim LiDAR'i duz monteliydi) - bu
     aracta duzeltilmeden kullanilirsa gate TAM TERSINE, aracin ARKASINI
     "on" sanip guvenlik kontrolunu YANLIS yonde yapar. Asagida her ham
     aciya +pi eklenerek (scan_front_filter.py ile ayni fiziksel kalibrasyon)
     govde-onu referansina donusturuluyor.

Mimari:
  controller_server (MPPI) -> velocity_smoother -> /cmd_vel  (Nav2'nin HAM isteği)
  bu node: /cmd_vel + scan -> /cmd_vel_gated        (GERÇEK motor komutu)
  surus_koprusu.py artık /cmd_vel_gated dinler (bkz. o dosyanın başı).

Neden: MPPI/NavFn, dar bir geçit TAMAMEN kapalıyken bazen "geriye doğru bir
kaçış rotası" üretip aracı garip biçimde döndürebiliyor (canlı testte
tekrar tekrar gözlemlendi - kayar engel senaryosunda). Nav2'nin KARAR VERME
sürecine güvenmek yerine, burada DOĞRUDAN LiDAR verisinden BASİT VE
DETERMİNİSTİK bir "önümdeki dar koridor şeridi boş mu" kontrolü yapılıyor.

ÖNEMLİ (2. tur - statik/hareketli engel ayrımı): tam-duruş güvenliği SADECE
kayar engel gibi GERÇEKTEN HAREKETLİ engeller için tasarlanmıştı, ama canlı
testte dubalar/dar geçit gibi TAMAMEN STATİK engellerde de araya girip
gereksiz donmalara yol açtığı görüldü. Çözüm: /odom ile aracın KENDİ
hareketini telafi edip, kutu içindeki en yakın noktanın DÜNYA (odom)
çerçevesindeki konumunun zaman içinde GERÇEKTEN değişip değişmediğini izliyoruz:
  - Konum belirgin şekilde değişiyorsa (motion_threshold_m'den fazla,
    motion_window_sec süresince) -> GERÇEKTEN hareketli bir engel -> tam
    duruş + watchdog/kaçış mantığı uygulanır.
  - Konum sabit kalıyorsa (araç yaklaşsa/dönse bile DÜNYADA aynı yerde
    duruyorsa) -> STATİK engel (duba, duvar) -> gate araya GİRMEZ, Nav2'nin
    komutu OLDUĞU GİBİ iletilir; kaçınmayı normal CostCritic/MPPI yapar.
  - Henüz yeterli veri yoksa (sınıflandırma penceresi dolmadıysa) -> emniyetli
    tarafta kal (tam duruş) - bu en fazla motion_window_sec kadar sürer.

TF KULLANILMAZ (kasıtlı): LiDAR'ın base_link'e göre sabit konumu (URDF'teki
lidar_joint origin'i) doğrudan sabit bir ofset olarak kullanılıyor; araç
pozisyonu için de TF yerine goal_manager_node'daki gibi doğrudan /odom
aboneliği kullanılıyor - bu, TF Buffer/Listener ile yaşanan executor
kilitlenmesi sınıfı sorunları baştan eler.
"""
import math
from collections import deque

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


def _angle_diff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


class Nav2CmdVelGate(Node):

    def __init__(self):
        super().__init__('nav2_cmd_vel_gate')
        self.declare_parameter('safety_length_m', 2.0)
        # width: taranan toplam genişlik (her iki yanı da görebilmek için
        # araç ihtiyacından geniş). width_needed: aracın GEÇMESİ için gereken
        # minimum sürekli boşluk (footprint 1.10m + 2x0.25 padding = 1.60m).
        self.declare_parameter('safety_width_m', 2.4)
        self.declare_parameter('width_needed_m', 1.6)
        self.declare_parameter('clear_confirm_sec', 1.0)
        # Nav2'nin resmi collision_monitor'ünde de VAR olan bir mekanizma
        # (bkz. docs.nav2.org - "zero velocity timeout"): tam duruş
        # SÜRESİZ sürerse, araç kötü bir açıyla (ör. dar bir dönüşte
        # duvara paralel/yakın) TAKILI kaldığında KENDİNİ DÜZELTEMEZ ve
        # SONSUZA DEK kilitli kalır. Belirli bir süre (stuck_timeout_sec)
        # KESİNTİSİZ blocked kalınırsa, kısa bir "kaçış penceresi"
        # (escape_duration_sec) açılıp SADECE dönüşe (linear hâlâ sıfır)
        # izin verilir.
        self.declare_parameter('stuck_timeout_sec', 20.0)
        self.declare_parameter('escape_duration_sec', 4.0)
        # Statik/hareketli engel ayrımı için hareket izleme penceresi ve eşiği.
        self.declare_parameter('motion_window_sec', 1.2)
        self.declare_parameter('motion_threshold_m', 0.15)
        # GERCEK ARAC UYARLAMASI: robot.xacro lidar_joint origin xyz="-0.5 0 ..."
        # (base_link cercevesinde); base_link, base_footprint'e gore yaw=pi
        # donuk oldugundan (bkz. robot.xacro base_joint yorumu), govde-onu
        # (base_footprint) cercevesindeki net ofset +0.5m'dir.
        self.declare_parameter('lidar_x_offset_m', 0.50)
        # robot.xacro base_link boyutu 1.65m (chassis_len) -> yarisi 0.825m
        # (base_link merkezinden on tampona mesafe) - config/nav2_params.yaml
        # footprint tanimiyla birebir tutarli.
        self.declare_parameter('base_front_x_m', 0.825)
        # *** GERCEK ARACA OZGU, SIMULASYONDA OLMAYAN parametre ***
        # bkz. dosya basindaki "GERCEK ARAC UYARLAMASI" notu (3). scan_front_
        # filter.py ile AYNI fiziksel kalibrasyon: ham LiDAR acisi 0 -> arka,
        # +-pi -> on. Bu yuzden govde-onu x/y izdusumu hesaplanmadan once ham
        # aciya bu ofset eklenir. LiDAR montaji/yonelimi degisirse (fiziksel
        # olarak yeniden konumlandirilirsa) hem bu deger hem scan_front_
        # filter.py'deki esik yeniden dogrulanmalidir.
        self.declare_parameter('lidar_angle_offset_rad', math.pi)
        self.declare_parameter('publish_rate_hz', 20.0)

        self._length = self.get_parameter('safety_length_m').value
        self._width = self.get_parameter('safety_width_m').value
        self._width_needed = self.get_parameter('width_needed_m').value
        self._confirm_sec = self.get_parameter('clear_confirm_sec').value
        self._stuck_timeout_sec = self.get_parameter('stuck_timeout_sec').value
        self._escape_duration_sec = self.get_parameter('escape_duration_sec').value
        self._motion_window_sec = self.get_parameter('motion_window_sec').value
        self._motion_threshold_m = self.get_parameter('motion_threshold_m').value
        self._blocked_since_ns = None
        self._escape_until_ns = None
        self._lidar_x = self.get_parameter('lidar_x_offset_m').value
        self._front_x = self.get_parameter('base_front_x_m').value
        self._angle_offset = self.get_parameter('lidar_angle_offset_rad').value

        self._latest_cmd = Twist()
        self._blocked = True  # başlangıçta emniyetli taraf: ilk tarama gelene kadar bekle
        self._clear_since_ns = None

        self._robot_pose = None  # (x, y, yaw) - /odom'dan (odom cercevesi)
        self._motion_history = deque()  # [(t_ns, world_x, world_y), ...]

        self._pub = self.create_publisher(Twist, '/cmd_vel_gated', 10)
        self.create_subscription(Twist, '/cmd_vel', self._on_cmd, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(LaserScan, 'scan', self._on_scan, 10)
        self.create_timer(1.0 / self.get_parameter('publish_rate_hz').value, self._tick)
        self.get_logger().info(
            f'nav2_cmd_vel_gate hazir: guvenlik kutusu {self._length}m x '
            f'{self._width}m, gerekli surekli bosluk {self._width_needed}m, '
            f'hareket esigi {self._motion_threshold_m}m/{self._motion_window_sec}s, '
            f'lidar_angle_offset={math.degrees(self._angle_offset):.0f} derece')

    def _on_cmd(self, msg: Twist):
        self._latest_cmd = msg

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self._robot_pose = (p.x, p.y, yaw)

    def _on_scan(self, msg: LaserScan):
        # ÖNEMLİ (canlı testte bulunan hata): kutu İÇİNDE HERHANGİ bir nokta
        # varsa "kapalı" saymak YANLIŞTI - açık alanda TEK YANLI bir engel
        # (ör. sadece sağ kenara yakın duran bir silindir) bile aracı
        # TAMAMEN durduruyordu, oysa aracın diğer yanından geçecek bolca
        # payı vardı. Doğrusu: kutu genişliği boyunca, aracın geçebileceği
        # (width_needed) kadar SÜREKLİ bir boşluk (gap) var mı diye bakmak -
        # sadece kayar engel gibi TÜM genişliği kapatan durumlarda "kapalı"
        # sayılır, tek taraflı engellerde MPPI kendi manevrasını yapabilir.
        now_ns = self.get_clock().now().nanoseconds
        y_hits = []
        closest = None  # (x_base, y_base) - kutudaki en yakin nokta
        angle = msg.angle_min
        for r in msg.ranges:
            if math.isfinite(r) and msg.range_min <= r <= msg.range_max:
                # GERCEK ARAC UYARLAMASI: ham aciyi govde-onu referansina
                # cevir (bkz. dosya basi + lidar_angle_offset_rad).
                effective_angle = angle + self._angle_offset
                x_base = r * math.cos(effective_angle) + self._lidar_x
                y_base = r * math.sin(effective_angle)
                if self._front_x < x_base <= self._front_x + self._length:
                    y_hits.append(y_base)
                    if closest is None or x_base < closest[0]:
                        closest = (x_base, y_base)
            angle += msg.angle_increment

        half_w = self._width / 2.0
        y_hits = sorted(y for y in y_hits if -half_w <= y <= half_w)
        boundaries = [-half_w] + y_hits + [half_w]
        max_gap = max(b2 - b1 for b1, b2 in zip(boundaries, boundaries[1:]))
        box_occupied = max_gap < self._width_needed

        if box_occupied:
            self._blocked = True
            self._clear_since_ns = None
            self._update_motion_history(now_ns, closest)
        else:
            if self._clear_since_ns is None:
                self._clear_since_ns = now_ns
            elapsed = (now_ns - self._clear_since_ns) / 1e9
            if elapsed >= self._confirm_sec:
                self._blocked = False
            self._motion_history.clear()

    def _update_motion_history(self, now_ns, closest_base):
        if closest_base is None or self._robot_pose is None:
            return
        rx, ry, ryaw = self._robot_pose

        # ÖNEMLİ (canlı testte bulunan hata): araç DÖNERKEN, "kutudaki en
        # yakın nokta" her taramada FARKLI bir fiziksel nesneye (bazen
        # koridor duvarı, bazen kayan engelin kendisi) karşılık gelebiliyor
        # - bu, izleme dizisinin farklı nesneleri karıştırıp GERÇEKTEN
        # hareketli olan kayar engeli YANLIŞLIKLA "statik" (ya da tam
        # tersi) sınıflandırmasına yol açıyordu. Aracın YÖNELİMİ son
        # eklemeden bu yana belirgin şekilde değiştiyse (dönüyorsa),
        # izlenen "en yakın nokta" artık GÜVENİLİR DEĞİL - tarihçeyi
        # sıfırlayıp baştan başlıyoruz; sınıflandırma SADECE araç
        # nispeten SABİT dururken (tam da "blocked" durumunun doğal hali)
        # olgunlaşabiliyor.
        if self._motion_history:
            _, _, _, last_yaw = self._motion_history[-1]
            if abs(_angle_diff(ryaw, last_yaw)) > 0.15:
                self._motion_history.clear()

        xb, yb = closest_base
        world_x = rx + xb * math.cos(ryaw) - yb * math.sin(ryaw)
        world_y = ry + xb * math.sin(ryaw) + yb * math.cos(ryaw)
        self._motion_history.append((now_ns, world_x, world_y, ryaw))
        cutoff = now_ns - int(self._motion_window_sec * 1e9)
        while self._motion_history and self._motion_history[0][0] < cutoff:
            self._motion_history.popleft()

    def _classify_obstacle(self):
        """True=hareketli, False=statik (guvenle siniflandirildi), None=henuz belirsiz."""
        if len(self._motion_history) < 2:
            return None
        t0 = self._motion_history[0][0]
        t_last = self._motion_history[-1][0]
        if (t_last - t0) / 1e9 < self._motion_window_sec * 0.8:
            return None  # pencere henuz yeterince dolmadi
        xs = [p[1] for p in self._motion_history]
        ys = [p[2] for p in self._motion_history]
        spread = math.hypot(max(xs) - min(xs), max(ys) - min(ys))
        return spread > self._motion_threshold_m

    def _tick(self):
        now_ns = self.get_clock().now().nanoseconds

        if not self._blocked:
            self._blocked_since_ns = None
            self._escape_until_ns = None
            self._pub.publish(self._latest_cmd)
            return

        moving = self._classify_obstacle()
        if moving is False:
            # STATİK engel: gate araya girmiyor, Nav2/MPPI kendi kaçınmasını
            # yapsın (CostCritic zaten footprint-farkında ve güvenli).
            self._blocked_since_ns = None
            self._escape_until_ns = None
            self._pub.publish(self._latest_cmd)
            return

        # moving is True (gercekten hareketli engel) VEYA None (henuz
        # siniflandirilamadi, en fazla motion_window_sec kadar surer) ->
        # emniyetli tarafta kal: tam duruş.
        #
        # Nav2'nin resmi collision_monitor node'u (bkz. docs.nav2.org/
        # configuration/packages/collision_monitor) TAM OLARAK bu mimariyi
        # kullanıyor. Onun "Stop" modeli tetiklendiğinde hızı TÜM eksenlerde
        # (doğrusal VE açısal) sıfırlar - kısmi geçiş YOKTUR. Daha önce
        # sadece linear.x'i sıfırlayıp angular.z'yi Nav2'den geçirmek
        # DENENDİ: bu, Nav2/MPPI'nin "ileri gidemiyorum, dönerek kaçış
        # rotası deneyeyim" iç dürtüsünü DOĞRUDAN motorlara taşıyordu -
        # araç ilerlemese de YERİNDE DÖNÜP duruyordu. Ama SAF tam duruş da
        # KENDİ SORUNUNU getirdi: araç bir kez kötü bir açıya (ör. dar U
        # dönüşünde duvara paralel) girerse SONSUZA DEK kilitli kalıyordu.
        # Bu yüzden watchdog: KESİNTİSİZ blocked süresi eşiği aşarsa kısa
        # bir "kaçış penceresi" açılıp SADECE dönüşe (linear hâlâ sıfır)
        # izin verilir.
        # GECICI OLARAK DEVRE DISI (kullanici istegiyle - bkz. asagida):
        # hareketli-engel tam-durus + kacis-penceresi mantigi burada
        # KAPATILDI. Kayar engel testi (sliding_obstacle_controller.py,
        # simulasyona ozgu) icin tasarlanmisti, ama gercek aracin dar
        # geçitlerden geçtigi canli testte "moving is None" (henuz
        # siniflandirilamadi) durumu YANLIŞLIKLA tetiklenip yol sadece
        # DARALDIGINDA bile aracı durduruyordu (dar geçitte "en yakın
        # nokta" siniflandirma penceresi boyunca kararli kalamiyor).
        # Su an sadece STATIK-engel davranisiyla AYNI sekilde Nav2/MPPI'nin
        # komutu oldugu gibi geçiriliyor - gercek hareketli engel guvenligi
        # YOK. Kod SILINMEDI, sadece yorum satirina alindi - gercek bir
        # hareketli-engel senaryosu (ör. baska bir arac/insan) test
        # edilecekse asagidaki blok geri acilmali.
        #
        # if self._blocked_since_ns is None:
        #     self._blocked_since_ns = now_ns
        #
        # if self._escape_until_ns is not None and now_ns < self._escape_until_ns:
        #     msg = Twist()
        #     msg.angular.z = self._latest_cmd.angular.z
        #     self._pub.publish(msg)
        #     return
        # self._escape_until_ns = None
        #
        # stuck_for = (now_ns - self._blocked_since_ns) / 1e9
        # if stuck_for >= self._stuck_timeout_sec:
        #     self.get_logger().warn(
        #         f'{stuck_for:.0f}s dir kesintisiz engelli, {self._escape_duration_sec}s '
        #         'kacis penceresi (sadece donus) aciliyor.')
        #     self._escape_until_ns = now_ns + int(self._escape_duration_sec * 1e9)
        #     self._blocked_since_ns = now_ns
        #     msg = Twist()
        #     msg.angular.z = self._latest_cmd.angular.z
        #     self._pub.publish(msg)
        #     return
        #
        # self._pub.publish(Twist())
        self._blocked_since_ns = None
        self._escape_until_ns = None
        self._pub.publish(self._latest_cmd)


def main(args=None):
    rclpy.init(args=args)
    node = Nav2CmdVelGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
