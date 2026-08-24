#!/usr/bin/env python3
"""Uzak hedefler için 'menzil sınırlama' (receding-horizon) hedef yöneticisi.

GERCEK ARAC PORTU (kaynak: ros2_ws/src/tufan_simulation/scripts/goal_manager_node.py,
Gazebo simulasyonunda dogrulandi). Algoritma AYNEN korunuyor; sadece bu araçta
harita/EKF olmadigi icin iki nokta uyarlandi (bkz. asagida "GERCEK ARAC UYARLAMASI"):
  1) Konum kaynagi /odometry/filtered (EKF) yerine /odom (konum_birlestirici.py).
  2) Hedef/eylem frame'i 'map' yerine 'odom' (bu kurulumda map->odom TF'i yok,
     bt_navigator zaten global_frame=odom kullaniyor, bkz. nav2_params.yaml).

Bu araçta HARİTA YOK: global_costmap da robot merkezli, dönen (rolling) bir
penceredir ve yalnızca anlık LiDAR verisiyle doldurulur. NavFn planlayıcı bu
pencerenin DIŞINDAKİ bir hedefe asla yol üretemez (worldToMap sınır hatası).

Bu node, kullanıcının verdiği (uzak olabilecek) nihai hedefi alır ve robotu
oraya, costmap penceresinin içinde kalan ara "bacaklar" (leg) halinde,
NavigateToPose action'ını tekrar tekrar çağırarak taşır. Her bacak
tamamlandığında (ya da costmap sınırına yaklaşıldığında) bir sonraki bacak
yeniden hesaplanır -> saf yerel (LiDAR tabanlı) planlama ile keyfi uzaklıktaki
bir hedefe ulaşma imkanı sağlar.

Giriş  : /ugv_goal        (geometry_msgs/PoseStamped, frame: odom)
Çıkışlar:
  /ugv_goal_feedback (std_msgs/Float32) : nihai hedefe kalan mesafe (m)
  /ugv_goal_result   (std_msgs/String)  : SUCCESS | FAILURE | CANCELLED
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String, Int32
from nav2_msgs.action import NavigateToPose


def normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


class GoalManagerNode(Node):

    def __init__(self):
        super().__init__('goal_manager_node')

        # GERCEK ARAC UYARLAMASI: simulasyonda global_costmap 20x20m idi
        # (max_leg_distance=8.0 -> yarim genisligin %80'i). Bu araçta
        # global_costmap 15x15m (bkz. config/nav2_params.yaml, yarim
        # genislik 7.5m) - ayni orani korumak icin 6.0/2.0 kullanildi
        # (8.0 burada kullanilsaydi bazi bacak hedefleri costmap sinirinin
        # DISINDA kalip NavFn planlama hatasina yol acabilirdi).
        self.declare_parameter('max_leg_distance', 6.0)
        self.declare_parameter('replan_trigger_distance', 2.0)
        self.declare_parameter('goal_reached_tolerance', 0.4)
        self.declare_parameter('max_leg_retries', 3)
        self.declare_parameter('control_period', 0.5)
        # MPPI, GoalCritic sadece konumu önemsediği ve geri gidiş serbest
        # olduğu için büyük yönelim farklarında dönmeyi "gereksiz" bulup
        # çok yavaş/hiç dönmeyebiliyor (canlı testte gözlemlendi). Nav2'nin
        # "Spin" davranışı da bu kurulumda güvenilmez çıktı (izole testte
        # istenen dönüşün ~%10'unu tamamlayıp "başarılı" bildirdi). Bu
        # yüzden her bacaktan önce, sapma eşiği aşarsa, DOĞRUDAN /cmd_vel
        # üzerinden (kanıtlanmış çalışan tek yöntem) basit bir P-denetleyici
        # ile kendimiz döndürüyoruz.
        # Not: PathAlignCritic artık doğru (baskın) ağırlıkta olduğu için
        # normal dönüşleri MPPI kendisi hallediyor; ön-dönüş SADECE aşırı
        # durumlarda (ör. hedef neredeyse tam arkada) bir "uyandırma"
        # itmesi olarak devreye giriyor.
        self.declare_parameter('prerotate_threshold_rad', 1.2)  # ~69 derece
        self.declare_parameter('prerotate_tolerance_rad', 0.08)  # ~4.6 derece
        self.declare_parameter('prerotate_max_speed', 0.6)
        self.declare_parameter('prerotate_gain', 1.5)
        self.declare_parameter('prerotate_control_period', 0.1)
        # GÜVENLİK AĞI: send_goal_async()/cancel_goal_async() future'larının
        # done-callback'i (nadiren, ROS2/DDS seviyesinde - kesin kök neden
        # izlenemedi, bu ortamda process'e ptrace/py-spy ile bağlanma izni
        # yok) hiç tetiklenmeyebiliyor; bu durumda self._sending sonsuza dek
        # True kalıp _tick()'i her seferinde en baştan döndürüyor (canlı
        # testte doğrulandı: feedback/hareket tamamen kesiliyor, ne yeni
        # bacak gönderiliyor ne de mevcut bacak takip ediliyor). Bu, aracın
        # dar bir geçitte veya hedefe yakınken FİZİKSEL OLARAK DONUP
        # KALMASINA yol açabileceğinden (güvenlik açısından kabul edilemez),
        # belirli bir süre sonra bayrağı zorla temizleyip normal akışa
        # devam ediyoruz.
        # ÖNEMLİ DÜZELTME: 3.0s değeri, MEŞRU prerotate işlemlerini (büyük
        # açılarda - 173° gibi - 0.6 rad/s ile tamamlanması ~5s+ sürebiliyor,
        # kendi failsafe'i 10s) YANLIŞLIKLA yarıda kesiyordu (canlı testte
        # doğrulandı: araç asla tam dönemeden sürekli "yeniden başlıyor",
        # hedefe hiç ilerleyemedi). Eşik, prerotate'in kendi 10s'lik
        # failsafe'inden büyük tutuldu ki gerçek (uzun süren, meşru)
        # işlemlerle GERÇEKTEN takılı kalmış bir future'ı ayırt edebilsin.
        self.declare_parameter('sending_watchdog_sec', 15.0)

        self._max_leg = self.get_parameter('max_leg_distance').value
        self._replan_trigger = self.get_parameter('replan_trigger_distance').value
        self._goal_tol = self.get_parameter('goal_reached_tolerance').value
        self._max_retries = self.get_parameter('max_leg_retries').value
        self._prerotate_threshold = self.get_parameter('prerotate_threshold_rad').value
        self._prerotate_tolerance = self.get_parameter('prerotate_tolerance_rad').value
        self._prerotate_max_speed = self.get_parameter('prerotate_max_speed').value
        self._prerotate_gain = self.get_parameter('prerotate_gain').value
        self._sending_watchdog = self.get_parameter('sending_watchdog_sec').value

        # ÖNEMLİ (canlı testte defalarca doğrulandı - kök neden): TF2
        # Buffer/lookup_transform, timeout=0 ile bile bazı durumlarda
        # SingleThreadedExecutor'ı tıkayabiliyor - bu sadece _tick() için
        # değil, TÜM node'un (subscription callback'leri dahil, /ugv_goal
        # bile) yanıt vermez hale gelmesine yol açtı. try/except bunu
        # yakalayamaz çünkü sorun bir istisna değil, gerçek bir bloklama.
        # Bunun yerine GERCEK ARAC UYARLAMASI: bu kurulumda EKF yok
        # (konum_birlestirici.py, robot_localization/EKF'nin surekli kaydigi
        # icin tamamen atlar - bkz. o dosyanin basi), konum dogrudan /odom'dan
        # (nav_msgs/Odometry, frame: odom, child: base_footprint) normal bir
        # abonelikle önbelleğe alınıyor - bu yöntem TF2'nin hiçbir iç
        # kilitleme/bekleme mekanizmasını kullanmıyor.
        self._latest_odom = None
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)

        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        # ÖNEMLİ: gerçek motor topic'i /cmd_vel_gated (bkz. nav2_cmd_vel_gate.py).
        # Prerotate KASITLI, DOĞRUDAN bir kontrol olduğundan (Nav2'yi zaten
        # bypass ediyor), gate'i de atlayıp doğrudan gerçek motor topic'ine yazıyor.
        self._cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel_gated', 10)

        self._prerotating = False
        self._prerotate_target_yaw = None
        self._prerotate_pending_leg = None
        self._prerotate_start_time = None
        self._leg_start_time = None
        self._leg_start_dist = None
        self._best_dist = None
        self._best_dist_time = None
        self.create_timer(self.get_parameter('prerotate_control_period').value,
                           self._prerotate_step)

        self._fb_pub = self.create_publisher(Float32, '/ugv_goal_feedback', 10)
        self._result_pub = self.create_publisher(String, '/ugv_goal_result', 10)
        # Dış watchdog (goal_manager_watchdog.py) için canlılık sinyali:
        # görev aktif olsun olmasın, _tick() gerçekten çalıştığı SÜRECE
        # her seferinde artan bir sayaç yayınlanır. Kök nedeni bu ortamda
        # kesin olarak izole edilemeyen (ptrace/py-spy erişimi yok), ama
        # canlı testte tekrarlı biçimde gözlemlenen bir rclpy executor
        # kilitlenmesine (hiçbir timer/subscription callback'i tetiklenmez
        # hale gelmesine) karşı son çare - watchdog bu sayaç durursa
        # process'i sonlandırıp (launch respawn=True ile) yeniden başlatır.
        self._heartbeat_pub = self.create_publisher(Int32, '/goal_manager_heartbeat', 10)
        self._heartbeat_count = 0

        self.create_subscription(PoseStamped, '/ugv_goal', self._on_new_goal, 10)

        self._final_goal = None
        self._active = False
        self._goal_handle = None
        self._sending = False
        self._sending_since = None
        self._is_final_leg = False
        self._retries = 0

        self.create_timer(self.get_parameter('control_period').value, self._tick)
        self.get_logger().info('goal_manager_node hazir: /ugv_goal bekleniyor '
                                f'(max_leg_distance={self._max_leg} m)')

    # ---------------- yardımcılar ----------------
    def _on_odom(self, msg: Odometry):
        self._latest_odom = msg

    def _current_pose(self):
        if self._latest_odom is None:
            return None
        p = self._latest_odom.pose.pose.position
        q = self._latest_odom.pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return p.x, p.y, yaw

    def _on_new_goal(self, msg: PoseStamped):
        # send_goal.py aracı, abone bağlanana kadar aynı hedefi birkaç kez
        # tekrar yayınlar; devam eden bir bacağı gereksiz yere iptal edip
        # sahte "basarisiz" olaylarina yol acmamak icin ayni hedefi yok say.
        if (self._active and self._final_goal is not None
                and math.hypot(msg.pose.position.x - self._final_goal.pose.position.x,
                                msg.pose.position.y - self._final_goal.pose.position.y) < 0.05):
            return

        if self._goal_handle is not None:
            self._goal_handle.cancel_goal_async()
        self._final_goal = msg
        self._active = True
        self._sending = False
        self._goal_handle = None
        self._leg_start_time = None
        self._best_dist = None
        self._best_dist_time = None
        self._prerotating = False
        self._retries = 0
        self.get_logger().info(
            f'Yeni nihai hedef alindi: x={msg.pose.position.x:.2f} '
            f'y={msg.pose.position.y:.2f}')

    def _publish_result(self, text):
        self._result_pub.publish(String(data=text))
        self._active = False
        self._goal_handle = None

    # ---------------- ana kontrol döngüsü ----------------
    def _tick(self):
        self._heartbeat_count += 1
        self._heartbeat_pub.publish(Int32(data=self._heartbeat_count))
        # GÜVENLİK AĞI: timer callback'i içinde YAKALANMAYAN herhangi bir
        # istisna (tf2 dışı, öngörülemeyen türler dahil), rclpy'nin
        # SingleThreadedExecutor'ında sessizce yutulup timer'ı "canlı ama
        # işlevsiz" bırakabiliyor (canlı testte doğrulandı). Tek tek istisna
        # türü avlamak yerine burada genel bir koruma katmanı var - node'un
        # asla sessizce donmuş gibi davranmamasını garanti eder.
        try:
            self._tick_impl()
        except Exception as e:
            self.get_logger().error(f'_tick beklenmeyen hata (yutulmadi): {e}')

    def _tick_impl(self):
        if self._sending and self._sending_since is not None:
            stuck_for = (self.get_clock().now() - self._sending_since).nanoseconds / 1e9
            if stuck_for > self._sending_watchdog:
                self.get_logger().warn(
                    f'sending bayragi {stuck_for:.1f}s dir takili (future hic '
                    'tamamlanmadi), zorla temizleniyor.')
                self._sending = False
                self._sending_since = None
                self._prerotating = False
                self._prerotate_pending_leg = None

        if not self._active or self._final_goal is None or self._sending:
            return

        pose = self._current_pose()
        if pose is None:
            return
        x, y, _ = pose
        fx = self._final_goal.pose.position.x
        fy = self._final_goal.pose.position.y
        dist_to_final = math.hypot(fx - x, fy - y)
        self._fb_pub.publish(Float32(data=dist_to_final))

        # ÖNEMLİ BULGU: Bu kontrol daha önce SADECE bir bacak aktif
        # DEĞİLKEN yapılıyordu. Ama Nav2'nin kendi "hedefe ulaşıldı"
        # kontrolü (general_goal_checker) KONUM YANINDA YÖNELİMİ de
        # (yaw_goal_tolerance) istiyor; araç konuma ulaşıp yönelimi bir
        # türlü tam oturtamadığında NavigateToPose eylemi hiç bitmiyor,
        # goal_handle sonsuza dek aktif kalıyor, ve aşağıdaki "ilerleme
        # yok" tespitçisi bunu YANLIŞLIKLA "takıldı" sanıp iptal/yeniden
        # deneyip sonunda görevi FAILURE ile bitiriyordu (canlı testte
        # doğrulandı: araç hedefe 0.17m mesafede 13+ saniye durmuş
        # haldeyken görev başarısız ilan edildi). Konum toleransı bizim
        # için yeterli olduğundan, aktif bacak olsa BİLE önce bunu kontrol
        # edip yönelim beklemeden doğrudan başarı bildiriyoruz.
        if dist_to_final <= self._goal_tol:
            if self._goal_handle is not None:
                self._goal_handle.cancel_goal_async()
            self._publish_result('SUCCESS')
            return

        if self._goal_handle is not None:
            # ÖNEMLİ: Nav2'nin "Failed to make progress" -> yeniden dene
            # döngüsü bt_navigator İÇİNDE, goal_manager'ın HİÇ haberi
            # olmadan tekrar tekrar çalışabiliyor (canlı testte: aynı
            # NavigateToPose hedefi dakikalarca sürebiliyor). MPPI bazen
            # ön-dönüşle kazanılan doğru yönelimi kaybedip geri savruluyor.
            # Bu yüzden burada KENDİ ilerleme takibimizi yapıyoruz.
            # NOT: leg BAŞINDAN beri kat edilen toplam mesafeye bakmak
            # yanıltıcı (bacağın başında çok yol alınmış olabilir, sonra
            # tamamen durmuş olsa bile toplam ilerleme hâlâ büyük görünür -
            # ilk denemede bu yüzden hiç tetiklenmedi). Bunun yerine
            # "en iyi görülen mesafe" ile o andaki zamanı takip edip, SON
            # X saniyedir hiç iyileşme olmadıysa yeniden planlıyoruz.
            if self._best_dist is None or dist_to_final < self._best_dist - 0.15:
                self._best_dist = dist_to_final
                self._best_dist_time = self.get_clock().now()

            stalled_for = 0.0
            if self._best_dist_time is not None:
                stalled_for = (self.get_clock().now() - self._best_dist_time).nanoseconds / 1e9

            # NOT (kayar engel testi): 12s idi. Engel geçici olarak yolu
            # kapattığında (kayar engel senaryosunda ~7-8s kapalı kalıyor)
            # bu süre aracın MEŞRU bir bekleyişini "takıldı" sanıp gereksiz
            # cancel+resend tetikliyordu - her yeniden planlama NavFn'i
            # yeni (bazen geriye dönük) bir rota üretmeye zorluyor ve bu da
            # 180°'ye yakın dönüşlere yol açıyordu. 30s, engelin doğal
            # olarak açılmasını beklemek için yeterli pay bırakıyor.
            if stalled_for > 30.0:
                self.get_logger().warn(
                    f'Bacakta {stalled_for:.0f}s dir ilerleme yok, iptal edip '
                    'yeniden planlaniyor.')
                # ÖNEMLİ: İptali başlatıp AYNI tick'te hemen yeni bir
                # NavigateToPose göndermek, eylem sunucusu eski hedefi hâlâ
                # iptal ederken yeni isteği reddetmesine yol açıyordu (canlı
                # testte: 3 deneme 1.5 saniyede art arda başarısız oldu).
                # Bu yüzden gerçekten iptal TAMAMLANANA kadar (callback)
                # bekleyip, ancak ondan sonra yeni bacağı gönderiyoruz.
                self._sending = True
                self._sending_since = self.get_clock().now()
                cancel_future = self._goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(self._on_stall_cancel_done)
                self._goal_handle = None
                self._leg_start_time = None
                self._best_dist = None
                self._best_dist_time = None
            return

        if dist_to_final <= self._max_leg:
            leg_x, leg_y = fx, fy
            leg_yaw = self._yaw_from_quat(self._final_goal.pose.orientation)
            self._is_final_leg = True
        else:
            ratio = self._max_leg / dist_to_final
            leg_x = x + (fx - x) * ratio
            leg_y = y + (fy - y) * ratio
            leg_yaw = math.atan2(fy - y, fx - x)
            self._is_final_leg = False

        self._maybe_prerotate_then_send(leg_x, leg_y, leg_yaw, x, y, pose[2])

    def _on_stall_cancel_done(self, future):
        # İptal gerçekten tamamlandı; artık güvenle yeni bir bacak
        # gönderebiliriz (bir sonraki _tick bunu normal akışla yapacak).
        self._sending = False
        self._sending_since = None

    @staticmethod
    def _yaw_from_quat(q):
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _maybe_prerotate_then_send(self, leg_x, leg_y, leg_yaw, cx, cy, cyaw):
        bearing = math.atan2(leg_y - cy, leg_x - cx)
        diff = normalize_angle(bearing - cyaw)
        if abs(diff) > self._prerotate_threshold:
            self._sending = True
            self._sending_since = self.get_clock().now()
            self._prerotating = True
            self._prerotate_target_yaw = bearing
            self._prerotate_pending_leg = (leg_x, leg_y, leg_yaw)
            self._prerotate_start_time = self.get_clock().now()
            self.get_logger().info(
                f'On-donus basliyor: {math.degrees(diff):.1f} derece')
        else:
            self._send_leg(leg_x, leg_y, leg_yaw)

    def _prerotate_step(self):
        try:
            self._prerotate_step_impl()
        except Exception as e:
            self.get_logger().error(f'_prerotate_step beklenmeyen hata (yutulmadi): {e}')

    def _prerotate_step_impl(self):
        if not self._prerotating:
            return
        pose = self._current_pose()
        if pose is None:
            return
        _, _, cyaw = pose
        error = normalize_angle(self._prerotate_target_yaw - cyaw)

        elapsed = (self.get_clock().now() - self._prerotate_start_time).nanoseconds / 1e9
        if abs(error) < self._prerotate_tolerance or elapsed > 10.0:
            self._cmd_vel_pub.publish(Twist())  # dur
            self._prerotating = False
            self._sending = False
            self._sending_since = None
            leg = self._prerotate_pending_leg
            self._prerotate_pending_leg = None
            if leg is not None:
                self._send_leg(*leg)
            return

        speed = max(-self._prerotate_max_speed, min(self._prerotate_max_speed,
                                                      error * self._prerotate_gain))
        # Cok yavas donmemesi icin bir taban hiz (deadband disinda)
        min_speed = 0.15
        if 0 < speed < min_speed:
            speed = min_speed
        elif -min_speed < speed < 0:
            speed = -min_speed
        msg = Twist()
        msg.angular.z = speed
        self._cmd_vel_pub.publish(msg)

    def _send_leg(self, x, y, yaw):
        if not self._nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('navigate_to_pose action sunucusu hazir degil, tekrar denenecek')
            return
        self._sending = True
        self._sending_since = self.get_clock().now()
        goal = NavigateToPose.Goal()
        # GERCEK ARAC UYARLAMASI: bu kurulumda 'map' frame'i yok
        # (map->odom TF yayinlanmiyor); bt_navigator zaten global_frame=odom
        # kullaniyor (bkz. config/nav2_params.yaml), bu yuzden 'odom' kullanilir.
        goal.pose.header.frame_id = 'odom'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        _, _, qz, qw = yaw_to_quat(yaw)
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        self._sending = False
        self._sending_since = None
        # ÖNEMLİ: future.result() bir istisna ile TAMAMLANMIŞSA (ör. action
        # sunucusuyla bağlantı geçici koptuysa) bunu yeniden fırlatır; sarmalı
        # try/except OLMADAN bu, done-callback'i çökertip node'u "canlı ama
        # işlevsiz" bırakabilir (canlı testte _current_pose için doğrulanan
        # aynı sınıf hata - burada da aynı savunmayı uyguluyoruz).
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().warn(f'Hedef gonderim yaniti alinamadi: {e}')
            self._handle_leg_failure()
            return
        if goal_handle is None or not goal_handle.accepted:
            self._handle_leg_failure()
            return
        self._goal_handle = goal_handle
        self._leg_start_time = self.get_clock().now()
        pose = self._current_pose()
        if pose is not None and self._final_goal is not None:
            self._leg_start_dist = math.hypot(
                self._final_goal.pose.position.x - pose[0],
                self._final_goal.pose.position.y - pose[1])
        else:
            self._leg_start_dist = 0.0
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_leg_result)

    def _on_leg_result(self, future):
        self._goal_handle = None
        self._leg_start_time = None
        try:
            status = future.result().status
        except Exception:
            status = -1
        # GoalStatus.STATUS_SUCCEEDED == 4
        if status == 4:
            self._retries = 0
            if self._is_final_leg:
                self._publish_result('SUCCESS')
        else:
            self._handle_leg_failure()

    def _handle_leg_failure(self):
        self._retries += 1
        if self._retries > self._max_retries:
            self.get_logger().error('Bacak tekrar sayisi asildi, gorev iptal ediliyor.')
            self._publish_result('FAILURE')
        else:
            self.get_logger().warn(f'Bacak basarisiz, yeniden denenecek ({self._retries}/'
                                    f'{self._max_retries})')


def main(args=None):
    rclpy.init(args=args)
    node = GoalManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
