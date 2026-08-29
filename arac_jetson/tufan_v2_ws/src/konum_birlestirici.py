#!/usr/bin/env python3
"""Konum artik ICP (/odom_lsm) DEGIL, hiz+yon ENTEGRASYONU (dead-reckoning) ile
hesaplaniyor: konum(x,y)=rf2o hizi + IMU yonu entegre edilerek, yon=IMU, EKF YOK.
GPS (RTK) VARSA, biriken surunmeyi sinirli/yumusak sekilde duzeltir (asagida 4).

Bu mimari, canli testte tespit edilen DORT ayri sorunun sirayla arastirilip
duzeltilmesinin sonucudur:

1) YAW: /imu/data'nin yawi BNO055 manyetometresi/ivmeolceri kalibrasyonsuz
   oldugu icin (calib_status: accel=0 mag=0) motor akiminda 13.6s'de 2.14
   derece sicradi, /odom_lsm'in kendi (ICP) yawi ayni sartlarda 48s'de sadece
   0.2 derece sapti. Once yaw da LSM'e tasindi ama kullanicinin tercihiyle
   IMU'ya GERI DONULDU (BNO055'in gyro ekseni zaten kalibre - 3/3 - ve
   manyetik girisimden etkilenmiyor). Acisal hiz de (angular_velocity.z) ayni
   nedenle dogrudan gyro'dan alinir. BU KISIM CANLI TESTTE IYI SONUC VERDI,
   DEGISTIRILMEDI.

2) LINEER HIZ: laser_scan_matcher.cpp'nin twist'i hiz=pozisyon_farki/dt olarak
   hesapliyordu, dt GERCEK ZAMANLI (wall-clock) araligiydi - Jetson'da Nav2
   yigini CPU paylasirken ufak bir zamanlama sapmasi bile FIZIKSEL OLARAK
   IMKANSIZ hiz sicramalarina cevirdi. Cozum: rf2o_laser_odometry - hizi
   range-flow denklemleriyle DOGRUDAN hesaplayan (Jaimez & Gonzalez-Jimenez),
   kendi ic zaman farkini LIDAR donanim zaman damgasindan alan, son 4
   orneğin ortalamasiyla zaten yumusatilmis bir kaynak. BU KISIM DA IYI
   SONUC VERDI, DEGISTIRILMEDI.

3) KONUM (x,y): /odom_lsm (laser_scan_matcher PL-ICP) canli testte "sag-sol/
   donus cok iyi ama ileri-geri hic yok" seklinde basarisiz oldu (dar-FOV
   aperture problem, bkz. launch dosyasindaki scan_matcher_filter_node
   yorumu). ICP'ye guvenmek yerine konum "dead-reckoning" ile hesaplanir:
   rf2o hizi + IMU yonu entegre edilir (x += vx*cos(yaw)*dt, ...). Bilinen
   dezavantaj: saf entegrasyon sinirsiz surunme (drift) biriktirebilir -
   bkz. asagida (4), GPS varsa bunu sinirli olarak duzeltir.

4) GPS (RTK) SURUKLENME DUZELTMESI (Cube Orange/ArduPilot + Here4 GNSS +
   TUSAGA-Aktif RTK, ayri bir calismada donanimsal olarak kuruldu - bkz.
   launch/cube_orange_mavros.launch.py, launch/gps_bringup.launch.py):
   dead-reckoning'in en buyuk zaafi sinirsiz surunmedir; RTK Float (fix_type
   >=5) ~6-9cm mutlak dogruluk verdigi icin bunu periyodik duzeltebilir.
   AMA robot_localization/EKF'in NEDEN terk edildigini unutma (yukarida 3) -
   sorun ozellikle sensor gurultusunun bir Kalman filtresi icinde nasil
   davrandigiydi, GPS'in kendisi degildi. Bu yuzden BURADA TAM BIR KALMAN
   FILTRESI KULLANILMIYOR - EKF'nin basarisiz oldugu sinif hatayi tekrar
   riske atmamak icin BASIT VE GOZLEMLENEBILIR iki adimli bir yaklasim var:

   a) YON HIZALAMA: GPS'in lat/lon'u ile odom'un (x,y) DOGRUDAN
      karsilastirilamaz - biri gercek dunyaya (Dogu/Kuzey), digeri aracin
      acilis anindaki (IMU yawi=0 anindaki, KEYFI ve manyetik sapmaya tabi)
      yonelimine gore. Bu ikisi arasindaki SABIT donme farkini (heading_
      offset) bulmadan GPS konumunu odom cercevesine cevirmek YANLIS yonde
      duzeltme yapar. Cozum: GPSRAW.cog (Course Over Ground - GPS'in
      ARDIŞIK konumlardan hesapladigi GERCEK hareket yonu, pusula
      referansli, MANYETOMETREDEN TAMAMEN BAGIMSIZ) ile o andaki odom_yaw
      karsilastirilir; arac makul bir hizin (gps_heading_min_speed_mps)
      UZERINDE duz gidiyorken bu ikisinin farki dairesel ortalamayla
      (kaymali pencere) surekli guncellenir. az sayida ornekle
      (gps_heading_samples_needed) "yeterince guvenilir" sayilir.
   b) SINIRLI DUZELTME: anchor (ilk kilitlenen RTK konumu, bkz. asagida) +
      heading_offset hazir olunca, her yeni iyi-kaliteli GPS okumasi
      odom-cercevesi hedef konumuna (gps_target) cevrilir. Bu hedefe
      DOGRUDAN ATLANMAZ ("teleport" YOK) - her tick'te en fazla
      gps_correction_max_speed_mps*dt kadar (yani sabit bir "duzeltme hizi"
      ile) hedefe DOGRU CEKILIR. Boylece buyuk bir GPS sicramasi bile
      aninda degil, birkac saniyeye yayilarak uygulanir.

   ANCHOR: /gps_anchor (NavSatFix, transient_local - gec baglanan
   abonelere de ulasir) SADECE BIR KEZ, ilk gps_anchor_samples_needed
   ardisik RTK-kaliteli (fix_type>=gps_anchor_min_fix_type) okumanin
   ortalamasi olarak kilitlenir (tek bir gurultulu ornegin TUM referans
   cercevesini bozmamasi icin). /gps_heading_offset (std_msgs/Float64)
   sadece yeterli ornek toplaninca yayinlanmaya BASLAR - ilk mesajin
   varligi "hizalama hazir" anlamina gelir. Her ikisi de
   gps_hedef_donusturucu.py tarafindan AYNI anchor/hizalamayla operator
   lat/lon hedeflerini /ugv_goal'a cevirmek icin de kullanilir (tutarlilik
   icin TEK kaynak - bkz. o dosyanin basi).

   GPS topic'leri (/mavros/...) hic yayinlanmiyorsa (gps_bringup.launch.py
   calismiyorsa) bu node SESSIZCE SADECE (1)-(3)'e (saf dead-reckoning)
   geri duser - GPS KESINLIKLE OPSIYONELDIR, konum_birlestirici GPS'siz de
   AYNI SEKILDE calismaya devam eder.

   *** CANLI TESTTE DOGRULANMASI GEREKEN VARSAYIMLAR (henuz test edilmedi):
   *** heading_offset turetme yonu/isareti (bkz. _enu_to_odom yorumu),
   *** gps_correction_max_speed_mps varsayilani (0.05 m/s - cok yavas/hizli
   *** olabilir), h_acc alaninin bu donanimda gercekten dolduruldugu. ***
"""

import math
from collections import deque

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float64
from tf2_ros import TransformBroadcaster

try:
    from mavros_msgs.msg import GPSRAW
    _MAVROS_MEVCUT = True
except ImportError:
    # mavros_msgs bu ortamda kurulu degilse (ör. sadece ~/ros2_humble
    # sourcelandi, /opt/ros/humble sourcelanmadi) GPS ozelligi sessizce
    # devre disi kalir - saf dead-reckoning calismaya devam eder.
    _MAVROS_MEVCUT = False

# WGS84 - equirectangular yaklasiklik icin Dunya yaricapi (metre). Yaris/
# gorev olceginde (birkac yuz metre - birkac km) hata payi ihmal edilebilir.
_EARTH_R = 6378137.0


def _yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def _latlon_to_enu(lat, lon, lat0, lon0):
    """(lat,lon) - (lat0,lon0) farkini duz Dogu/Kuzey metreye cevirir."""
    dlat = math.radians(lat - lat0)
    dlon = math.radians(lon - lon0)
    north = dlat * _EARTH_R
    east = dlon * _EARTH_R * math.cos(math.radians(lat0))
    return east, north


def _enu_to_odom(east, north, heading_offset):
    """ENU (Dogu,Kuzey) metre vektorunu odom cercevesine cevirir.

    Turetme: heading_offset = odom_yaw=0'in PUSULA yonu (0=Kuzey, saat
    yonunde artar - GPSRAW.cog ile ayni kural). ENU'nun (Dogu=x,Kuzey=y)
    MATEMATIKSEL acisi (Dogu'dan saat yonu TERSINE) ile pusula acisi
    arasinda math_aci = 90 - pusula iliskisi vardir. odom'un x ekseni,
    ENU icinde math_aci=90-heading_offset yonunu gosterir; ENU vektorunu
    odom cercevesine cevirmek bu acinin TERSI kadar dondurmektir:
      x_odom = east*sin(heading_offset) + north*cos(heading_offset)
      y_odom = -east*cos(heading_offset) + north*sin(heading_offset)
    (heading_offset=0 -> odom x ekseni tam Kuzey ise x_odom=north,
    y_odom=-east dogru cikiyor: Kuzeye gitmek +x_odom, Doguya gitmek
    -y_odom - saga (Doguya) donus sag-el kuraliyla -y yonunde olmasi
    gerektigi icin tutarli.) *** CANLI TESTTE DOGRULANMALI. ***
    """
    x = east * math.sin(heading_offset) + north * math.cos(heading_offset)
    y = -east * math.cos(heading_offset) + north * math.sin(heading_offset)
    return x, y


class KonumBirlestirici(Node):
    def __init__(self):
        super().__init__('konum_birlestirici')

        self.declare_parameter('integrate_rate_hz', 20.0)
        # CANLI TESTTE BULUNDU: rf2o'nun twist.linear.x isareti bu montajda
        # GERCEK ileri yonunun TERSI cikti (geri surunce RViz'de ileri
        # gidiyor gibi gorundu, ve tam tersi). Yon/donus (IMU kaynakli)
        # DOGRUYDU - sadece hiz isareti ters. Tek noktadan duzeltiliyor.
        self.declare_parameter('vx_sign', -1.0)
        self._vx_sign = self.get_parameter('vx_sign').value

        # --- GPS parametreleri (bkz. dosya basi "4) GPS" notu) ---
        self.declare_parameter('gps_anchor_min_fix_type', 5)  # RTK Float+
        self.declare_parameter('gps_anchor_samples_needed', 5)
        self.declare_parameter('gps_correct_min_fix_type', 5)  # RTK Float+
        self.declare_parameter('gps_correct_max_h_acc_m', 0.5)
        self.declare_parameter('gps_correct_max_age_sec', 3.0)
        self.declare_parameter('gps_correction_max_speed_mps', 0.05)
        self.declare_parameter('gps_heading_min_fix_type', 3)  # 3D fix yeter
        self.declare_parameter('gps_heading_min_speed_mps', 0.3)
        self.declare_parameter('gps_heading_samples_needed', 30)

        self._anchor_min_fix = self.get_parameter('gps_anchor_min_fix_type').value
        self._anchor_samples_needed = self.get_parameter('gps_anchor_samples_needed').value
        self._correct_min_fix = self.get_parameter('gps_correct_min_fix_type').value
        self._correct_max_h_acc = self.get_parameter('gps_correct_max_h_acc_m').value
        self._correct_max_age = self.get_parameter('gps_correct_max_age_sec').value
        self._correction_max_speed = self.get_parameter('gps_correction_max_speed_mps').value
        self._heading_min_fix = self.get_parameter('gps_heading_min_fix_type').value
        self._heading_min_speed = self.get_parameter('gps_heading_min_speed_mps').value
        self._heading_samples_needed = self.get_parameter('gps_heading_samples_needed').value

        self._son_imu_orientation = None
        self._son_imu_wz = 0.0
        self._son_vx = 0.0

        # dead-reckoning durumu (odom cercevesinde, baslangic = 0,0)
        self._x = 0.0
        self._y = 0.0
        self._last_tick_time = None

        # GPS durumu
        self._last_fix = None
        self._anchor = None  # (lat0, lon0) - kilitlenince sabit
        self._anchor_samples = []
        self._heading_offset = None  # radyan - kilitlenince /gps_heading_offset yayinlanir
        self._heading_samples = deque(maxlen=self._heading_samples_needed)
        self._gps_target = None  # (x, y) odom cercevesinde - en son iyi-kaliteli GPS hedefi
        self._gps_target_time = None

        self.create_subscription(Imu, '/imu/data', self._imu_cb, 10)
        self.create_subscription(Odometry, '/odom_rf2o', self._rf2o_cb, 10)

        if _MAVROS_MEVCUT:
            # NavSatFix: mavros varsayilan olarak BEST_EFFORT yayinliyor -
            # qos_profile_sensor_data ile eslesiyoruz (ayrica /fix koprusune
            # BAGIMLI DEGILIZ, bu node kendi QoS'unu kendi ayarliyor).
            self.create_subscription(
                NavSatFix, '/mavros/global_position/raw/fix', self._fix_cb,
                qos_profile_sensor_data)
            self.create_subscription(
                GPSRAW, '/mavros/gpsstatus/gps1/raw', self._gpsraw_cb, 10)
            self._anchor_pub = self.create_publisher(
                NavSatFix, '/gps_anchor',
                rclpy.qos.QoSProfile(
                    depth=1,
                    durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL))
            self._heading_pub = self.create_publisher(Float64, '/gps_heading_offset', 10)
        else:
            self.get_logger().warn(
                'mavros_msgs bulunamadi - GPS surunme duzeltmesi DEVRE DISI, '
                'sadece dead-reckoning calisiyor.')

        self._pub = self.create_publisher(Odometry, '/odom', 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        rate = self.get_parameter('integrate_rate_hz').value
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            'konum_birlestirici aktif: EKF YOK. Konum(x,y)=rf2o hizi + IMU '
            'yonu entegre edilerek (dead-reckoning) hesaplaniyor (ICP/odom_lsm '
            'ARTIK KULLANILMIYOR), yon+acisal_hiz=/imu/data, GPS(RTK) varsa '
            'sinirli surunme duzeltmesi uygulanir (bkz. dosya basi notu)'
        )

    def _imu_cb(self, msg: Imu):
        self._son_imu_orientation = msg.orientation
        self._son_imu_wz = msg.angular_velocity.z

    def _rf2o_cb(self, msg: Odometry):
        self._son_vx = self._vx_sign * msg.twist.twist.linear.x

    def _fix_cb(self, msg: NavSatFix):
        self._last_fix = msg

    def _gpsraw_cb(self, msg):
        if self._last_fix is None:
            return
        self._maybe_update_heading(msg)
        self._maybe_capture_anchor(msg)
        self._maybe_update_gps_target(msg)

    def _maybe_update_heading(self, msg):
        if self._son_imu_orientation is None:
            return
        if msg.fix_type < self._heading_min_fix:
            return
        if msg.cog >= 65535 or msg.vel >= 65535:
            return  # GPSRAW: 65535 = bilinmiyor
        vel_mps = msg.vel / 100.0
        if vel_mps < self._heading_min_speed:
            return  # dusuk hizda cog gurultulu/anlamsiz

        cog_rad = math.radians(msg.cog / 100.0)
        odom_yaw = _yaw_of(self._son_imu_orientation)
        # heading_offset turetme: bkz. dosya basi (4a) notu.
        sample = _normalize_angle(cog_rad + odom_yaw)
        self._heading_samples.append(sample)

        if len(self._heading_samples) < self._heading_samples_needed:
            return

        # dairesel ortalama (aci sarmasini dogru ele almak icin duz
        # aritmetik ortalama KULLANILMAZ - ör. 359 ve 1 derece ortalamasi
        # 180 degil 0 olmali).
        sin_sum = sum(math.sin(s) for s in self._heading_samples)
        cos_sum = sum(math.cos(s) for s in self._heading_samples)
        self._heading_offset = math.atan2(sin_sum, cos_sum)
        self._heading_pub.publish(Float64(data=self._heading_offset))

    def _maybe_capture_anchor(self, msg):
        if self._anchor is not None:
            return
        if msg.fix_type < self._anchor_min_fix:
            # kalite gecici dustu: yarim kalmis diziyi at, bastan basla
            # (tek bir kotu okumayla karisik bir anchor kilitlenmesin).
            self._anchor_samples = []
            return

        self._anchor_samples.append((self._last_fix.latitude, self._last_fix.longitude))
        if len(self._anchor_samples) < self._anchor_samples_needed:
            return

        lat0 = sum(s[0] for s in self._anchor_samples) / len(self._anchor_samples)
        lon0 = sum(s[1] for s in self._anchor_samples) / len(self._anchor_samples)
        self._anchor = (lat0, lon0)

        anchor_msg = NavSatFix()
        anchor_msg.header.stamp = self.get_clock().now().to_msg()
        anchor_msg.header.frame_id = 'odom'
        anchor_msg.latitude = lat0
        anchor_msg.longitude = lon0
        self._anchor_pub.publish(anchor_msg)
        self.get_logger().info(
            f'GPS anchor kilitlendi: lat={lat0:.7f} lon={lon0:.7f} '
            f'({len(self._anchor_samples)} ornek ortalamasi)')

    def _maybe_update_gps_target(self, msg):
        if self._anchor is None or self._heading_offset is None:
            return
        if msg.fix_type < self._correct_min_fix:
            return
        # h_acc (mm, MAVLink v2 GPS_RAW_INT) - 0 ise bu FC/donanimda
        # doldurulmuyor demektir, kalite kontrolunu atla (sadece fix_type'a
        # guven). *** Bu davranis canli testte dogrulanmali. ***
        if msg.h_acc > 0:
            h_acc_m = msg.h_acc / 1000.0
            if h_acc_m > self._correct_max_h_acc:
                return

        east, north = _latlon_to_enu(
            self._last_fix.latitude, self._last_fix.longitude, *self._anchor)
        gx, gy = _enu_to_odom(east, north, self._heading_offset)
        self._gps_target = (gx, gy)
        self._gps_target_time = self.get_clock().now()

    def _apply_gps_correction(self, now, dt):
        if self._gps_target is None or self._gps_target_time is None:
            return
        age = (now - self._gps_target_time).nanoseconds / 1e9
        if age > self._correct_max_age:
            return  # hedef bayatlamis, GPS kalitesi dustu - duzeltmeyi durdur

        ex = self._gps_target[0] - self._x
        ey = self._gps_target[1] - self._y
        err_mag = math.hypot(ex, ey)
        if err_mag < 1e-6:
            return
        # SINIRLI duzeltme: hedefe DOGRUDAN ATLAMA YOK, sabit bir "duzeltme
        # hizi" ile cekilir (bkz. dosya basi 4b notu - "teleport" onleme).
        step = min(err_mag, self._correction_max_speed * dt)
        self._x += ex / err_mag * step
        self._y += ey / err_mag * step

    def _tick(self):
        if self._son_imu_orientation is None:
            return  # henuz gercek yon bilgisi yok, entegrasyona baslama

        now = self.get_clock().now()
        if self._last_tick_time is None:
            self._last_tick_time = now
            return
        dt = (now - self._last_tick_time).nanoseconds / 1e9
        self._last_tick_time = now
        if dt <= 0.0 or dt > 0.5:
            # ilk tik / anormal buyuk bosluk (ör. node donmustu): bu adimi
            # entegre etme, sadece zaman damgasini guncelle.
            return

        yaw = _yaw_of(self._son_imu_orientation)
        self._x += self._son_vx * math.cos(yaw) * dt
        self._y += self._son_vx * math.sin(yaw) * dt

        if _MAVROS_MEVCUT:
            self._apply_gps_correction(now, dt)

        stamp = now.to_msg()

        cikis = Odometry()
        cikis.header.stamp = stamp
        cikis.header.frame_id = 'odom'
        cikis.child_frame_id = 'base_footprint'
        cikis.pose.pose.position.x = self._x
        cikis.pose.pose.position.y = self._y
        cikis.pose.pose.position.z = 0.0
        cikis.pose.pose.orientation = self._son_imu_orientation
        cikis.twist.twist.linear.x = self._son_vx
        cikis.twist.twist.angular.z = self._son_imu_wz
        self._pub.publish(cikis)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = 'odom'
        tf_msg.child_frame_id = 'base_footprint'
        tf_msg.transform.translation.x = self._x
        tf_msg.transform.translation.y = self._y
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation = self._son_imu_orientation
        self._tf_broadcaster.sendTransform(tf_msg)


def main(args=None):
    rclpy.init(args=args)
    node = KonumBirlestirici()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
