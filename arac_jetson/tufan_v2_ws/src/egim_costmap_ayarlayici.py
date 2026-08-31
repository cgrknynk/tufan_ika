#!/usr/bin/env python3
"""Egim/rampa gecislerinde LIDAR'in zemini engel sanmasini onler (2026-08-31,
kullanici istegi ve tasarim karari).

SORUN: 2D LIDAR araca sabit monteli, taradigi duzlem govdeyle birlikte
egiliyor. Arac bir rampaya girip/cikarken (pitch degisirken), normalde ufka
kadar giden yatay tarama artik birkac metre ilerideki DUZ ZEMINE carpip geri
donuyor - costmap bunu katı bir engel saniyor (2D LIDAR govde-egimi ile
gercek engeli ayirt edemez). Bu, hem rampaya TIRMANIRKEN hem TEPEDE duzlugE
GECERKEN hem de INISTE olusabilir.

COZUM: sabit bir tabela-tetikli pencere yerine, DOGRUDAN FIZIKSEL SEBEBE
(pitch) bagli, otomatik bir mekanizma: /odom'daki (konum_birlestirici.py -
BNO055 montaj-duzeltmesi ZATEN uygulanmis, bkz. o dosyanin notu) pitch
acisi izlenir. |pitch| esigi asinca local+global costmap'in obstacle_layer
LIDAR gozlem kaynaginin (scan) obstacle_max_range/raytrace_max_range
degerleri GECICI olarak dusurulur (rampa/govde geometrisinin LIDAR'i
kandirdigi yakin mesafe disariya cikarilir); pitch tekrar duze donup bir
sure (histerezis - gecis anindaki titremede surekli acilip kapanmasin)
sabit kalinca ORIJINAL (baslangicta okunan, nav2_params.yaml'daki gercek)
degerlere geri donulur.

Bu mekanizma HEM CIKISTA HEM TEPEDE GECISTE HEM DE INISTE OTOMATIK calisir -
tabela_etap_yoneticisi.py'nin ayrica bir "rampa modu" tetiklemesine GEREK
YOK, sadece /ugv_goal ile normal ileri hedef gonderir, bu node arka planda
kendi isini yapar.

ONEMLI (rclpy deadlock onlemi): parametre servis cagrilari ASENKRON
(call_async + add_done_callback) yapilir - /odom callback'i icinden
spin_until_future_complete GIBI BLOKE EDEN bir cagri YAPILMAZ (tek
threadli executor'da bu, executor zaten o callback'i calistirdigi icin
future'in hic tamamlanamamasina / donume yol acar).

*** CANLI TESTTE DOGRULANMASI GEREKEN DEGERLER: pitch_esigi_derece
*** (varsayilan 8.0) ve dusuk_menzil_m (varsayilan 1.1 - 63cm LIDAR
*** yuksekligi + 25 derece rampa acisindan hesaplanan ~1.35m kesisim
*** mesafesinin altinda, guvenli pay icin). Arac tepede/duzlukte
*** GENUINE (gercek) bir engele bu dusuk menzil nedeniyle GEC tepki
*** verebilir - bu, rampa gecisinde LIDAR'in yanlis engel algilama
*** riskine karsi BILINCLI bir odun.
"""
import math

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import Parameter as ParamMsg
from rcl_interfaces.msg import ParameterValue, ParameterType
from rcl_interfaces.srv import GetParameters, SetParameters
from nav_msgs.msg import Odometry


def _pitch_of(q):
    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    sinp = max(-1.0, min(1.0, sinp))
    return math.asin(sinp)


# (costmap_dugum_ismi) - navigation_launch.py bunlari boyle adlandiriyor,
# hem servis ad-alani (namespace) hem dugum ismi ayni: /<ad>/<ad>/...
_COSTMAPLER = ['local_costmap', 'global_costmap']
_GOZLEM_KAYNAGI = 'scan'  # nav2_params.yaml: observation_sources: scan
_ALANLAR = ['obstacle_max_range', 'raytrace_max_range']


def _param_isimleri():
    return [f'obstacle_layer.{_GOZLEM_KAYNAGI}.{alan}' for alan in _ALANLAR]


def _set_params_msg(degerler_sozluk):
    return [
        ParamMsg(
            name=f'obstacle_layer.{_GOZLEM_KAYNAGI}.{alan}',
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                  double_value=float(degerler_sozluk[alan])),
        )
        for alan in _ALANLAR
    ]


class EgimCostmapAyarlayici(Node):
    def __init__(self):
        super().__init__('egim_costmap_ayarlayici')

        self.declare_parameter('pitch_esigi_derece', 8.0)
        self.declare_parameter('dusuk_menzil_m', 1.1)
        self.declare_parameter('histerezis_saniye', 2.0)

        self._pitch_esigi = math.radians(self.get_parameter('pitch_esigi_derece').value)
        self._dusuk_menzil = self.get_parameter('dusuk_menzil_m').value
        self._histerezis_s = self.get_parameter('histerezis_saniye').value

        self._orijinal_degerler = {}  # costmap_ad -> {alan: deger}
        self._okuma_bekliyor = set()  # okuma cevabi beklenen costmap'ler
        self._dusuk_modda = False
        self._duz_zeminden_beri_ns = None

        self._get_clients = {ad: self.create_client(GetParameters, f'/{ad}/{ad}/get_parameters')
                              for ad in _COSTMAPLER}
        self._set_clients = {ad: self.create_client(SetParameters, f'/{ad}/{ad}/set_parameters')
                              for ad in _COSTMAPLER}

        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        # Orijinal degerleri periyodik (asenkron) okumayi dener - costmap
        # dugumleri bu node'dan sonra/gecikmeli hazir olabilir.
        self.create_timer(2.0, self._orijinal_degerleri_dene)

        self.get_logger().info(
            f'egim_costmap_ayarlayici aktif: pitch_esigi='
            f'{self.get_parameter("pitch_esigi_derece").value} derece, '
            f'dusuk_menzil={self._dusuk_menzil}m (orijinal degerler asenkron '
            'olarak okunuyor)')

    def _orijinal_degerleri_dene(self):
        for ad in _COSTMAPLER:
            if ad in self._orijinal_degerler or ad in self._okuma_bekliyor:
                continue
            client = self._get_clients[ad]
            if not client.service_is_ready():
                continue
            self._okuma_bekliyor.add(ad)
            future = client.call_async(GetParameters.Request(names=_param_isimleri()))
            future.add_done_callback(lambda f, ad=ad: self._orijinal_deger_cevabi(ad, f))

    def _orijinal_deger_cevabi(self, ad, future):
        self._okuma_bekliyor.discard(ad)
        try:
            sonuc = future.result()
        except Exception as e:
            self.get_logger().warn(f'{ad} orijinal parametre okuma hatasi: {e} (tekrar denenecek)')
            return
        degerler = {alan: pv.double_value for alan, pv in zip(_ALANLAR, sonuc.values)}
        self._orijinal_degerler[ad] = degerler
        self.get_logger().info(f'{ad} orijinal LIDAR menzil degerleri kaydedildi: {degerler}')

    def _menzil_ayarla(self, ad, degerler_sozluk):
        client = self._set_clients[ad]
        if not client.service_is_ready():
            return
        client.call_async(SetParameters.Request(parameters=_set_params_msg(degerler_sozluk)))

    def _odom_cb(self, msg: Odometry):
        pitch = _pitch_of(msg.pose.pose.orientation)
        now_ns = self.get_clock().now().nanoseconds
        egimde = abs(pitch) > self._pitch_esigi

        if egimde:
            self._duz_zeminden_beri_ns = None
            if not self._dusuk_modda:
                if len(self._orijinal_degerler) < len(_COSTMAPLER):
                    return  # orijinal degerler henuz okunamadi, bu tick'i atla
                dusuk = {alan: self._dusuk_menzil for alan in _ALANLAR}
                for ad in _COSTMAPLER:
                    self._menzil_ayarla(ad, dusuk)
                self._dusuk_modda = True
                self.get_logger().warn(
                    f'⛰️ EGIM ALGILANDI (pitch={math.degrees(pitch):.1f} derece) - '
                    f'costmap LIDAR menzili {self._dusuk_menzil}m\'e dusuruldu.')
            return

        # duz zemin
        if self._dusuk_modda:
            if self._duz_zeminden_beri_ns is None:
                self._duz_zeminden_beri_ns = now_ns
            elapsed = (now_ns - self._duz_zeminden_beri_ns) / 1e9
            if elapsed >= self._histerezis_s:
                for ad in _COSTMAPLER:
                    orijinal = self._orijinal_degerler.get(ad)
                    if orijinal is not None:
                        self._menzil_ayarla(ad, orijinal)
                self._dusuk_modda = False
                self.get_logger().info(
                    '⛰️ Duz zemine donuldu - costmap LIDAR menzili ORIJINAL '
                    'degerlere geri yuklendi.')


def main(args=None):
    rclpy.init(args=args)
    node = EgimCostmapAyarlayici()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
