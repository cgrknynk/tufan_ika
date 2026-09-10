#!/usr/bin/env python3
"""Saglik bekcisi - "AYAKTA AMA SESSIZ" dugumleri tespit edip oldurur.

NEDEN VAR (2026-09-09, sahada bir gun boyunca yasandi):
Bu araçtaki arizalarin hicbiri COKME degildi - hepsi SESSIZ SUSMA idi:

  * LIDAR'in USB-seri adaptoru (CP2102N) TEK ACILISTA 6 KEZ koptu
    (dmesg: "status: -19"/ENODEV, "urb stopped: -32"). Her kopmada
    sllidar_node OLU dosya tanimlayicisini tutmaya devam etti: sureç
    ayakta, %CPU normal, HATA LOGU YOK, ama /scan_raw TAMAMEN sustu.
    Bir kere de ROS grafigine hic kaydolmadi (topic bile olusmadi).
  * Silah kamerasi (C922) USB bant genisligi yarisini kaybedince
    cap.read() SUREKLI False dondurdu - turret_node ayakta, log yok,
    goruntu yok (olcum: silah kamerasinin ag payi 19 KB/s = sifir).

Boyle bir durumda launch'in respawn'i DEVREYE GIRMEZ, cunku sureç
olmedi. Cozum iki parcali:
  1) launch'ta respawn=True  -> olen/oldurulen sureç geri gelir
  2) BU DUGUM               -> susan sureci OLDURUR, boylece (1) calisir
Ikisi birlikte "donma/kopma" arizalarini elle mudahale gerektirmeyen
bir kendini-toparlama dongusune cevirir.

TASARIM KURALLARI (hepsi sahada ogrenildi):
  * BASLANGIC PAYI: yigin ~60sn'de aciliyor (YOLO/TensorRT motorlari,
    Nav2 lifecycle). O sirada topic'ler hakli olarak sessiz - bu surede
    ASLA oldurme yapilmaz, yoksa bekci acilisi hic tamamlanmaz.
  * ILK VERIYI BEKLE: bir akis HIC gelmediyse (or. donanim hic takili
    degil) surekli oldurmek fayda getirmez, sadece log kirletir. Bu
    yuzden "once veri geldi, sonra kesildi" durumu ile "hic gelmedi"
    durumu AYRI ele alinir (ikincisinde de oldurulur ama COK daha uzun
    bir esikle ve tekrar araligiyla).
  * SOGUMA SURESI: her hedef icin oldurme sonrasi bekleme - respawn +
    yeniden acilis suresi kadar (LIDAR'da handshake ~5sn). Yoksa bekci
    surekli olduren bir dongude sikisir.
  * pkill DESENI KOSELI PARANTEZLI: bu projede pgrep/pkill'in KENDI
    komut satirini yakalamasi tuzagina BES KEZ dusuldu (pkill, arayuz
    butonu, iki teshis komutu, butonun "duzeltmesi"). Desen '[s]llidar'
    yazilir: regex GERCEK surecin 'sllidar' metnini yakalar, ama bu
    dugumun/komutun kendi metni '[s]llidar' oldugu icin KENDINI
    yakalamaz. Ayrica oldurmeden once eslesen PID'ler kendi PID'imiz
    ile karsilastirilir (ikinci emniyet).
"""
import json
import os
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class Izlenen:
    """Tek bir veri akisinin saglik durumu."""

    def __init__(self, ad, topic, esik_s, desen, hic_gelmedi_esik_s, soguma_s):
        self.ad = ad
        self.topic = topic
        self.esik_s = float(esik_s)
        self.desen = desen                    # pkill -f deseni (KOSELI PARANTEZLI)
        self.hic_gelmedi_esik_s = float(hic_gelmedi_esik_s)
        self.soguma_s = float(soguma_s)
        self.son_veri = None                  # None = hic veri gelmedi
        self.son_oldurme = 0.0
        self.oldurme_sayisi = 0
        self.mesaj_sayisi = 0


class SaglikBekcisi(Node):
    def __init__(self):
        super().__init__('saglik_bekcisi')

        self.declare_parameter('baslangic_payi_s', 75.0)
        self.declare_parameter('kontrol_araligi_s', 2.0)
        self.declare_parameter('etkin', True)
        self._baslangic_payi = float(self.get_parameter('baslangic_payi_s').value)
        self._etkin = bool(self.get_parameter('etkin').value)
        self._baslama_zamani = time.time()

        # LIDAR: sensor verisi BEST_EFFORT olabilir; sllidar RELIABLE
        # yayinliyor ama bekci HER IKISINI de gorebilsin diye BEST_EFFORT
        # abone olunur (BEST_EFFORT abone, RELIABLE yayinciyi okuyabilir;
        # tersi calismaz - bu uyumsuzluk bu projede daha once sessiz
        # veri kaybina yol acti).
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self._izlenenler = [
            # LIDAR ham verisi: adaptor koptugunda burasi susar.
            Izlenen('lidar', '/scan_raw', esik_s=6.0,
                    desen='[s]llidar_node',
                    hic_gelmedi_esik_s=45.0, soguma_s=25.0),
            # Filtre cikisi: /scan_raw akarken /scan susuyorsa filtre
            # dugumu takilmistir (9 Eylul'de bu da yasandi).
            Izlenen('scan_filtresi', '/scan', esik_s=8.0,
                    desen='[s]can_front_filter',
                    hic_gelmedi_esik_s=60.0, soguma_s=25.0),
        ]
        for iz in self._izlenenler:
            self.create_subscription(
                LaserScan, iz.topic,
                (lambda i: (lambda msg: self._veri_geldi(i)))(iz),
                sensor_qos)

        self._durum_pub = self.create_publisher(String, '/saglik_durumu', 10)
        self.create_timer(
            float(self.get_parameter('kontrol_araligi_s').value), self._kontrol)

        self.get_logger().info(
            'saglik_bekcisi aktif: %s izleniyor, baslangic payi %.0fsn '
            '(susan dugum OLDURULUR, launch respawn geri getirir)'
            % (', '.join(i.topic for i in self._izlenenler), self._baslangic_payi))

    def _veri_geldi(self, iz: Izlenen):
        iz.son_veri = time.time()
        iz.mesaj_sayisi += 1

    # ------------------------------------------------------------------
    def _kontrol(self):
        su_an = time.time()
        acilis_bitti = (su_an - self._baslama_zamani) >= self._baslangic_payi

        durum = {'t': int(su_an), 'acilis_bitti': acilis_bitti, 'akislar': {}}
        for iz in self._izlenenler:
            if iz.son_veri is None:
                # HIC VERI GELMEDI: sessizlik, BASLANGIC PAYININ BITTIGI
                # andan itibaren sayilir - dugum baslangicindan DEGIL.
                # Aksi halde pay (75sn) zaten hic_gelmedi_esik'ten (45sn)
                # buyuk oldugu icin esik pay dolar dolmaz asilmis olur ve
                # uzun esik ISLEVSIZ kalirdi: cihaz gercekten sokulmusse
                # bekci her soguma suresinde (25sn) bosuna oldururdu.
                sessiz_s = max(
                    0.0, su_an - (self._baslama_zamani + self._baslangic_payi))
                canli = False
            else:
                sessiz_s = su_an - iz.son_veri
                canli = sessiz_s <= iz.esik_s
            durum['akislar'][iz.ad] = {
                'canli': canli,
                'sessiz_s': round(sessiz_s, 1),
                'mesaj': iz.mesaj_sayisi,
                'oldurme': iz.oldurme_sayisi,
            }

            if not acilis_bitti or not self._etkin:
                continue
            esik = iz.esik_s if iz.son_veri is not None else iz.hic_gelmedi_esik_s
            if sessiz_s < esik:
                continue
            if (su_an - iz.son_oldurme) < iz.soguma_s:
                continue
            self._oldur(iz, sessiz_s)

        try:
            self._durum_pub.publish(String(data=json.dumps(durum)))
        except Exception:
            pass

    def _oldur(self, iz: Izlenen, sessiz_s):
        """Susan sureci SIGTERM ile sonlandirir; launch respawn geri getirir."""
        try:
            cikti = subprocess.run(['pgrep', '-f', iz.desen],
                                   capture_output=True, text=True, timeout=5)
            pidler = [p for p in cikti.stdout.split() if p.strip()]
        except Exception as e:
            self.get_logger().error('%s: pgrep basarisiz: %s' % (iz.ad, e))
            return

        # KENDIMIZI ASLA OLDURMEYELIM (koseli parantez zaten engelliyor,
        # bu ikinci emniyet - bkz. dosya basindaki pkill DESENI notu).
        kendi = str(os.getpid())
        pidler = [p for p in pidler if p != kendi]
        if not pidler:
            self.get_logger().warn(
                '%s: %s %.1fsn sessiz ama "%s" ile eslesen sureç YOK - '
                'zaten olmus, launch respawn bekleniyor.'
                % (iz.ad, iz.topic, sessiz_s, iz.desen))
            iz.son_oldurme = time.time()
            return

        for p in pidler:
            try:
                subprocess.run(['kill', '-TERM', p], timeout=5)
            except Exception as e:
                self.get_logger().error('%s: pid %s oldurulemedi: %s'
                                        % (iz.ad, p, e))
        iz.son_oldurme = time.time()
        iz.oldurme_sayisi += 1
        # Oldurdukten sonra sayaci sifirla: yeni sureç veriyi bastan
        # getirecek, aksi halde ayni sessizlik hemen tekrar tetiklenirdi.
        iz.son_veri = None
        self._baslama_zamani = time.time()   # respawn icin yeni pay
        self.get_logger().warn(
            'DONMA TESPIT EDILDI: %s (%s) %.1fsn sessiz -> pid %s SIGTERM '
            'ile sonlandirildi, launch yeniden baslatacak (%d. kez).'
            % (iz.ad, iz.topic, sessiz_s, ','.join(pidler), iz.oldurme_sayisi))


def main(args=None):
    rclpy.init(args=args)
    node = SaglikBekcisi()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
