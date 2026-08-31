#!/usr/bin/env python3
"""Tum gorev sekansini yoneten karar dugumu (2026-08-31, kullanici istegi).

AKIS:
  1) Etap takibi (risksiz, bilgi amacli): One..Eleven tabelalari gorulunce
     /guncel_etap (Int32) yayinlanir. EndOfEleven -> /parkur_durumu
     "TAMAMLANDI".
  2) ETAP 8 (kullanicinin DUZELTMESI - ilk mesajda "10" denmisti, sonra
     "8 olacak" diye duzeltildi) -> DIK RAMPAYA CIKIS: guncel pozisyon+
     yon (/odom) okunur, RAMPA_HEDEF_MESAFE_M (varsayilan 20m - tam rampa+
     duzluk mesafesi bilinmedigi icin BOL tutuldu, Stop tabelasi zaten
     erken kesecek) kadar ileri bir /ugv_goal gonderilir. LIDAR'in rampada
     zemini engel sanma sorunu bu dugumun degil, egim_costmap_ayarlayici.py
     node'unun isi (pitch-tabanli, otomatik - bkz. o dosyanin basi).
  3) STOP TABELASI (SADECE OTONOM modda, ARDIŞIK STOP_DOGRULAMA_ADEDI kare
     boyunca) -> GECICI DURDURMA: mevcut /odom pozisyonu YENI /ugv_goal
     olarak yayinlanir - goal_manager_node.py bunu ANINDA "hedefe ulasildi"
     sayip yumusakca durur (KALICI acil-durdurma kilidi - /arac_komut
     EMERGENCY_STOP_CMD - KULLANILMAZ, kullanicinin acik istegi: "otomatik
     devam etsin", o kilit operator "DEVAM ET" basmadan asla acilmaz).
     Ayni anda: /tabela_model_aktif=False (on kamera modeli durur, kamera
     ACIK kalir), /turret_model_aktif=True (silah kamerasi/izleme baslar),
     /silah_modu=OTONOM (turret_node.py varsayilan MANUEL'de baslar,
     otonom hedeflemenin baslamasi icin BU sart).
  4) /silah_hedef_vuruldu >= HEDEF_VURULDU_ESIGI (turret_node.py: 3 kere
     kilitlenip 5er saniye bekleme dongusu TAMAMLANDI, kullanicinin acik
     istegi) -> /turret_model_aktif=False, /silah_modu=MANUEL (silah
     devre disi), sonra INIS: rampaya cikarken kat edilen mesafe KADAR
     (kullanicinin secimi: "cikista kullanilan mesafeyle ayni - simetrik
     rampa") + 5m (kullanicinin sabit istegi) ileri bir /ugv_goal
     gonderilir. Bu hedef DOGAL olarak (goal_manager SUCCESS) tamamlanir,
     ayrica bir durdurma tetiklemeye gerek yok - MPPI/velocity_smoother
     zaten hedefte yumusakca durur.

Model siniflari (tabela_ana.engine/Tabela_ana.pt, canli sorgulandi):
One..Eleven (etap numarasi tabelalari), EndOfEleven (parkur/etap 11 bitis
tabelasi), Stop.
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Bool
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

SAYI_ETAP_HARITASI = {
    'One': 1, 'Two': 2, 'Three': 3, 'Four': 4, 'Five': 5, 'Six': 6,
    'Seven': 7, 'Eight': 8, 'Nine': 9, 'Ten': 10, 'Eleven': 11,
}

GUVEN_ESIGI = 0.6  # bu altindaki tespitler yok sayilir (gurultu/kararsizlik)
STOP_DOGRULAMA_ADEDI = 3  # ust uste bu kadar kare Stop gormeden TETIKLENMEZ
RAMPA_ETABI = 8  # *** kullanicinin duzeltmesi: ilk mesajda 10 (Ten) denmisti ***
RAMPA_HEDEF_MESAFE_M = 20.0  # tam mesafe bilinmedigi icin bol tutuldu
SON_ILERI_PAY_M = 5.0  # inis sonrasi sabit ek mesafe (kullanici istegi)
HEDEF_VURULDU_ESIGI = 3  # turret_node.py ile TUTARLI olmali


def _yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class TabelaEtapYoneticisi(Node):
    def __init__(self):
        super().__init__('tabela_etap_yoneticisi')

        self._guncel_etap = None
        self._surus_modu = 'MANUEL'
        self._ardisik_stop_sayaci = 0
        self._stop_tetiklendi = False
        self._rampa_baslangic_pozu = None  # (x, y) - etap 8 hedefi gonderilirken

        self._son_odom = None

        self._etap_pub = self.create_publisher(Int32, '/guncel_etap', 10)
        self._parkur_pub = self.create_publisher(String, '/parkur_durumu', 10)
        self._goal_pub = self.create_publisher(PoseStamped, '/ugv_goal', 10)
        self._tabela_model_pub = self.create_publisher(Bool, '/tabela_model_aktif', 10)
        self._turret_model_pub = self.create_publisher(Bool, '/turret_model_aktif', 10)
        self._silah_modu_pub = self.create_publisher(String, '/silah_modu', 10)

        self.create_subscription(String, '/tabela_tespit', self._tespit_cb, 10)
        self.create_subscription(String, '/surus_modu', self._mod_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(Int32, '/silah_hedef_vuruldu', self._hedef_vuruldu_cb, 10)

        self.get_logger().info(
            'tabela_etap_yoneticisi aktif: /tabela_tespit dinleniyor - '
            f'etap {RAMPA_ETABI} -> rampa cikisi, Stop -> gecici dur + silah '
            'fazi, hedef vuruldu -> inis + son ilerleme.'
        )

    def _mod_cb(self, msg):
        self._surus_modu = msg.data
        if self._surus_modu != 'OTONOM':
            self._ardisik_stop_sayaci = 0
            self._stop_tetiklendi = False

    def _odom_cb(self, msg: Odometry):
        self._son_odom = msg

    def _guncel_poz(self):
        if self._son_odom is None:
            return None
        p = self._son_odom.pose.pose.position
        yaw = _yaw_of(self._son_odom.pose.pose.orientation)
        return p.x, p.y, yaw

    def _relatif_hedef_gonder(self, mesafe_m):
        """Guncel pozisyondan mesafe_m kadar aracin o anki yonune ileri bir
        /ugv_goal yayinlar. Basarili olursa (x0, y0) baslangic pozisyonunu
        doner (mesafe takibi icin), olmazsa None."""
        poz = self._guncel_poz()
        if poz is None:
            self.get_logger().error('/odom henuz gelmedi, relatif hedef gonderilemiyor!')
            return None
        x, y, yaw = poz
        hedef_x = x + mesafe_m * math.cos(yaw)
        hedef_y = y + mesafe_m * math.sin(yaw)
        goal = PoseStamped()
        goal.header.frame_id = 'odom'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = hedef_x
        goal.pose.position.y = hedef_y
        goal.pose.orientation.w = 1.0
        self._goal_pub.publish(goal)
        self.get_logger().info(
            f'Relatif hedef gonderildi: +{mesafe_m:.1f}m -> x={hedef_x:.2f} y={hedef_y:.2f}')
        return (x, y)

    def _gecici_dur(self):
        """Mevcut pozisyonu YENI hedef olarak yayinlar - goal_manager_node
        bunu aninda 'hedefe ulasildi' sayip yumusakca durur. KALICI
        acil-durdurma kilidini KULLANMAZ (bkz. dosya basi notu)."""
        poz = self._guncel_poz()
        if poz is None:
            self.get_logger().error('/odom henuz gelmedi, gecici durdurma gonderilemiyor!')
            return
        x, y, _ = poz
        goal = PoseStamped()
        goal.header.frame_id = 'odom'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0
        self._goal_pub.publish(goal)
        self.get_logger().info(f'Gecici durdurma: hedef = guncel pozisyon (x={x:.2f} y={y:.2f})')

    def _tespit_cb(self, msg):
        veri = msg.data
        if veri == 'YOK' or ':' not in veri:
            self._ardisik_stop_sayaci = 0
            return

        cls_adi, guven_str = veri.rsplit(':', 1)
        try:
            guven = float(guven_str)
        except ValueError:
            return
        if guven < GUVEN_ESIGI:
            self._ardisik_stop_sayaci = 0
            return

        if cls_adi in SAYI_ETAP_HARITASI:
            self._ardisik_stop_sayaci = 0
            yeni_etap = SAYI_ETAP_HARITASI[cls_adi]
            if yeni_etap != self._guncel_etap:
                onceki_etap = self._guncel_etap
                self._guncel_etap = yeni_etap
                self._etap_pub.publish(Int32(data=yeni_etap))
                self.get_logger().info(f'🏁 ETAP {yeni_etap} tabelasi algilandi (güven={guven:.2f})')
                if yeni_etap == RAMPA_ETABI and onceki_etap != RAMPA_ETABI:
                    self._rampa_cikisini_baslat()
            return

        if cls_adi == 'EndOfEleven':
            self._ardisik_stop_sayaci = 0
            self._parkur_pub.publish(String(data='TAMAMLANDI'))
            self.get_logger().info(f'🏁 PARKUR TAMAMLANDI tabelasi algilandi (güven={guven:.2f})')
            return

        if cls_adi == 'Stop':
            self._stop_tespiti_isle(guven)
            return

        # Bilinmeyen/ilgisiz sinif - Stop dogrulama zincirini bozar
        self._ardisik_stop_sayaci = 0

    def _rampa_cikisini_baslat(self):
        self._rampa_baslangic_pozu = self._relatif_hedef_gonder(RAMPA_HEDEF_MESAFE_M)
        if self._rampa_baslangic_pozu is None:
            self.get_logger().error(
                f'Etap {RAMPA_ETABI} algilandi ama rampa hedefi gonderilemedi (/odom yok)!')

    def _stop_tespiti_isle(self, guven):
        if self._surus_modu != 'OTONOM':
            return
        if self._stop_tetiklendi:
            return

        self._ardisik_stop_sayaci += 1
        self.get_logger().info(
            f'🛑 Stop tabelasi tespiti {self._ardisik_stop_sayaci}/{STOP_DOGRULAMA_ADEDI} (güven={guven:.2f})'
        )
        if self._ardisik_stop_sayaci < STOP_DOGRULAMA_ADEDI:
            return

        self._stop_tetiklendi = True
        self.get_logger().warn('🛑 STOP TABELASI DOĞRULANDI - araç geçici durduruluyor, silah fazına geçiliyor.')

        self._gecici_dur()
        self._tabela_model_pub.publish(Bool(data=False))
        self._turret_model_pub.publish(Bool(data=True))
        self._silah_modu_pub.publish(String(data='OTONOM'))
        self.get_logger().info('🔫 Silah fazı başladı: ön kamera modeli durduruldu, turret AKTİF/OTONOM.')

    def _hedef_vuruldu_cb(self, msg: Int32):
        if msg.data < HEDEF_VURULDU_ESIGI:
            return  # henuz 3 basarili kilit tamamlanmadi

        self.get_logger().warn('✅ HEDEF VURULDU - silah devre dışı bırakılıyor, iniş başlıyor.')
        self._turret_model_pub.publish(Bool(data=False))
        self._silah_modu_pub.publish(String(data='MANUEL'))

        inis_mesafesi = SON_ILERI_PAY_M
        if self._rampa_baslangic_pozu is not None and self._son_odom is not None:
            x0, y0 = self._rampa_baslangic_pozu
            x1 = self._son_odom.pose.pose.position.x
            y1 = self._son_odom.pose.pose.position.y
            kat_edilen = math.hypot(x1 - x0, y1 - y0)
            inis_mesafesi = kat_edilen + SON_ILERI_PAY_M
            self.get_logger().info(
                f'Rampa cikisinda kat edilen mesafe: {kat_edilen:.1f}m -> '
                f'inis+son ilerleme hedefi: {inis_mesafesi:.1f}m')
        else:
            self.get_logger().warn(
                'Rampa cikis mesafesi bilinmiyor (kayit yok) - sadece '
                f'{SON_ILERI_PAY_M}m sabit ilerleniyor.')

        self._relatif_hedef_gonder(inis_mesafesi)


def main(args=None):
    rclpy.init(args=args)
    node = TabelaEtapYoneticisi()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
