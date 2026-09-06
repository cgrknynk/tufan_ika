#!/usr/bin/env python3
"""Tum gorev sekansini yoneten karar dugumu (2026-08-31, kullanici istegi;
2026-09-01 GUNCELLENDI - kullanici istegi: "'1' numarasini gorunce ileri
hareket baslasin, S donuslerini tamamlayacak sekilde hareket etsin";
2026-09-01 TEKRAR GUNCELLENDI - kullanici istegi: "arka kamerayi simdilik
kullanmayalim, sadece on kamera ve silah kamerasi acik olsun, Stop'u
gordugunde on kamera modeli kapansin ve silahin modeli acilsin ve silahin
kodu calissin" - Stop -> 15sn bekleyip devam etme davranisi KALDIRILDI,
ORIJINAL silah/turret fazi GERI GETIRILDI).

GUNCEL AKIS (2026-09-01):
  1) Etap takibi (risksiz, bilgi amacli): One..Eleven tabelalari gorulunce
     /guncel_etap (Int32) yayinlanir. EndOfEleven -> /parkur_durumu
     "TAMAMLANDI". DEGISMEDI.
  2) ETAP 1 (One) tabelasi ILK gorulunce -> /otonom_surus_aktif=True
     yayinlanir. serbest_yon_takipcisi.py bu sinyali dinleyip LIDAR'a
     gore en acik yone dogru surekli kisa-mesafeli hedefler gondermeye
     baslar (gap-following - bkz. o dosyanin basi, haritasiz/bilinmeyen
     parkurda S donuslerini REAKTIF olarak takip eder).
  3) ETAP 8 RAMPA TETIKLEMESI HALA GECICI DEVRE DISI (bu mesajda
     degismedi) - asagida _rampa_cikisini_baslat() cagrisi YORUM
     SATIRINDA, kod silinmedi, ileride geri acilabilir.
  4) STOP TABELASI (SADECE OTONOM modda, ARDIŞIK STOP_DOGRULAMA_ADEDI kare
     boyunca) -> GECICI DURDURMA + SILAH FAZI: mevcut /odom pozisyonu
     YENI /ugv_goal olarak yayinlanir (goal_manager_node aninda "hedefe
     ulasildi" sayip yumusakca durur) + /otonom_surus_aktif=False
     (serbest_yon_takipcisi durur, silah fazi boyunca yeniden hedef
     atmasin diye) + /tabela_model_aktif=False (on kamera modeli durur,
     KAMERA ACIK KALIR) + /turret_model_aktif=True + /silah_modu=OTONOM
     (turret_node.py varsayilan MANUEL'de baslar, otonom hedeflemenin
     baslamasi icin BU sart). KALICI acil-durdurma kilidi (/arac_komut
     EMERGENCY_STOP_CMD) KULLANILMAZ (degismedi, dosya sonu notu).
  5) /silah_hedef_vuruldu >= HEDEF_VURULDU_ESIGI (turret_node.py: 3 kere
     kilitlenip 5er saniye bekleme dongusu TAMAMLANDI) -> /turret_model_
     aktif=False, /silah_modu=MANUEL (silah devre disi), sonra INIS:
     rampaya cikarken kat edilen mesafe KADAR (rampa tetiklemesi devre
     disi oldugu icin bu bilgi genelde YOK - bu durumda sadece
     SON_ILERI_PAY_M sabit mesafesi kullanilir, guvenli fallback) ileri
     bir /ugv_goal gonderilir.

ARKA KAMERA (2026-09-01, kullanici istegi): tufan_mppi.launch.py'den
CIKARILDI - simdilik SADECE on kamera (tabela) ve silah (turret) kamerasi
acik. Bu dosyayla dogrudan ilgisi yok, launch dosyasinda not var.

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
ILERI_BASLATMA_ETABI = 1  # 'One' tabelasi -> serbest yon takibi baslasin
# KAYAR ENGEL (2026-09-06, kullanici istegi: "parkura kayar engel
# ekleyeceğim; ön kamerada 6. tabelayı tespit ettiğinde aracın kayar engel
# aşamasına geldiğini anlaması gerekiyor, araç engelden kaçıp geri
# dönmemeli, kayar engel açıldığında devam etmeli").
# 'Six' tabelasi gorulunce /kayar_engel_etabi=True yayinlanir;
# on_bosluk_nokta_atici.py bunu gorunce ONUNDEKI kapali engelden KACMAZ -
# yerinde bekler ve engel acilinca duz gecer (bkz. o dosyadaki
# KAYAR_ENGEL modu). Etap 6'yi GECEN bir tabela (7+) gorulunce sinyal
# kalkar.
KAYAR_ENGEL_ETABI = 6
# *** CANLI OLCUMDE BULUNDU (2026-09-06) ***: model, kamera parkurda
# degilken bile GUVEN_ESIGI'ni (0.6) asan SAHTE tabela tespitleri
# uretiyor - tek bir test sirasinda Etap 8 (0.77 / 0.80 / 0.71), Etap 2
# (0.76) ve Etap 1 (0.60) ard arda "goruldu". Kayar engel kararini TEK
# kareye baglamak tehlikeli: sahte bir "7+" tespiti asamayi ANINDA iptal
# eder ve arac tam da kacmamasi gereken anda engelden kacmaya calisirdi.
# Bu yuzden kayar engel durumu, AYNI sinifin ust uste bu kadar kare
# gorulmesini bekler (Stop tabelasindaki STOP_DOGRULAMA_ADEDI ile ayni
# desen). /guncel_etap yayini DEGISMEDI - o zaten "risksiz, bilgi amacli".
KAYAR_ENGEL_DOGRULAMA_ADEDI = 3
RAMPA_ETABI = 8  # *** GECICI DEVRE DISI (2026-09-01, kullanici istegi) ***
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
        self._rampa_baslangic_pozu = None  # (x, y) - etap 8 hedefi gonderilirken (su an tetiklenmiyor)

        self._son_odom = None

        self._etap_pub = self.create_publisher(Int32, '/guncel_etap', 10)
        self._parkur_pub = self.create_publisher(String, '/parkur_durumu', 10)
        self._goal_pub = self.create_publisher(PoseStamped, '/ugv_goal', 10)
        self._otonom_surus_pub = self.create_publisher(Bool, '/otonom_surus_aktif', 10)
        self._tabela_model_pub = self.create_publisher(Bool, '/tabela_model_aktif', 10)
        self._turret_model_pub = self.create_publisher(Bool, '/turret_model_aktif', 10)
        self._silah_modu_pub = self.create_publisher(String, '/silah_modu', 10)
        self._kayar_engel_pub = self.create_publisher(Bool, '/kayar_engel_etabi', 10)
        self._kayar_engel_aktif = False
        self._son_etap_adayi = None
        self._etap_aday_sayaci = 0
        # Durum PERIYODIK tekrarlanir: on_bosluk_nokta_atici.py sonradan
        # baslarsa ya da yeniden baslatilirsa tek seferlik bir yayini
        # KACIRIRDI ve kayar engelden kacmaya calisirdi. Bu projede ayni
        # ders /surus_modu ve panel baglanti durumunda da yasandi.
        self.create_timer(1.0, self._kayar_engel_durumunu_tekrarla)

        self.create_subscription(String, '/tabela_tespit', self._tespit_cb, 10)
        self.create_subscription(String, '/surus_modu', self._mod_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_subscription(Int32, '/silah_hedef_vuruldu', self._hedef_vuruldu_cb, 10)

        self.get_logger().info(
            'tabela_etap_yoneticisi aktif: /tabela_tespit dinleniyor - '
            f'etap {ILERI_BASLATMA_ETABI} -> serbest yon takibi (gap-following) '
            'baslar, Stop -> gecici dur + on kamera modeli kapanir + silah '
            'fazi baslar, hedef vuruldu -> silah kapanir + inis.'
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
            # KAYAR ENGEL KARARI icin ARDISIK DOGRULAMA (bkz. sabitteki not).
            if yeni_etap == self._son_etap_adayi:
                self._etap_aday_sayaci += 1
            else:
                self._son_etap_adayi = yeni_etap
                self._etap_aday_sayaci = 1
            if self._etap_aday_sayaci >= KAYAR_ENGEL_DOGRULAMA_ADEDI:
                self._kayar_engel_etabini_guncelle(yeni_etap)
            if yeni_etap != self._guncel_etap:
                onceki_etap = self._guncel_etap
                self._guncel_etap = yeni_etap
                self._etap_pub.publish(Int32(data=yeni_etap))
                self.get_logger().info(f'🏁 ETAP {yeni_etap} tabelasi algilandi (güven={guven:.2f})')

                if yeni_etap == ILERI_BASLATMA_ETABI and onceki_etap != ILERI_BASLATMA_ETABI:
                    self._otonom_surus_pub.publish(Bool(data=True))
                    self.get_logger().info(
                        '🧭 Etap 1 tabelasi görüldü - serbest yön takibi (gap-following) BAŞLADI.')
                # RAMPA TETIKLEMESI GECICI DEVRE DISI (kullanici istegi, 2026-09-01):
                # if yeni_etap == RAMPA_ETABI and onceki_etap != RAMPA_ETABI:
                #     self._rampa_cikisini_baslat()
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

    def _kayar_engel_etabini_guncelle(self, etap):
        """Etap 6 -> kayar engel asamasi ACIK; 7 ve sonrasi -> KAPALI.

        Etap numarasi GERI gitmez (tabelalar sirayla goruluyor), bu yuzden
        'etap > 6' testi asamanin bittigini guvenle gosterir."""
        yeni = (etap == KAYAR_ENGEL_ETABI)
        if etap > KAYAR_ENGEL_ETABI:
            yeni = False
        if yeni == self._kayar_engel_aktif:
            return
        self._kayar_engel_aktif = yeni
        self._kayar_engel_pub.publish(Bool(data=yeni))
        if yeni:
            self.get_logger().warn(
                '🚧 KAYAR ENGEL ASAMASI BASLADI (etap 6 tabelasi) - arac '
                'onundeki kapali engelden KACMAYACAK, yerinde bekleyip '
                'engel acilinca duz gececek.')
        else:
            self.get_logger().info(
                '🚧 Kayar engel asamasi bitti (etap %d goruldu).' % etap)

    def _kayar_engel_durumunu_tekrarla(self):
        try:
            self._kayar_engel_pub.publish(Bool(data=bool(self._kayar_engel_aktif)))
        except Exception:
            pass

    def _rampa_cikisini_baslat(self):
        # GECICI DEVRE DISI - bkz. dosya basi notu. Kod korunuyor, cagrilmiyor.
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
        self._otonom_surus_pub.publish(Bool(data=False))
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
                'Rampa cikis mesafesi bilinmiyor (rampa tetiklemesi devre disi) - sadece '
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
