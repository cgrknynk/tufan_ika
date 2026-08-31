"""Manuel surus joystick Arduino'sunu okuyan seri port thread'i.

Fiziksel 2 eksenli analog joystick -> Arduino (arduino_joystick/
arduino_joystick.ino) -> USB seri -> bu thread -> telemetri_sistemi.py'nin
zaten var olan MANUEL surus/palet altyapisi (anlik_sol_pwm, anlik_sag_pwm,
0.8s guvenlik watchdog'u, /palet_hizlari yayini).

Bu thread SADECE donanimdan okuyup -1..1 araliginda ORAN hesaplar (PWM
DEGIL) - gercek PWM'e olcekleme (85-255 sinirlariyla, ayarlar ekranindan
canli degistirilebilir) telemetri_sistemi.py'deki tek merkezde yapilir ki
klavye ve joystick ayni sinirlara uysun. ROS2 yayinini da telemetri_sistemi.py
yapar (tek yayin kaynagi orada kalsin diye).
"""

import glob
import time
from collections import deque

try:
    import serial
except ImportError:
    serial = None

from PyQt5.QtCore import QThread, pyqtSignal

BAUDRATE = 115200
# 300kg'lik araç icin temkinli/hosgorulu merkez bolgesi - kucuk el
# titremesi/dokunma aracin ani hareket etmesine sebep olmasin.
ADC_OLU_BOLGE = 60      # merkez etrafinda titremeyi yok say
TERS_X = False          # joystick fiziksel yonu ters gelirse True yapin
TERS_Y = False
KALIBRASYON_ORNEK_SAYISI = 25  # ~0.5sn @ 50Hz

# SUREKLI OTO-MERKEZLEME: yayli/kendiliginden ortalanan joystick'lerde
# birakildiginda mekanik durak yuzunden okuma neredeyse hic kipirdamaz -
# elle tutarken (aktif surus) ise insan eli hep hafifce titrer. Bu farktan
# yararlanip, okuma yeterince UZUN SURE yeterince SABIT kalirsa bunu
# "gercekten birakildi" kabul edip merkezi CANLI olarak duzeltiyoruz - tek
# seferlik ilk kalibrasyon yanlis/kaymis olsa bile, joystick birakilir
# birakilmaz kendini duzeltiyor, elle yeniden baglanmaya gerek kalmiyor.
# GUVENLIK SINIRI: joystick TAM ILERI/GERI/YANA itilip MEKANIK DAYANAGA
# sabit tutulursa o konum da "kipirdamiyor" gibi gorunur - bu ASLA yeni
# merkez sanilmamali (300kg araç: surucu eli gevsetince ters yonde ani
# komut olusabilir). Bu yuzden sadece MEVCUT merkeze bu kadar YAKIN
# duzeltmeler kabul edilir; uzak (gercek tam itilme) asla kabul edilmez.
#
# ONEMLI: bu, ADC_OLU_BOLGE'den (60) BELIRGIN SEKILDE BUYUK olmali. Ikisi
# esit olursa (eski hata), duzeltilmesi GEREKEN her kayma (>60, aksi halde
# zaten sorun olusturmaz) otomatik olarak duzeltme siniri disina da dusuyor
# ve HICBIR ZAMAN duzeltilemiyor - "joystick hareketsizken aracin durmamasi"
# canli bildirilen hatanin gercek kok nedeni buydu. 150, gercekci elektriksel/
# mekanik kaymayi (birkac on birim) rahatca kapsarken, gercek TAM ITILME
# (merkezden ~400-500 birim uzakta) ile karistirilmasi icin hala bol bol
# pay birakiyor.
MAKS_MERKEZ_KAYMASI = 150
STABILITE_PENCERE = 15      # ~0.3sn @ 50Hz
STABILITE_TOLERANSI = 3     # bu pencere icindeki max-min fark bundan kucukse "sabit" say

# SON GUVENLIK AGI (2026-08-31, kaptan bildirimi uzerine): joystick FIZIKSEL
# OLARAK hareketsizken aracin SUREKLI hareket etmeye devam ettigi, acil durdurma
# + joystick'i biraz oynatmadan duzelmeyen bir durum canli bildirildi. Yukaridaki
# merkez/olu-bolge hesabinda HENUZ TESPIT EDILEMEMIS bir hata (donanim
# gurultusu/gevsek baglanti/baska bir yazilim kusuru) olsa BILE, bu kontrol
# TAMAMEN BAGIMSIZ ve KALIBRASYONA (canli merkez_x/merkez_y) HIC GUVENMEZ:
# ham ADC okumasi yeterince UZUN SURE (yaklasik 1 saniye) neredeyse hic
# kipirdamadiysa VE bu sabit deger, guc-acilisindaki ILK (asla degismeyen)
# kalibrasyon merkezine YAKINSA, o eksenin ciktisi HESAPLANAN DEGER NE
# OLURSA OLSUN sifira zorlanir.
#
# ONEMLI GUVENLIK AYRINTISI: "ILK kalibrasyon merkezine YAKIN" sarti KASITLI
# - sadece ham ADC'nin SABIT olup olmadigina bakilsaydi, suruculer joystick'i
# TAM ILERI'de birkac saniyeden uzun süre BASILI TUTTUGUNDA (duz devam etmek
# icin gayet normal bir kullanim) o konum da "sabit" gorunup YANLISLIKLA
# sifirlanirdi - bu, dur-kalk yapan, guvenilmez bir surus deneyimi yaratirdi.
# Ilk kalibrasyon merkezine yakinlik sarti, bu agin SADECE "merkeze yakin bir
# yerde takili kalma" (tam da bildirilen hata) durumunu yakalamasini, gercek
# TAM ITILME durumlarina hic dokunmamasini saglar.
GUVENLIK_SABIT_PENCERE = 50           # ~1.0sn @ 50Hz
GUVENLIK_SABIT_TOLERANSI = 5          # ADC birimi - STABILITE_TOLERANSI'ndan biraz gevsek (gurultu payi)
GUVENLIK_SABIT_MAKS_UZAKLIK = 220     # ilk kalibrasyon merkezinden bu kadar uzagi "merkeze yakin" sayar -
                                       # MAKS_MERKEZ_KAYMASI'ndan (150) biraz genis: canli merkez zaten o
                                       # kadar kayabildigi icin, ilk merkeze gore payin da en az o kadar
                                       # olmasi gerekir; gercek tam itilme (~400-500) ile hala net ayrisir.


def _port_bul():
    adaylar = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    return adaylar[0] if adaylar else None


class SurusJoystickThread(QThread):
    pwm_sinyali = pyqtSignal(float, float)   # sol_oran, sag_oran [-1, 1] - PWM DEGIL (surus/tank karisimi SONRASI)
    yon_sinyali = pyqtSignal(float, float)   # ham x, y [-1, 1] - karisim ONCESI (silah pan/tilt icin)
    baglanti_sinyali = pyqtSignal(bool)
    guvenlik_sinyali = pyqtSignal(str)       # son guvenlik agi devreye girdiginde log icin (bkz. run())

    def __init__(self, port=None):
        super().__init__()
        self._sabit_port = port
        self._calisiyor = True
        self._baglanti_durumu = False

    def _normalize(self, ham, merkez, ters):
        # Merkez potansiyometrenin gercek elektriksel orta noktasi olmayabilir
        # (kablolama/referansa gore degisir - bkz. _kalibre_et). Bu yuzden
        # +/- yonlerdeki kalan araligi sabit 512 yerine merkeze gore ayri
        # ayri olcekliyoruz, yoksa dar taraf cok erken doyar (satüre olur).
        fark = ham - merkez
        if abs(fark) < ADC_OLU_BOLGE:
            return 0.0
        if fark > 0:
            bolen = max(1023 - merkez, 1)
        else:
            bolen = max(merkez, 1)
        oran = max(min(fark / bolen, 1.0), -1.0)
        return -oran if ters else oran

    def _kalibre_et(self, ser):
        # Baglaninca joystick'e dokunulmadigi varsayimiyla ilk birkac
        # okumanin ortalamasini merkez (0,0) kabul ediyoruz.
        xs, ys = [], []
        ser.reset_input_buffer()
        while len(xs) < KALIBRASYON_ORNEK_SAYISI and self._calisiyor:
            satir = ser.readline().decode("utf-8", errors="ignore").strip()
            parcalar = satir.split(",")
            if len(parcalar) < 2:
                continue
            try:
                xs.append(int(parcalar[0]))
                ys.append(int(parcalar[1]))
            except ValueError:
                continue
        if not xs:
            return 512, 512
        return sum(xs) // len(xs), sum(ys) // len(ys)

    def _baglanti_durumunu_guncelle(self, durum):
        if durum != self._baglanti_durumu:
            self._baglanti_durumu = durum
            self.baglanti_sinyali.emit(durum)

    def run(self):
        if serial is None:
            self.baglanti_sinyali.emit(False)
            return
        while self._calisiyor:
            port = self._sabit_port or _port_bul()
            if port is None:
                self._baglanti_durumunu_guncelle(False)
                time.sleep(2.0)
                continue

            try:
                with serial.Serial(port, BAUDRATE, timeout=1.0) as ser:
                    time.sleep(2.0)  # Arduino reset/kendine gelme suresi
                    merkez_x, merkez_y = self._kalibre_et(ser)
                    # ILK kalibrasyon merkezi - guvenlik agi icin SABIT referans,
                    # canli merkez_x/merkez_y gibi HICBIR ZAMAN degismez (bkz.
                    # GUVENLIK_SABIT_MAKS_UZAKLIK notu).
                    ilk_merkez_x, ilk_merkez_y = merkez_x, merkez_y
                    self._baglanti_durumunu_guncelle(True)
                    pencere_x = deque(maxlen=STABILITE_PENCERE)
                    pencere_y = deque(maxlen=STABILITE_PENCERE)
                    guvenlik_pencere_x = deque(maxlen=GUVENLIK_SABIT_PENCERE)
                    guvenlik_pencere_y = deque(maxlen=GUVENLIK_SABIT_PENCERE)
                    son_guvenlik_uyari_x = False
                    son_guvenlik_uyari_y = False
                    while self._calisiyor:
                        satir = ser.readline().decode("utf-8", errors="ignore").strip()
                        if not satir:
                            continue
                        parcalar = satir.split(",")
                        if len(parcalar) < 2:
                            continue
                        try:
                            x_ham, y_ham = int(parcalar[0]), int(parcalar[1])
                        except ValueError:
                            continue

                        pencere_x.append(x_ham)
                        pencere_y.append(y_ham)
                        if len(pencere_x) == STABILITE_PENCERE:
                            # GUVENLIK: sadece MEVCUT merkeze YAKIN (kucuk
                            # kayma/ilk kalibrasyon hatasi) duzeltmeleri
                            # kabul et. Joystick tam ileri/geri/yana itilip
                            # MEKANIK DAYANAGA sabit tutulursa o konum da
                            # "sabit" gorunur - bunu asla yeni merkez SANMA,
                            # yoksa surucu eli gevsetince ters yonde ani
                            # komut olusur (300kg aracta tehlikeli).
                            #
                            # X ve Y EKSENLERI BAGIMSIZ kontrol edilir (eskiden
                            # ikisi birden sabit olmadikca hicbiri duzelmiyordu -
                            # surus sirasindaki titresim bir eksende surekli
                            # kucuk gurultu yaratirsa, digeri gercekten kaymis
                            # olsa bile HICBIR ZAMAN duzelemiyordu; "joystick
                            # hareketsizken aracin durmamasi" seklinde canli
                            # gozlenen bir guvenlik hatasiydi, cunku watchdog
                            # bunu yakalayamaz - joystick veri gondermeye DEVAM
                            # ediyor, sadece merkezi yanlis).
                            if max(pencere_x) - min(pencere_x) < STABILITE_TOLERANSI:
                                aday_x = sum(pencere_x) // len(pencere_x)
                                if abs(aday_x - merkez_x) <= MAKS_MERKEZ_KAYMASI:
                                    merkez_x = aday_x
                            if max(pencere_y) - min(pencere_y) < STABILITE_TOLERANSI:
                                aday_y = sum(pencere_y) // len(pencere_y)
                                if abs(aday_y - merkez_y) <= MAKS_MERKEZ_KAYMASI:
                                    merkez_y = aday_y

                        x = self._normalize(x_ham, merkez_x, TERS_X)
                        y = self._normalize(y_ham, merkez_y, TERS_Y)

                        # SON GUVENLIK AGI: canli merkez_x/merkez_y'ye HIC
                        # guvenmeden, ham ADC'nin (a) GERCEKTEN ~1sn kipirdamadigini
                        # VE (b) bu sabit deger ILK (hic degismeyen) kalibrasyon
                        # merkezine yakin oldugunu kontrol eder. Ikisi de doğruysa
                        # x/y ne hesaplanmis olursa olsun sifira zorlanir. (b) sarti
                        # olmadan, joystick'i TAM ILERI'de birkac saniye BASILI
                        # TUTMAK (gayet normal bir kullanim) da "sabit" sayilip
                        # yanlislikla sifirlanirdi - bkz. dosya basindaki not.
                        guvenlik_pencere_x.append(x_ham)
                        guvenlik_pencere_y.append(y_ham)
                        if len(guvenlik_pencere_x) == GUVENLIK_SABIT_PENCERE:
                            x_sabit = (max(guvenlik_pencere_x) - min(guvenlik_pencere_x)) < GUVENLIK_SABIT_TOLERANSI
                            x_merkeze_yakin = abs(x_ham - ilk_merkez_x) <= GUVENLIK_SABIT_MAKS_UZAKLIK
                            if x_sabit and x_merkeze_yakin and x != 0.0:
                                if not son_guvenlik_uyari_x:
                                    self.guvenlik_sinyali.emit(
                                        f"⚠️ GÜVENLİK AĞI: X ekseni ~1sn hareketsiz (merkeze yakın) "
                                        f"ama oran={x:.2f} hesaplanmıştı, ZORLA sıfırlandı "
                                        f"(ham={x_ham}, canlı merkez={merkez_x}, ilk merkez={ilk_merkez_x})")
                                    son_guvenlik_uyari_x = True
                                x = 0.0
                            else:
                                son_guvenlik_uyari_x = False
                        if len(guvenlik_pencere_y) == GUVENLIK_SABIT_PENCERE:
                            y_sabit = (max(guvenlik_pencere_y) - min(guvenlik_pencere_y)) < GUVENLIK_SABIT_TOLERANSI
                            y_merkeze_yakin = abs(y_ham - ilk_merkez_y) <= GUVENLIK_SABIT_MAKS_UZAKLIK
                            if y_sabit and y_merkeze_yakin and y != 0.0:
                                if not son_guvenlik_uyari_y:
                                    self.guvenlik_sinyali.emit(
                                        f"⚠️ GÜVENLİK AĞI: Y ekseni ~1sn hareketsiz (merkeze yakın) "
                                        f"ama oran={y:.2f} hesaplanmıştı, ZORLA sıfırlandı "
                                        f"(ham={y_ham}, canlı merkez={merkez_y}, ilk merkez={ilk_merkez_y})")
                                    son_guvenlik_uyari_y = True
                                y = 0.0
                            else:
                                son_guvenlik_uyari_y = False

                        sol_oran = max(min(y + x, 1.0), -1.0)
                        sag_oran = max(min(y - x, 1.0), -1.0)

                        self.pwm_sinyali.emit(sol_oran, sag_oran)
                        # Ham x/y (tank karisimi ONCESI) - silah pan/tilt kontrolu
                        # icin. Aracin surus kalibrasyonu (merkez, olu bolge,
                        # oto-merkezleme, guvenlik siniri) burada zaten uygulanmis
                        # oldugundan silah icin AYRICA kalibrasyon yapmaya gerek yok.
                        self.yon_sinyali.emit(x, y)
            except (serial.SerialException, OSError):
                self._baglanti_durumunu_guncelle(False)
                time.sleep(2.0)

    def durdur(self):
        self._calisiyor = False
        self.quit()
        self.wait()
