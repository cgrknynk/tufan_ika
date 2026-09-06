"""Yer istasyonu fiziksel kontrol panelini okuyan seri port thread'i.

Arayuz monitoru (dokunmatik ekran) yer istasyonuna monte edildikten sonra
yanina fiziksel bir kontrol paneli baglandi. HEPSI TEK bir Arduino'ya bagli
(bkz. arduino_kontrol_paneli/arduino_kontrol_paneli.ino):

    Sol joystick   -> A3, A4   -> araci sur   (tank karisimi SONRASI oran)
    Sag joystick   -> A0, A1   -> silah/turret pan-tilt (ham oran)
    Potansiyometre -> A5       -> PWM ust siniri (85..255) CANLI
    Sol toggle sw  -> D4       -> GND'ye alininca ACIL STOP
    Sag toggle sw  -> D5       -> GND'ye alininca SILAH ATES
    Sol buton      -> D9       -> Manuel <-> Otonom
    Sag buton      -> D10      -> Farlar ac/kapa

Bu thread SADECE donanimdan okur, kalibre eder ve -1..1 ORAN / kenar
olaylari uretir. PWM'e olcekleme, tank karisimi ROS2 yayinlari
telemetri_sistemi.py'deki TEK merkezde yapilir (klavye ile ayni sinirlara
uysun ve tek yayin kaynagi kalsin diye).

--------------------------------------------------------------------------
ONEMLI: Bu dosya, ARTIK KULLANILMAYAN surus_joystick_sistemi.py'nin yerini
alir. Oradaki TUM guvenlik mantigi (canli bildirilen "joystick hareketsizken
aracin durmamasi" hatasinin kok nedeni + son guvenlik agi) buraya BIREBIR
tasindi - tek eksenlik _EksenDurumu sinifina paketlendi ve her iki joystick'in
dort ekseni icin ayri ayri uygulaniyor.
--------------------------------------------------------------------------
"""

import glob
import json
import os
import time
from collections import deque

try:
    import serial
except ImportError:
    serial = None

from PyQt5.QtCore import QThread, pyqtSignal

BAUDRATE = 115200

# ==========================================================================
# EKSEN YERLESIMI / YON YAPILANDIRMASI
# --------------------------------------------------------------------------
# Joystick'lerin hangi pini X (donus/pan) hangisi Y (ileri-geri/tilt) ve
# yonlerinin ters olup olmadigi FIZIKSEL MONTAJA gore degisir - bilinmiyor.
# Arayuz > Ayarlar > "PANEL TESTI" dialogu ile her ekseni tek tek oynatip
# hangi ham kanalin degistigini ve yonunu gozleyin, sonra buradaki
# sabitleri ayarlayin. (Sag joystick sadece silahi kontrol ettiginden yanlis
# ayarlanmasi 300kg araci HAREKET ETTIRMEZ - once sol joystick'i dogrulayin.)
#
# DUZELTME (2026-09-04, kullanici istegi: "kalibrasyon ayarini arayuze
# ekleyebilir misin") - eskiden bu sabitler SADECE koddan elle
# degistirilebiliyordu (her degisiklik icin dosya duzenleyip arayuzu
# yeniden baslatmak gerekiyordu). Artik varsayilan olarak asagidaki
# degerler kullanilir AMA gercek deger ayarlari_yukle() ile diskteki JSON
# dosyasindan (varsa) okunur - "PANEL TESTI" dialogundaki yeni kalibrasyon
# kontrolleri buraya YAZAR, KontrolPaneliThread de her (yeniden)baglantida
# TAZE okur (bkz. run()) - kod duzenlemeye/yeniden baslatmaya gerek kalmaz.
# ==========================================================================
AYAR_DOSYASI = os.path.join(os.path.dirname(os.path.abspath(__file__)), "kontrol_paneli_ayarlari.json")

_VARSAYILAN_AYARLAR = {
    "sol_x_pin": "A3",       # sol joystick'in DONUS ekseni hangi pinde: "A3" veya "A4"
    "sol_ters_x": False,     # sola itince araç saga donuyorsa True
    "sol_ters_y": False,     # ileri itince araç geri gidiyorsa True
    "sag_x_pin": "A0",       # sag joystick'in PAN ekseni hangi pinde: "A0" veya "A1"
    "sag_ters_x": False,
    "sag_ters_y": False,
    # Toggle switch "aktif" yonu. Varsayilan: GND'ye alininca (LOW) aktif -
    # kullanici "GND'ye aldigimda acil stop / ates" dedi. Anahtar ters
    # kabloluysa (dinlenme konumu GND) True yapin.
    "sol_toggle_ters": False,
    "sag_toggle_ters": False,
}


def ayarlari_yukle():
    """Diskteki JSON'dan kalibrasyon ayarlarini okur - dosya yoksa/bozuksa
    yukaridaki varsayilanlara sessizce duser (ilk kurulumda/hicbir sey
    kaydedilmemisken bile calismaya devam etsin diye)."""
    ayarlar = dict(_VARSAYILAN_AYARLAR)
    if os.path.exists(AYAR_DOSYASI):
        try:
            with open(AYAR_DOSYASI, "r", encoding="utf-8") as f:
                kayitli = json.load(f)
            ayarlar.update({k: v for k, v in kayitli.items() if k in _VARSAYILAN_AYARLAR})
        except (OSError, ValueError):
            pass
    return ayarlar


def ayarlari_kaydet(ayarlar):
    """PANEL TESTI dialogundaki 'KAYDET VE UYGULA' butonundan cagrilir."""
    tam = dict(_VARSAYILAN_AYARLAR)
    tam.update({k: v for k, v in ayarlar.items() if k in _VARSAYILAN_AYARLAR})
    with open(AYAR_DOSYASI, "w", encoding="utf-8") as f:
        json.dump(tam, f, indent=2, ensure_ascii=False)
    return tam

# ==========================================================================
# KALIBRASYON / GUVENLIK SABITLERI  (surus_joystick_sistemi.py'den birebir)
# Degistirmeden once o dosyadaki uzun aciklamalari okuyun - her biri canli
# bildirilen bir guvenlik olayinin sonucu.
# ==========================================================================
ADC_OLU_BOLGE = 60                    # merkez etrafinda titremeyi yok say
KALIBRASYON_ORNEK_SAYISI = 25         # ~0.5sn @ 50Hz

# DUZELTME (2026-09-04, kullanici: "joystick bazen gidip geliyor") - CANLI
# MANTIK TESTINDE DOGRULANDI: MAKS_MERKEZ_KAYMASI=150 ve ESKI
# GUVENLIK_SABIT_MAKS_UZAKLIK=220 o kadar genisti ki joystick'i SABIT bir
# ORTA/KISMI konumda (ör. merkezden 80-219 birim - sabit hiz icin dogal bir
# surus hareketi!) sadece ~0.3-1sn tutmak bile YETERLI oluyordu: 150'ye
# kadar olan konumlar oto-merkezleme tarafindan "yeni merkez" sayilip oran
# SIFIRA CEKILIYORDU, 150-220 arasi ise SON GUVENLIK AGI tarafindan
# ZORLA SIFIRLANIYORDU - pratikte NEREDEYSE HER surekli kismi gaz komutu
# birkac saniye icinde sifira dusuyor, sonra pozisyon en ufak kiprissa
# (dogal el titremesi) geri geliyordu: "gidip geliyor" TAM OLARAK BUDUR.
# Her iki esik de artik SADECE gercek MERKEZ/RAHAT konumuna yakin kucuk
# sapmalari (deadzone'un hemen ustu) telafi edecek sekilde SIKILASTIRILDI -
# gercek bir surus komutunu (deadzone disindaki HERHANGI bir kasitli
# itis) ARTIK YOK SAYMIYOR. Orijinal amac (kalibrasyonsuz/kablo arizali
# joystick UZAK bir sabit degerde takiliysa aracin durmasi) hala calisiyor
# CUNKU boyle bir ariza tipik olarak UC deger (0'a veya 1023'e yakin)
# uretir, bu yeni dar bantla da kesin yakalanir.
MAKS_MERKEZ_KAYMASI = 90              # canli oto-merkezlemede kabul edilen max kayma
STABILITE_PENCERE = 15               # ~0.3sn @ 50Hz
STABILITE_TOLERANSI = 3              # bu pencerede max-min bundan kucukse "sabit"

# SON GUVENLIK AGI - kalibrasyona HIC guvenmez (bkz. surus_joystick_sistemi.py)
GUVENLIK_SABIT_PENCERE = 50           # ~1.0sn @ 50Hz
GUVENLIK_SABIT_TOLERANSI = 5
GUVENLIK_SABIT_MAKS_UZAKLIK = 90

# ==========================================================================
# POTANSIYOMETRE -> PWM UST SINIRI
# ==========================================================================
PWM_ALT = 85
PWM_UST = 255
POT_EMA_ALFA = 0.25                   # dusuk = daha cok yumusatma (gurultu)
POT_MIN_DEGISIM = 2                   # PWM biriminde: bundan kucuk oynama yayinlanmaz
POT_MIN_ARALIK_S = 0.12              # ardisik yayinlar arasi min sure

# Digital debounce: 50Hz'de 2 ornek = ~40ms
DEBOUNCE_ORNEK = 2

# ==========================================================================
# YER BATARYASI (2026-09-05, kullanici istegi: "yer istasyonunu besleyen
# 22.2V (6S) LiPo'nun sarjini yer sarj gostergesine cekelim") - A2 pinine
# GERILIM BOLUCU uzerinden baglanir (bkz. arduino_kontrol_paneli.ino basi
# - R1=100k ust, R2=22k alt). Buradaki degerler firmware'deki ile AYNI
# OLMALI, biri degisirse digeri de guncellenmeli.
# ==========================================================================
YER_BAT_R1_OHM = 100000.0
YER_BAT_R2_OHM = 22000.0
YER_BAT_ADC_REF_V = 5.0               # Arduino UNO analog referans gerilimi
YER_BAT_HUCRE_SAYISI = 6              # 22.2V nominal = 6S LiPo (3.7V x 6)
YER_BAT_HUCRE_BOS_V = 3.3             # %0 kabul edilen hucre gerilimi (guvenli deşarj siniri)
YER_BAT_HUCRE_DOLU_V = 4.2            # %100 kabul edilen hucre gerilimi (tam sarj)
YER_BAT_EMA_ALFA = 0.1                # pot'tan daha yavas yumusatma - sarj hizli degismez
YER_BAT_MIN_ARALIK_S = 2.0            # ardisik yayinlar arasi min sure (yavas degisen deger, GUI'yi bosuna mesgul etmesin)


def _port_bul():
    adaylar = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    return adaylar[0] if adaylar else None


def _kirp(deger, alt, ust):
    return max(alt, min(ust, deger))


def _yer_bat_yuzde_hesapla(ham_adc):
    """Ham A2 ADC'sini (gerilim bolucu uzerinden) -> (batarya_voltaji, yuzde).

    bkz. dosya basi 'YER BATARYASI' notu ve arduino_kontrol_paneli.ino'daki
    AYNI gerilim bolucu aciklamasi - degerler ORADAKI ile TUTARLI olmali.
    """
    pin_v = ham_adc * (YER_BAT_ADC_REF_V / 1023.0)
    bat_v = pin_v * (YER_BAT_R1_OHM + YER_BAT_R2_OHM) / YER_BAT_R2_OHM
    hucre_v = bat_v / YER_BAT_HUCRE_SAYISI
    yuzde = (hucre_v - YER_BAT_HUCRE_BOS_V) / (YER_BAT_HUCRE_DOLU_V - YER_BAT_HUCRE_BOS_V) * 100.0
    yuzde = _kirp(yuzde, 0.0, 100.0)
    return bat_v, yuzde


class _EksenDurumu:
    """Tek analog eksen icin kalibrasyon + guvenlik mantigi.

    surus_joystick_sistemi.py'deki _normalize + canli oto-merkezleme +
    son guvenlik agi tek eksen icin buraya paketlendi. X ve Y eksenleri
    BIRBIRINDEN BAGIMSIZ kontrol edilir (eski hata: ikisi birden sabit
    olmadikca hicbiri duzelmiyordu).
    """

    def __init__(self, ad, ters):
        self.ad = ad
        self.ters = ters
        self.merkez = 512
        self.ilk_merkez = 512            # guc-acilisi kalibrasyonu - ASLA degismez
        self._kal_ornekleri = []
        self._pencere = deque(maxlen=STABILITE_PENCERE)
        self._guvenlik_pencere = deque(maxlen=GUVENLIK_SABIT_PENCERE)
        self._son_guvenlik_uyari = False

    # --- kalibrasyon (baglaninca, joystick'e dokunulmadigi varsayimiyla) ---
    def kalibrasyon_ornegi_ekle(self, ham):
        self._kal_ornekleri.append(ham)

    def kalibrasyonu_bitir(self):
        if self._kal_ornekleri:
            ort = sum(self._kal_ornekleri) // len(self._kal_ornekleri)
        else:
            ort = 512
        self.merkez = ort
        self.ilk_merkez = ort
        self._kal_ornekleri = []
        self._pencere.clear()
        self._guvenlik_pencere.clear()
        self._son_guvenlik_uyari = False

    def _normalize(self, ham):
        # Merkez tam 512 olmayabilir - +/- yonlerdeki kalan araligi merkeze
        # gore AYRI olcekle, yoksa dar taraf erken doyar.
        fark = ham - self.merkez
        if abs(fark) < ADC_OLU_BOLGE:
            return 0.0
        if fark > 0:
            bolen = max(1023 - self.merkez, 1)
        else:
            bolen = max(self.merkez, 1)
        oran = _kirp(fark / bolen, -1.0, 1.0)
        return -oran if self.ters else oran

    def guncelle(self, ham):
        """ham ADC -> (oran [-1..1], guvenlik_mesaji|None)."""
        # --- canli oto-merkezleme: yeterince UZUN + SABIT kalirsa ve
        # MEVCUT merkeze YAKINSA (tam itilme DEGIL) merkezi duzelt ---
        #
        # DUZELTME (2026-09-04, kullanici: "surerken joystick yazilimsal
        # olarak takili kaliyor" + "joystick ortada dururken ileri/sag komut
        # full veriyor") - BUG: eskiden `aday` MEVCUT merkeze (self.merkez)
        # gore kontrol ediliyordu. self.merkez ZATEN kaymissa (kumulatif),
        # bu kontrol hep "yakin" cikar - VE joystick suru sirasinda KADEMELI
        # itilip her basamakta sadece ~0.3sn (STABILITE_PENCERE) sabit
        # tutulunca (TAMAMEN DOGAL surus hareketi - hizlanirken durmadan
        # itmek insan icin zor) merkez, joystick'in o an tutuldugu yere
        # basamak basamak KAYIYORDU (izole testte DOGRULANDI: 560->900 arasi
        # 7 basamakla merkez tam tutulan degere kadar sursunuyor, oran hep
        # 0.000 -> arac "takili" hissi). Joystick gercek ortasina birakilinca
        # da artik kaymis merkeze gore fark cok buyuk cikiyor, ters isaretle
        # "full komut" gibi okunuyor (ikinci sikayet, AYNI kok neden).
        #
        # Artik SABIT referans olan ilk_merkez'e (guc-acilisi kalibrasyonu,
        # ASLA degismez) gore kontrol ediliyor - merkez sadece GERCEK
        # idle'a yakin kucuk driftleri (MAKS_MERKEZ_KAYMASI icinde) telafi
        # eder, joystick'in tutuldugu herhangi bir pozisyona ASLA kaymaz.
        self._pencere.append(ham)
        if len(self._pencere) == STABILITE_PENCERE:
            if max(self._pencere) - min(self._pencere) < STABILITE_TOLERANSI:
                aday = sum(self._pencere) // len(self._pencere)
                if abs(aday - self.ilk_merkez) <= MAKS_MERKEZ_KAYMASI:
                    self.merkez = aday

        oran = self._normalize(ham)

        # --- SON GUVENLIK AGI: canli merkeze HIC guvenmeden, ham ADC ~1sn
        # (neredeyse) kipirdamadi VE ilk kalibrasyon merkezine yakinsa,
        # oran ne hesaplanirsa hesaplansin sifira zorla ---
        mesaj = None
        self._guvenlik_pencere.append(ham)
        if len(self._guvenlik_pencere) == GUVENLIK_SABIT_PENCERE:
            sabit = (max(self._guvenlik_pencere) - min(self._guvenlik_pencere)) < GUVENLIK_SABIT_TOLERANSI
            merkeze_yakin = abs(ham - self.ilk_merkez) <= GUVENLIK_SABIT_MAKS_UZAKLIK
            if sabit and merkeze_yakin and oran != 0.0:
                if not self._son_guvenlik_uyari:
                    mesaj = (f"⚠️ GÜVENLİK AĞI: {self.ad} ekseni ~1sn hareketsiz "
                             f"(merkeze yakın) ama oran={oran:.2f} hesaplanmıştı, "
                             f"ZORLA sıfırlandı (ham={ham}, canlı merkez={self.merkez}, "
                             f"ilk merkez={self.ilk_merkez})")
                    self._son_guvenlik_uyari = True
                oran = 0.0
            else:
                self._son_guvenlik_uyari = False
        return oran, mesaj


class _DijitalDurum:
    """Debounce'lu digital giris. ham: 0 = LOW/GND/basili, 1 = HIGH/serbest.

    aktif_dusuk=True (varsayilan): LOW iken aktif (INPUT_PULLUP + GND'ye
    cekilen buton/anahtar). ters kabloluysa False.
    """

    def __init__(self, debounce_n=DEBOUNCE_ORNEK, aktif_dusuk=True):
        self.aktif = False
        self._aday = False
        self._sayac = 0
        self._debounce_n = debounce_n
        self._aktif_dusuk = aktif_dusuk

    def ham_aktif(self, ham):
        return (int(ham) == 0) if self._aktif_dusuk else (int(ham) == 1)

    def guncelle(self, ham):
        """-> 'bas' | 'birak' | None (debounce'lu kenar)."""
        yeni = self.ham_aktif(ham)
        if yeni == self.aktif:
            self._sayac = 0
            return None
        if yeni != self._aday:
            self._aday = yeni
            self._sayac = 1
            return None
        self._sayac += 1
        if self._sayac >= self._debounce_n:
            self.aktif = yeni
            self._sayac = 0
            return "bas" if yeni else "birak"
        return None


class KontrolPaneliThread(QThread):
    # sol joystick, TANK KARISIMI SONRASI oran [-1..1] (PWM DEGIL)
    surus_sinyali = pyqtSignal(float, float)      # sol_oran, sag_oran
    # sag joystick, HAM oran [-1..1] - silah pan/tilt
    turret_sinyali = pyqtSignal(float, float)     # x, y
    # potansiyometre -> PWM ust siniri (85..255), histerezis + throttle sonrasi
    pot_sinyali = pyqtSignal(int)
    # yer istasyonu bataryasi (A2, gerilim bolucu uzerinden) -> "%XX" metni,
    # main.py'deki label_yerSarj'in beklediği format (bkz. yer_batarya_renklendir)
    yer_batarya_sinyali = pyqtSignal(str)
    # toggle switch'ler (GND'ye alininca aktif=True); baglaninca ilk durum da yayinlanir
    sol_toggle_sinyali = pyqtSignal(bool)         # D4 - ACIL STOP
    sag_toggle_sinyali = pyqtSignal(bool)         # D5 - SILAH ATES
    # butonlar - sadece BAS kenari
    sol_buton_sinyali = pyqtSignal()             # D9 - Manuel/Otonom
    sag_buton_sinyali = pyqtSignal()             # D10 - Farlar
    baglanti_sinyali = pyqtSignal(bool)
    guvenlik_sinyali = pyqtSignal(str)           # log icin
    ham_veri_sinyali = pyqtSignal(dict)          # PANEL TESTI dialogu (ham_yayin_acik iken)

    def __init__(self, port=None):
        super().__init__()
        self._sabit_port = port
        self._calisiyor = True
        self._baglanti_durumu = False
        # PANEL TESTI dialogu acikken True - bosuna 50Hz dict kurmayalim
        self.ham_yayin_acik = False
        # DUZELTME (2026-09-04): "KAYDET VE UYGULA" bu bayragi kaldirir,
        # ic dongu bunu gorunce (Arduino'yu HIC koparmadan/programi yeniden
        # baslatmadan) TAZE diskteki ayarlarla yeniden baglanir - bkz.
        # ayarlari_yeniden_yukle() / run().
        self._ayarlar_degisti_bayragi = False
        # DUZELTME (2026-09-05, canli bildirildi: "joystickleri ellemiyorum
        # ama sağa dönüyor, panel testinde sağa çekilmiş gibi davranıyor") -
        # guc-acilisi kalibrasyonundaki ilk_merkez GERCEK fiziksel ortadan
        # uzaksa (ör. baglanti kurulurken joystick tam ortada degildi),
        # canli oto-merkezleme MAKS_MERKEZ_KAYMASI (90 birim) disina asla
        # cikamadigi icin bunu KENDI BASINA duzeltemiyordu. PANEL TESTI
        # dialoguna "Merkezi Sıfırla" butonu eklendi - bu bayrak, AYNI
        # guvenli reconnect+yeniden-kalibrasyon yolunu (ayarlar_degisti ile
        # AYNI mekanizma, Arduino hic koparilmadan) tetikler, sadece ayri
        # bir log mesaji icin ayri bayrak.
        self._merkez_sifirla_bayragi = False

    def ayarlari_yeniden_yukle(self):
        """PANEL TESTI dialogundaki 'KAYDET VE UYGULA' butonundan cagrilir -
        thread'i yeniden baglanip diskteki (yeni kaydedilmis) ayarlari
        uygulamaya zorlar."""
        self._ayarlar_degisti_bayragi = True

    def merkezi_sifirla(self):
        """PANEL TESTI dialogundaki '🎯 Merkezi Sıfırla' butonundan cagrilir -
        joystickler BIRAKILMIŞ (dokunulmuyor) varsayimiyla, guc-acilisindaki
        AYNI kalibrasyon dongusunu (_kalibre_et) yeniden calistirir; ilk_merkez
        de dahil TÜM merkez degerleri o an okunan ham ADC'ye sifirlanir."""
        self._merkez_sifirla_bayragi = True

    # ------------------------------------------------------------------
    def _baglanti_durumunu_guncelle(self, durum):
        if durum != self._baglanti_durumu:
            self._baglanti_durumu = durum
            self.baglanti_sinyali.emit(durum)

    def _satir_ayristir(self, satir):
        """'P,a3,a4,a0,a1,a5,a2,d4,d5,d9,d10' -> (analog dict, digital dict) | None.

        GERİYE DÖNÜK UYUMLULUK (2026-09-05, A2/yer bataryası eklendi): eski
        firmware (henüz yeni .ino Arduino'ya YÜKLENMEMİŞSE) hâlâ 10 parça
        gönderir (A2 alanı YOK) - bu durumda TÜM DİĞER kontroller (joystick/
        toggle/buton) YİNE DE çalışmaya devam etsin diye satır REDDEDİLMEZ,
        sadece A2 None sayılır (yer bataryası göstergesi "veri yok" kalır,
        Arduino'ya yeni firmware yüklenince otomatik devreye girer).
        """
        if not satir.startswith("P,"):
            return None
        parcalar = satir.split(",")
        if len(parcalar) >= 11:
            try:
                a3, a4, a0, a1, a5, a2 = (int(parcalar[1]), int(parcalar[2]),
                                          int(parcalar[3]), int(parcalar[4]),
                                          int(parcalar[5]), int(parcalar[6]))
                d4, d5, d9, d10 = (int(parcalar[7]), int(parcalar[8]),
                                   int(parcalar[9]), int(parcalar[10]))
            except ValueError:
                return None
            return ({"A3": a3, "A4": a4, "A0": a0, "A1": a1, "A5": a5, "A2": a2},
                    {"D4": d4, "D5": d5, "D9": d9, "D10": d10})
        if len(parcalar) < 10:
            return None
        try:
            a3, a4, a0, a1, a5 = (int(parcalar[1]), int(parcalar[2]),
                                  int(parcalar[3]), int(parcalar[4]), int(parcalar[5]))
            d4, d5, d9, d10 = (int(parcalar[6]), int(parcalar[7]),
                               int(parcalar[8]), int(parcalar[9]))
        except ValueError:
            return None
        return ({"A3": a3, "A4": a4, "A0": a0, "A1": a1, "A5": a5, "A2": None},
                {"D4": d4, "D5": d5, "D9": d9, "D10": d10})

    def _kalibre_et(self, ser, sol_x, sol_y, sag_x, sag_y, sol_x_pin, sol_y_pin, sag_x_pin, sag_y_pin):
        ser.reset_input_buffer()
        alinan = 0
        while alinan < KALIBRASYON_ORNEK_SAYISI and self._calisiyor:
            satir = ser.readline().decode("utf-8", errors="ignore").strip()
            veri = self._satir_ayristir(satir)
            if veri is None:
                continue
            analog, _ = veri
            sol_x.kalibrasyon_ornegi_ekle(analog[sol_x_pin])
            sol_y.kalibrasyon_ornegi_ekle(analog[sol_y_pin])
            sag_x.kalibrasyon_ornegi_ekle(analog[sag_x_pin])
            sag_y.kalibrasyon_ornegi_ekle(analog[sag_y_pin])
            alinan += 1
        for eksen in (sol_x, sol_y, sag_x, sag_y):
            eksen.kalibrasyonu_bitir()

    # ------------------------------------------------------------------
    def run(self):
        if serial is None:
            self.baglanti_sinyali.emit(False)
            self.guvenlik_sinyali.emit("⚠️ pyserial kurulu değil - kontrol paneli devre dışı.")
            return

        while self._calisiyor:
            # DUZELTME (2026-09-04): ayarlar HER (yeniden)baglantida
            # DISKTEN TAZE okunuyor - "PANEL TESTI" dialogundaki "KAYDET VE
            # UYGULA" (bkz. ayarlari_yeniden_yukle) kod duzenlemeden/
            # arayuzu yeniden baslatmadan aninda etkili olur.
            ayarlar = ayarlari_yukle()
            sol_x_pin = ayarlar["sol_x_pin"]
            sag_x_pin = ayarlar["sag_x_pin"]
            # hangi pin Y ekseni: X olmayan digeri
            sol_y_pin = "A4" if sol_x_pin == "A3" else "A3"
            sag_y_pin = "A1" if sag_x_pin == "A0" else "A0"

            port = self._sabit_port or _port_bul()
            if port is None:
                self._baglanti_durumunu_guncelle(False)
                time.sleep(2.0)
                continue
            try:
                with serial.Serial(port, BAUDRATE, timeout=1.0) as ser:
                    time.sleep(2.0)  # Arduino reset

                    sol_x = _EksenDurumu("SOL-X", ayarlar["sol_ters_x"])
                    sol_y = _EksenDurumu("SOL-Y", ayarlar["sol_ters_y"])
                    sag_x = _EksenDurumu("SAĞ-X", ayarlar["sag_ters_x"])
                    sag_y = _EksenDurumu("SAĞ-Y", ayarlar["sag_ters_y"])
                    self._kalibre_et(ser, sol_x, sol_y, sag_x, sag_y,
                                      sol_x_pin, sol_y_pin, sag_x_pin, sag_y_pin)

                    sol_toggle = _DijitalDurum(aktif_dusuk=not ayarlar["sol_toggle_ters"])
                    sag_toggle = _DijitalDurum(aktif_dusuk=not ayarlar["sag_toggle_ters"])
                    sol_buton = _DijitalDurum()
                    sag_buton = _DijitalDurum()

                    pot_ema = None
                    son_pot_pwm = None
                    son_pot_zamani = 0.0
                    yer_bat_ema = None
                    son_yer_bat_yuzde = None
                    son_yer_bat_zamani = 0.0
                    ilk_toggle_yayinlandi = False

                    self._baglanti_durumunu_guncelle(True)

                    while (self._calisiyor and not self._ayarlar_degisti_bayragi
                           and not self._merkez_sifirla_bayragi):
                        satir = ser.readline().decode("utf-8", errors="ignore").strip()
                        if not satir:
                            continue
                        veri = self._satir_ayristir(satir)
                        if veri is None:
                            continue
                        analog, digital = veri

                        # --- TOGGLE SWITCH'LER (joystick emit'inden ONCE:
                        # ACIL STOP kalkani ilk kareden itibaren gecerli olsun) ---
                        if not ilk_toggle_yayinlandi:
                            # baglaninca mevcut durumu bir kez yayinla (operator
                            # zaten acil-stop'u GND'ye almis olabilir)
                            sol_toggle.aktif = sol_toggle.ham_aktif(digital["D4"])
                            sag_toggle.aktif = sag_toggle.ham_aktif(digital["D5"])
                            self.sol_toggle_sinyali.emit(sol_toggle.aktif)
                            self.sag_toggle_sinyali.emit(sag_toggle.aktif)
                            ilk_toggle_yayinlandi = True
                        else:
                            if sol_toggle.guncelle(digital["D4"]) is not None:
                                self.sol_toggle_sinyali.emit(sol_toggle.aktif)
                            if sag_toggle.guncelle(digital["D5"]) is not None:
                                self.sag_toggle_sinyali.emit(sag_toggle.aktif)

                        # --- JOYSTICK EKSENLERI ---
                        sx, msg = sol_x.guncelle(analog[sol_x_pin])
                        if msg:
                            self.guvenlik_sinyali.emit(msg)
                        sy, msg = sol_y.guncelle(analog[sol_y_pin])
                        if msg:
                            self.guvenlik_sinyali.emit(msg)
                        gx, msg = sag_x.guncelle(analog[sag_x_pin])
                        if msg:
                            self.guvenlik_sinyali.emit(msg)
                        gy, msg = sag_y.guncelle(analog[sag_y_pin])
                        if msg:
                            self.guvenlik_sinyali.emit(msg)

                        # tank karisimi (surus_joystick_sistemi.py ile ayni)
                        sol_oran = _kirp(sy + sx, -1.0, 1.0)
                        sag_oran = _kirp(sy - sx, -1.0, 1.0)

                        # --- ACIL STOP YEREL KALKANI ---
                        # D4 GND'deyken araca HICBIR surus komutu gitmesin
                        # (arayuz tarafi kilit + 0.8sn watchdog'a EK katman).
                        # Silah (turret + ates) KASITLI OLARAK etkilenmez -
                        # acil stop "araçtaki motorlara giden kodlar" icindir.
                        if not sol_toggle.aktif:
                            self.surus_sinyali.emit(sol_oran, sag_oran)
                        self.turret_sinyali.emit(gx, gy)

                        # --- POTANSIYOMETRE -> PWM UST SINIRI ---
                        ham_pot = analog["A5"]
                        pot_ema = ham_pot if pot_ema is None else (
                            POT_EMA_ALFA * ham_pot + (1 - POT_EMA_ALFA) * pot_ema)
                        pwm = int(round(PWM_ALT + (pot_ema / 1023.0) * (PWM_UST - PWM_ALT)))
                        pwm = _kirp(pwm, PWM_ALT, PWM_UST)
                        simdi = time.time()
                        if (son_pot_pwm is None or
                                (abs(pwm - son_pot_pwm) >= POT_MIN_DEGISIM and
                                 simdi - son_pot_zamani >= POT_MIN_ARALIK_S)):
                            son_pot_pwm = pwm
                            son_pot_zamani = simdi
                            self.pot_sinyali.emit(int(pwm))

                        # --- YER BATARYASI (A2, gerilim bolucu) ---
                        # bkz. dosya basi 'YER BATARYASI' notu. A2 None ise
                        # (eski firmware, henuz yuklenmemis) sessizce atlanir -
                        # gosterge "veri yok" durumunda kalir, HATA vermez.
                        if analog.get("A2") is not None:
                            ham_yer_bat = analog["A2"]
                            yer_bat_ema = ham_yer_bat if yer_bat_ema is None else (
                                YER_BAT_EMA_ALFA * ham_yer_bat + (1 - YER_BAT_EMA_ALFA) * yer_bat_ema)
                            _, yer_bat_yuzde = _yer_bat_yuzde_hesapla(yer_bat_ema)
                            yer_bat_yuzde = int(round(yer_bat_yuzde))
                            if (son_yer_bat_yuzde is None or
                                    (yer_bat_yuzde != son_yer_bat_yuzde and
                                     simdi - son_yer_bat_zamani >= YER_BAT_MIN_ARALIK_S)):
                                son_yer_bat_yuzde = yer_bat_yuzde
                                son_yer_bat_zamani = simdi
                                self.yer_batarya_sinyali.emit(f"%{yer_bat_yuzde}")

                        # --- BUTONLAR (sadece bas kenari) ---
                        if sol_buton.guncelle(digital["D9"]) == "bas":
                            self.sol_buton_sinyali.emit()
                        if sag_buton.guncelle(digital["D10"]) == "bas":
                            self.sag_buton_sinyali.emit()

                        # --- PANEL TESTI ham veri ---
                        if self.ham_yayin_acik:
                            if analog.get("A2") is not None:
                                _yer_bat_v, _yer_bat_yzd = _yer_bat_yuzde_hesapla(analog["A2"])
                                yer_bat_metin = f"{_yer_bat_v:.2f}V (%{_yer_bat_yzd:.0f})"
                            else:
                                yer_bat_metin = "YOK (eski firmware - A2 gönderilmiyor)"
                            self.ham_veri_sinyali.emit({
                                "A3": analog["A3"], "A4": analog["A4"],
                                "A0": analog["A0"], "A1": analog["A1"],
                                "A5": analog["A5"],
                                "A2": analog.get("A2"),
                                "yer_bat_metin": yer_bat_metin,
                                "D4": digital["D4"], "D5": digital["D5"],
                                "D9": digital["D9"], "D10": digital["D10"],
                                "sol_oran": round(sol_oran, 3),
                                "sag_oran": round(sag_oran, 3),
                                "turret_x": round(gx, 3), "turret_y": round(gy, 3),
                                "pwm": int(pwm),
                                "sol_merkez": (sol_x.merkez, sol_y.merkez),
                                "sag_merkez": (sag_x.merkez, sag_y.merkez),
                            })
                    if self._ayarlar_degisti_bayragi:
                        # DUZELTME (2026-09-04): ic dongu "KAYDET VE UYGULA"
                        # yuzunden bitti (hata DEGIL) - bayragi temizleyip
                        # dis dongu basina don, 'with' bloğundan cikildigi
                        # icin seri port zaten kapandi (Arduino resetlenir,
                        # ayni 2sn bekleme + yeniden kalibrasyon dis dongude
                        # dogal olarak tekrar calisir).
                        self._ayarlar_degisti_bayragi = False
                        self.guvenlik_sinyali.emit("🔧 Kontrol paneli kalibrasyonu güncellendi, yeniden bağlanılıyor...")
                    elif self._merkez_sifirla_bayragi:
                        self._merkez_sifirla_bayragi = False
                        self.guvenlik_sinyali.emit("🎯 Joystick merkezi sıfırlanıyor - joystickleri BIRAKIN, yeniden kalibre ediliyor...")
            except (serial.SerialException, OSError):
                self._baglanti_durumunu_guncelle(False)
                time.sleep(2.0)

    def durdur(self):
        self._calisiyor = False
        self.quit()
        self.wait()
