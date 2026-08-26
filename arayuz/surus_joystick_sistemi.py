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

try:
    import serial
except ImportError:
    serial = None

from PyQt5.QtCore import QThread, pyqtSignal

BAUDRATE = 115200
ADC_OLU_BOLGE = 40      # merkez etrafinda titremeyi yok say
TERS_X = False          # joystick fiziksel yonu ters gelirse True yapin
TERS_Y = False
KALIBRASYON_ORNEK_SAYISI = 25  # ~0.5sn @ 50Hz


def _port_bul():
    adaylar = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    return adaylar[0] if adaylar else None


class SurusJoystickThread(QThread):
    pwm_sinyali = pyqtSignal(float, float)   # sol_oran, sag_oran [-1, 1] - PWM DEGIL
    baglanti_sinyali = pyqtSignal(bool)

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
                    self._baglanti_durumunu_guncelle(True)
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

                        x = self._normalize(x_ham, merkez_x, TERS_X)
                        y = self._normalize(y_ham, merkez_y, TERS_Y)

                        sol_oran = max(min(y + x, 1.0), -1.0)
                        sag_oran = max(min(y - x, 1.0), -1.0)

                        self.pwm_sinyali.emit(sol_oran, sag_oran)
            except (serial.SerialException, OSError):
                self._baglanti_durumunu_guncelle(False)
                time.sleep(2.0)

    def durdur(self):
        self._calisiyor = False
        self.quit()
        self.wait()
