#!/usr/bin/env python3
"""KARSI TARAFTA calistirilacak goruntu alici (arayuz/yer istasyonu makinesi).

Jetson uzerindeki tabela_node.py (on), turret_node.py (silah) ve
arka_kamera_node.py, isaretlenmis/ham JPEG karelerini UDP ile bu makinenin
IP'sine (192.168.1.20) gonderiyor - PORT ATAMASI (2026-09-01, kullanici
istegi - araç tarafindaki tufan_mppi.launch.py ile BIREBIR AYNI olmali,
bkz. o dosyadaki "kullanici port atamasini TERSINE CEVIRDI" notu):
  - On (tabela)     -> port 5000
  - Silah (turret)  -> port 5001
  - Arka            -> port 5002

Bu modül, bu görüntüleri Qt arayüzüne aktaracak şekilde çalışır.
"""

import argparse
import os
import select
import socket
import time

import cv2
import numpy as np
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtGui import QImage


# --- VIDEO KAYDI (2026-09-09, kullanici istegi: "veri kaydet butonuna
# basinca SADECE kamera goruntulerini kaydetsin") ---
# Kayitlar bu dosyanin yanindaki 'kayitlar/' klasorune, her basista yeni
# bir ZAMAN DAMGALI alt klasore yazilir - eski kayitlarin uzerine ASLA
# yazilmaz.
KAYIT_KOK_KLASORU = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 'kayitlar')
# Kaynak akis MJPEG oldugu icin ayni kodek: en dusuk riskli/en uyumlu
# secim (Jetson'da ek kodek paketi GEREKTIRMEZ), .avi kabinde.
KAYIT_KODEK = 'MJPG'
KAYIT_UZANTI = '.avi'
# FPS OLCULEREK belirlenir (sabit deger yazsak kayit HIZLI/YAVAS oynar):
# ilk karelerin zaman damgalarindan gercek kare hizi hesaplanir. Olcum
# tamamlanana kadar kareler TAMPONA alinir, writer acilinca yazilir.
KAYIT_FPS_OLCUM_KARE = 12
KAYIT_FPS_OLCUM_SURESI_S = 2.0
KAYIT_FPS_ALT, KAYIT_FPS_UST = 4.0, 30.0
KAYIT_FPS_VARSAYILAN = 15.0


def yeni_soket(ip, port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((ip, port))
    s.setblocking(False)
    return s


class KameraThread(QThread):
    kare_sinyali = pyqtSignal(dict)

    def __init__(self, ip="0.0.0.0", on_port=5000, silah_port=5001, arka_port=5002):
        super().__init__()
        self.aktif = True
        self.on_ip = ip
        self.on_port = on_port
        self.silah_ip = ip
        self.silah_port = silah_port
        self.arka_ip = ip
        self.arka_port = arka_port
        self.son_on_frame = None
        self.son_silah_frame = None
        self.son_arka_frame = None
        # --- video kaydi durumu (bkz. kayit_durumu_degistir) ---
        # Bayrak GUI thread'inden yazilir, run() icinde OKUNUR; writer
        # nesneleri SADECE run()'in kendi thread'inde acilir/kapanir -
        # OpenCV VideoWriter thread'ler arasi paylasilmaz.
        self._kayit_isteniyor = False
        self._kayit_aktif = False
        self._kayit_klasoru = None
        self._kayit_durumlari = {}

    def _frame_to_qimage(self, data):
        frame = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return None
        return self._bgr_to_qimage(frame)

    @staticmethod
    def _bgr_to_qimage(frame):
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = frame_rgb.shape
        return QImage(frame_rgb.data, w, h, ch * w, QImage.Format_RGB888).copy()

    # ------------------ VIDEO KAYDI ------------------
    def kayit_durumu_degistir(self, aktif):
        """GUI thread'inden cagrilir (main.py::veri_kaydet_tetikle).

        SADECE bayragi degistirir - gercek dosya acma/kapama run()'in
        kendi thread'inde yapilir (OpenCV VideoWriter thread'ler arasi
        paylasilmaz). Idempotent: zaten kayitta iken True, kayitta
        DEGILKEN False zararsizdir.

        Doner: kayit klasoru (baslatildiysa, henuz olusmamis olabilir)."""
        if aktif and not self._kayit_isteniyor:
            # Klasor ADI buton basildigi ANDA belirlenir ki main.py loga
            # yazabilsin; dizinin KENDISI run() thread'inde olusturulur.
            self._kayit_bekleyen_klasor = os.path.join(
                KAYIT_KOK_KLASORU, time.strftime('%Y%m%d_%H%M%S'))
        self._kayit_isteniyor = bool(aktif)
        return (self._kayit_bekleyen_klasor if aktif else self._kayit_klasoru)

    def _kayit_baslat(self):
        self._kayit_klasoru = getattr(self, '_kayit_bekleyen_klasor', None) or \
            os.path.join(KAYIT_KOK_KLASORU, time.strftime('%Y%m%d_%H%M%S'))
        try:
            os.makedirs(self._kayit_klasoru, exist_ok=True)
        except OSError as e:
            print(f'[!] Kayit klasoru olusturulamadi ({self._kayit_klasoru}): {e}')
            self._kayit_isteniyor = False
            return
        self._kayit_durumlari = {}
        self._kayit_aktif = True
        print(f'[*] VIDEO KAYDI BASLADI -> {self._kayit_klasoru}')

    def _kayit_karesi_yaz(self, anahtar, frame):
        """Bir kamera karesini kaydeder. FPS olculene kadar kareler
        TAMPONA alinir (bkz. KAYIT_FPS_OLCUM_KARE) - yanlis FPS ile
        acilan dosya hizli/yavas oynardi."""
        d = self._kayit_durumlari.get(anahtar)
        if d is None:
            d = {'writer': None, 'tampon': [], 'zamanlar': [], 'kare': 0,
                 'boyut': None}
            self._kayit_durumlari[anahtar] = d

        su_an = time.time()
        if d['writer'] is None:
            d['tampon'].append(frame)
            d['zamanlar'].append(su_an)
            yeterli_kare = len(d['zamanlar']) >= KAYIT_FPS_OLCUM_KARE
            yeterli_sure = (d['zamanlar'][-1] - d['zamanlar'][0]) >= KAYIT_FPS_OLCUM_SURESI_S
            if not (yeterli_kare or yeterli_sure):
                return
            gecen = d['zamanlar'][-1] - d['zamanlar'][0]
            fps = ((len(d['zamanlar']) - 1) / gecen) if gecen > 0 else KAYIT_FPS_VARSAYILAN
            fps = min(max(fps, KAYIT_FPS_ALT), KAYIT_FPS_UST)
            h, w = frame.shape[:2]
            yol = os.path.join(self._kayit_klasoru, anahtar + KAYIT_UZANTI)
            writer = cv2.VideoWriter(
                yol, cv2.VideoWriter_fourcc(*KAYIT_KODEK), fps, (w, h))
            if not writer.isOpened():
                print(f'[!] Kayit dosyasi acilamadi: {yol}')
                d['tampon'] = []
                d['zamanlar'] = []
                return
            d['writer'] = writer
            d['boyut'] = (w, h)
            d['yol'] = yol
            print(f'[*] {anahtar}: {w}x{h} @ {fps:.1f} fps -> {os.path.basename(yol)}')
            for tamponlu in d['tampon']:
                writer.write(tamponlu)
                d['kare'] += 1
            d['tampon'] = []
            return

        # Cozunurluk degisirse (kamera yeniden acildi) writer'a YAZILAMAZ -
        # sessizce bozuk dosya olusmasin diye kare olceklenir.
        if d['boyut'] is not None and frame.shape[1::-1] != d['boyut']:
            frame = cv2.resize(frame, d['boyut'])
        d['writer'].write(frame)
        d['kare'] += 1

    def _kayit_bitir(self):
        for anahtar, d in sorted(self._kayit_durumlari.items()):
            w = d.get('writer')
            if w is not None:
                try:
                    w.release()
                except Exception:
                    pass
                print(f'[*] {anahtar}: {d["kare"]} kare yazildi -> {d.get("yol")}')
            elif d['tampon']:
                print(f'[!] {anahtar}: FPS olculemeden durduruldu, '
                      f'{len(d["tampon"])} kare YAZILMADI (cok kisa kayit)')
        self._kayit_durumlari = {}
        self._kayit_aktif = False
        print(f'[*] VIDEO KAYDI DURDU -> {self._kayit_klasoru}')

    def run(self):
        sock_on = yeni_soket(self.on_ip, self.on_port)
        sock_silah = yeni_soket(self.silah_ip, self.silah_port)
        sock_arka = yeni_soket(self.arka_ip, self.arka_port)
        soketler = [sock_on, sock_silah, sock_arka]
        # Kayit dosya adlari (bkz. _kayit_karesi_yaz).
        kamera_adi = {sock_on: 'on_kamera', sock_silah: 'silah_kamerasi',
                      sock_arka: 'arka_kamera'}

        print(f'[*] On (tabela)    : {self.on_ip}:{self.on_port} dinleniyor')
        print(f'[*] Silah (turret) : {self.silah_ip}:{self.silah_port} dinleniyor')
        print(f'[*] Arka           : {self.arka_ip}:{self.arka_port} dinleniyor')

        try:
            while self.aktif:
                # KAYIT BASLAT/BITIR: kare AKMASA bile calissin. Bitirme
                # kare dongusunun icinde olsaydi, kayit durdurulduktan
                # sonra hic kare gelmezse dosya release EDILMEZ ve
                # indekssiz/oynatilamaz kalirdi.
                if self._kayit_isteniyor and not self._kayit_aktif:
                    self._kayit_baslat()
                elif self._kayit_aktif and not self._kayit_isteniyor:
                    self._kayit_bitir()

                hazir, _, _ = select.select(soketler, [], [], 0.2)
                for s in hazir:
                    # DÜZELTME (2026-09-04, kullanıcı: "kamera verisi geç
                    # geliyor") - eskiden select() HAZIR dediği anda
                    # SADECE BİR paket okunuyordu. Kod (çözme+QImage
                    # dönüşümü+sinyal) kameranın kare hızına yetişemezse,
                    # çekirdeğin UDP alma tamponunda BİRİKEN eski kareler
                    # bir SONRAKİ turda okunuyordu - görüntü gittikçe
                    # GERİYE DÜŞÜYORDU (gecikme SÜREKLİ ARTIYORDU - ağın
                    # kendisi değil, BİRİKEN BACKLOG suçluydu). Artık soket
                    # TAMAMEN BOŞALTILIYOR (en yeni paket dışında hepsi
                    # ATILIYOR, socket zaten non-blocking - bkz. yeni_soket)
                    # - canlı görüntüde "biraz önce" yerine her zaman
                    # "şu an" gösteriliyor.
                    data = None
                    while True:
                        try:
                            data, _addr = s.recvfrom(65535)
                        except OSError:
                            break
                    if data is None:
                        continue

                    # TEK cozme: hem ekran (QImage) hem kayit ayni
                    # BGR kareyi kullanir (eskiden sadece QImage'a
                    # cozuluyordu; kayit icin IKINCI kez cozmek bosa
                    # CPU olurdu).
                    frame = cv2.imdecode(
                        np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
                    if frame is None:
                        continue

                    # KAYIT: SADECE kamera goruntusu (kullanici istegi) -
                    # baska hicbir veri (telemetri/log) bu dosyalara
                    # yazilmaz. Baslat/bitir gecisleri dongu BASINDA
                    # (kare gelmese bile) ele alinir.
                    if self._kayit_aktif:
                        self._kayit_karesi_yaz(kamera_adi[s], frame)

                    q_img = self._bgr_to_qimage(frame)

                    if s is sock_silah:
                        self.son_silah_frame = q_img
                    elif s is sock_on:
                        self.son_on_frame = q_img
                    else:
                        self.son_arka_frame = q_img

                    # DUZELTME (2026-09-01, kullanici istegi): anahtar 3
                    # (ARKA) eskiden hic bu sozlukte gercek bir kareye
                    # baglanmiyordu (main.py sabit None kullaniyordu) -
                    # artik gercek arka_kamera_node.py karesi de akiyor.
                    self.kare_sinyali.emit({
                        1: self.son_silah_frame,
                        2: self.son_on_frame,
                        3: self.son_arka_frame,
                    })
        except Exception as e:
            print(f"UDP kamera hatası: {e}")
        finally:
            # Kayit acik kaldiysa dosyalari DUZGUN kapat - release()
            # edilmeyen bir .avi indekssiz/oynatilamaz kalir.
            if self._kayit_aktif:
                self._kayit_bitir()
            sock_on.close()
            sock_silah.close()
            sock_arka.close()

    def stop(self):
        # DUZELTME (2026-09-04, kullanici: "arayüzün bir anda çökmemesi
        # lazım çok nadiren de olsa oluyor") - eskiden sadece self.aktif
        # False yapiliyordu, run()'un bunu FARK EDIP CIKMASINI (en fazla
        # 0.2sn, select() timeout'u) BEKLEMEDEN geri donuyordu. Uygulama
        # kapanirken bu QThread nesnesi HALA CALISIRKEN yok edilirse Qt
        # "QThread: Destroyed while thread is still running" fatal
        # hatasi ile SESSIZCE COKUYOR (terminate called without an active
        # exception) - digger worker thread'lerdeki (Ros2GcsMotoru,
        # NtripRtkThread, KontrolPaneliThread) dogru desen ile TUTARLI
        # hale getirildi: quit()+wait() ile GERCEKTEN bitmesini bekliyoruz.
        self.aktif = False
        self.quit()
        self.wait()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--ip', default='0.0.0.0', help='Bu makinede dinlenecek IP (genelde 0.0.0.0 birakilir)')
    parser.add_argument('--on-port', type=int, default=5000, help='tabela_node.py video_target_port ile ayni olmali')
    parser.add_argument('--silah-port', type=int, default=5001, help='turret_node.py video_target_port ile ayni olmali')
    parser.add_argument('--arka-port', type=int, default=5002, help='arka_kamera_node.py video_target_port ile ayni olmali')
    args = parser.parse_args()

    sock_on = yeni_soket(args.ip, args.on_port)
    sock_silah = yeni_soket(args.ip, args.silah_port)
    sock_arka = yeni_soket(args.ip, args.arka_port)
    soketler = [sock_on, sock_silah, sock_arka]

    print(f'[*] On (tabela)    : {args.ip}:{args.on_port} dinleniyor')
    print(f'[*] Silah (turret) : {args.ip}:{args.silah_port} dinleniyor')
    print(f'[*] Arka           : {args.ip}:{args.arka_port} dinleniyor')
    print("[*] Cikmak icin herhangi bir pencerede 'q' tusuna basin veya Ctrl+C.")

    try:
        while True:
            hazir, _, _ = select.select(soketler, [], [], 0.5)
            for s in hazir:
                try:
                    data, _addr = s.recvfrom(65535)
                except OSError:
                    continue

                frame = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is None:
                    continue

                if s is sock_on:
                    pencere_adi = 'On (Tabela)'
                elif s is sock_silah:
                    pencere_adi = 'Silah (Turret)'
                else:
                    pencere_adi = 'Arka'
                cv2.imshow(pencere_adi, frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        pass
    finally:
        sock_on.close()
        sock_silah.close()
        sock_arka.close()
        cv2.destroyAllWindows()
        print('[*] Alici kapatildi.')


if __name__ == '__main__':
    main()
