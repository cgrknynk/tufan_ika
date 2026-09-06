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
import select
import socket

import cv2
import numpy as np
from PyQt5.QtCore import QThread, pyqtSignal
from PyQt5.QtGui import QImage


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

    def _frame_to_qimage(self, data):
        frame = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return None

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = frame_rgb.shape
        return QImage(frame_rgb.data, w, h, ch * w, QImage.Format_RGB888).copy()

    def run(self):
        sock_on = yeni_soket(self.on_ip, self.on_port)
        sock_silah = yeni_soket(self.silah_ip, self.silah_port)
        sock_arka = yeni_soket(self.arka_ip, self.arka_port)
        soketler = [sock_on, sock_silah, sock_arka]

        print(f'[*] On (tabela)    : {self.on_ip}:{self.on_port} dinleniyor')
        print(f'[*] Silah (turret) : {self.silah_ip}:{self.silah_port} dinleniyor')
        print(f'[*] Arka           : {self.arka_ip}:{self.arka_port} dinleniyor')

        try:
            while self.aktif:
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

                    q_img = self._frame_to_qimage(data)
                    if q_img is None:
                        continue

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
