#!/usr/bin/env python3
"""KARSI TARAFTA calistirilacak goruntu alici (arayuz/yer istasyonu makinesi).

Jetson uzerindeki turret_node.py (silah) ve tabela_node.py, isaretlenmis
JPEG karelerini UDP ile bu makinenin IP'sine (10.40.64.48) gonderiyor:
  - Silah (turret)  -> port 5000
  - Tabela          -> port 5001

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

    def __init__(self, udp_ip="0.0.0.0", udp_port=5000, tabela_ip="0.0.0.0", tabela_port=5001):
        super().__init__()
        self.aktif = True
        self.silah_ip = udp_ip
        self.silah_port = udp_port
        self.tabela_ip = tabela_ip
        self.tabela_port = tabela_port
        self.son_silah_frame = None
        self.son_tabela_frame = None

    def _frame_to_qimage(self, data):
        frame = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return None

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = frame_rgb.shape
        return QImage(frame_rgb.data, w, h, ch * w, QImage.Format_RGB888).copy()

    def run(self):
        sock_silah = yeni_soket(self.silah_ip, self.silah_port)
        sock_tabela = yeni_soket(self.tabela_ip, self.tabela_port)
        soketler = [sock_silah, sock_tabela]

        print(f'[*] Silah (turret) : {self.silah_ip}:{self.silah_port} dinleniyor')
        print(f'[*] Tabela         : {self.tabela_ip}:{self.tabela_port} dinleniyor')

        try:
            while self.aktif:
                hazir, _, _ = select.select(soketler, [], [], 0.2)
                for s in hazir:
                    try:
                        data, _addr = s.recvfrom(65535)
                    except OSError:
                        continue

                    q_img = self._frame_to_qimage(data)
                    if q_img is None:
                        continue

                    if s is sock_silah:
                        self.son_silah_frame = q_img
                    else:
                        self.son_tabela_frame = q_img

                    self.kare_sinyali.emit({
                        1: self.son_silah_frame,
                        2: self.son_tabela_frame,
                        3: None,
                    })
        except Exception as e:
            print(f"UDP kamera hatası: {e}")
        finally:
            sock_silah.close()
            sock_tabela.close()

    def stop(self):
        self.aktif = False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--ip', default='0.0.0.0', help='Bu makinede dinlenecek IP (genelde 0.0.0.0 birakilir)')
    parser.add_argument('--silah-port', type=int, default=5000, help='turret_node.py video_target_port ile ayni olmali')
    parser.add_argument('--tabela-port', type=int, default=5001, help='tabela_node.py video_target_port ile ayni olmali')
    args = parser.parse_args()

    sock_silah = yeni_soket(args.ip, args.silah_port)
    sock_tabela = yeni_soket(args.ip, args.tabela_port)
    soketler = [sock_silah, sock_tabela]

    print(f'[*] Silah (turret) : {args.ip}:{args.silah_port} dinleniyor')
    print(f'[*] Tabela         : {args.ip}:{args.tabela_port} dinleniyor')
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

                pencere_adi = 'Silah (Turret)' if s is sock_silah else 'Tabela'
                cv2.imshow(pencere_adi, frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        pass
    finally:
        sock_silah.close()
        sock_tabela.close()
        cv2.destroyAllWindows()
        print('[*] Alici kapatildi.')


if __name__ == '__main__':
    main()

