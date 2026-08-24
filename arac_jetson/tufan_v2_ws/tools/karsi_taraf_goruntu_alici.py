#!/usr/bin/env python3
"""KARSI TARAFTA calistirilacak goruntu alici (arayuz/yer istasyonu makinesi).

Jetson uzerindeki turret_node.py (silah) ve tabela_node.py, isaretlenmis
JPEG karelerini UDP ile bu makinenin IP'sine (10.40.64.48) gonderiyor:
  - Silah (turret)  -> port 5000
  - Tabela          -> port 5001

Bu script her iki portu da dinleyip OpenCV pencerelerinde gosterir.
Jetson'a hic ihtiyac duymaz - bu makinede sadece python3 + opencv-python +
numpy kurulu olmasi yeterli:
    pip install opencv-python numpy

Kullanim (bu makinede, yani 10.40.64.48 uzerinde):
    python3 karsi_taraf_goruntu_alici.py
    python3 karsi_taraf_goruntu_alici.py --silah-port 5000 --tabela-port 5001

Cikmak icin herhangi bir goruntu penceresindeyken 'q' tusuna basin veya Ctrl+C.
"""

import argparse
import select
import socket

import cv2
import numpy as np


def yeni_soket(ip, port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((ip, port))
    s.setblocking(False)
    return s


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
