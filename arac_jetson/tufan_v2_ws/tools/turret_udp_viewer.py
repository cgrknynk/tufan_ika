#!/usr/bin/env python3
"""Turret UDP goruntu izleyici (test/debug amacli, ROS'a bagli degil).

turret_node.py'nin gonderdigi JPEG kareleri UDP'den alip bir pencerede gosterir.
turret.launch.py'nin varsayilan hedefiyle (127.0.0.1:5000) ayni makinede test
etmek icin kullanilir.

Kullanim:
    python3 turret_udp_viewer.py
    python3 turret_udp_viewer.py --ip 127.0.0.1 --port 5000

Cikmak icin goruntu penceresindeyken 'q' tusuna basin veya Ctrl+C.
"""

import argparse
import socket

import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--ip', default='127.0.0.1', help='Dinlenecek IP (turret_node.py video_target_ip ile ayni olmali)')
    parser.add_argument('--port', type=int, default=5001, help='Dinlenecek port (turret_node.py video_target_port ile ayni olmali)')
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.ip, args.port))
    print(f'[*] {args.ip}:{args.port} dinleniyor... Cikmak icin pencerede q tusuna basin.')

    try:
        while True:
            data, addr = sock.recvfrom(65536)
            frame = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                cv2.imshow('Turret Goruntu (UDP)', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        cv2.destroyAllWindows()
        print('[*] Izleyici kapatildi.')


if __name__ == '__main__':
    main()
