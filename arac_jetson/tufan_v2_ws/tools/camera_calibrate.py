#!/usr/bin/env python3
"""Turret kamerasi icin OpenCV satranc-tahtasi kalibrasyonu.

Neden: paralaks duzeltme formulu (dx_px = tan(atan2(offset_mm, mesafe_mm)) *
odak_px) dogru, ama "odak_px" (kameranin piksel cinsinden odak uzakligi)
sahada tahmin edilerek (800 -> 3054 -> 964 -> 810) belirlenmeye calisilinca
hicbir zaman tutarli sonuc vermedi. Bu script gercek odak_px'i standart
OpenCV checkerboard kalibrasyonuyla dogrudan OLCER - tahmin degil.

NOT: --square-mm degerinin GERCEK boyutla birebir aynı olmasi GEREKMEZ.
Zhang'in kalibrasyon yonteminde (cv2.calibrateCamera'nin temeli) kare
boyutunun tum karelerde AYNI (tutarli) varsayilmasi yeterli - cikan fx/fy
(piksel cinsinden odak uzakligi, bizim tek ihtiyacimiz olan deger) bu
olcege gore DEGISMEZ, sadece kullanilmayan mesafe/tvec ciktilari
etkilenir. Yani cetvelle olcmeden varsayilan degerle de calistirabilirsin.

Kullanim:
    1) generate_checkerboard.py ile checkerboard.png uret, yazdir ya da bir
       ekranda tam ekran goster.
    2) turret_udp_viewer.py'nin CALISTIGINDAN emin ol (canli geri besleme
       icin ayni UDP porta yayin yapar):
         python3 turret_udp_viewer.py --ip 127.0.0.1 --port 5000
    3) Bu scripti calistir, checkerboard'u kameranin onunde YAVASCA farkli
       acilardan/mesafelerden/konumlardan goster (viewer penceresinde yesil
       kose noktalari gorununce o kare otomatik kaydedilir):
         python3 camera_calibrate.py --camera-index 0

    Tamamlaninca fx/fy/cx/cy piksel cinsinden hesaplanip --out dosyasina
    (varsayilan camera_calibration.json) yazilir. turret_node.py bu fx
    degerini "kamera_odak_px" olarak kullanacak.
"""

import argparse
import json
import time

import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--camera-index', type=int, default=0)
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--cols', type=int, default=9, help='Ic kose sayisi (yatay)')
    parser.add_argument('--rows', type=int, default=6, help='Ic kose sayisi (dikey)')
    parser.add_argument('--square-mm', type=float, default=25.0, help='Sadece tutarlilik icin (bkz. dosya basindaki NOT) - fx/fy sonucunu etkilemez, cetvelle olcmene gerek yok')
    parser.add_argument('--target-frames', type=int, default=20)
    parser.add_argument('--min-interval', type=float, default=1.2, help='Iki kayit arasi min sure (s) - tahtayi hareket ettirmeye firsat tanir')
    parser.add_argument('--udp-ip', default='127.0.0.1')
    parser.add_argument('--udp-port', type=int, default=5000)
    parser.add_argument('--out', default='camera_calibration.json')
    args = parser.parse_args()

    import socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    pattern_size = (args.cols, args.rows)
    objp = np.zeros((args.cols * args.rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2)
    objp *= args.square_mm

    objpoints = []
    imgpoints = []

    cap = cv2.VideoCapture(args.camera_index)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    if not cap.isOpened():
        print(f'HATA: kamera acilamadi (index={args.camera_index})')
        return

    print(f'Kalibrasyon basladi. Hedef: {args.target_frames} kare. '
          f'Checkerboard\'u kameranin onunde YAVASCA farkli acilardan/mesafelerden goster.')
    print(f'Canli geri besleme icin viewer\'a bak: udp {args.udp_ip}:{args.udp_port}')

    frame_size = None
    son_kayit_zamani = 0.0
    kayit_sayisi = 0

    try:
        while kayit_sayisi < args.target_frames:
            ret, frame = cap.read()
            if not ret:
                time.sleep(0.05)
                continue

            frame_size = (frame.shape[1], frame.shape[0])
            gri = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            bulundu, kose = cv2.findChessboardCorners(
                gri, pattern_size,
                flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
            )

            su_an = time.time()
            gosterim = frame.copy()

            if bulundu:
                kose_ince = cv2.cornerSubPix(
                    gri, kose, (11, 11), (-1, -1),
                    (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                )
                cv2.drawChessboardCorners(gosterim, pattern_size, kose_ince, bulundu)

                if su_an - son_kayit_zamani >= args.min_interval:
                    objpoints.append(objp.copy())
                    imgpoints.append(kose_ince)
                    kayit_sayisi += 1
                    son_kayit_zamani = su_an
                    print(f'[KAYIT] {kayit_sayisi}/{args.target_frames}')
                    cv2.putText(gosterim, f'KAYDEDILDI {kayit_sayisi}/{args.target_frames}',
                                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
                else:
                    kalan = args.min_interval - (su_an - son_kayit_zamani)
                    cv2.putText(gosterim, f'TESPIT EDILDI - bekle {kalan:.1f}s / hareket ettir',
                                (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
            else:
                cv2.putText(gosterim, f'ARANIYOR... ({kayit_sayisi}/{args.target_frames} kayitli)',
                            (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

            ret_enc, buf = cv2.imencode('.jpg', gosterim, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
            if ret_enc:
                data = buf.tobytes()
                if len(data) < 65000:
                    try:
                        sock.sendto(data, (args.udp_ip, args.udp_port))
                    except Exception:
                        pass

    finally:
        cap.release()

    if kayit_sayisi < 4:
        print(f'HATA: yeterli kare toplanamadi ({kayit_sayisi}). Kalibrasyon YAPILMADI.')
        return

    print('Kalibrasyon hesaplaniyor...')
    rms, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, frame_size, None, None
    )

    fx, fy = mtx[0, 0], mtx[1, 1]
    cx, cy = mtx[0, 2], mtx[1, 2]

    print('--- KALIBRASYON SONUCU ---')
    print(f'Yeniden-izdusum hatasi (RMS, px, ne kadar dusukse o kadar iyi): {rms:.4f}')
    print(f'fx = {fx:.2f} px')
    print(f'fy = {fy:.2f} px')
    print(f'cx = {cx:.2f} px  cy = {cy:.2f} px')
    print(f'Kare/frame boyutu: {frame_size[0]}x{frame_size[1]}')

    sonuc = {
        'fx_px': float(fx),
        'fy_px': float(fy),
        'cx_px': float(cx),
        'cy_px': float(cy),
        'rms_reprojection_error_px': float(rms),
        'frame_width': frame_size[0],
        'frame_height': frame_size[1],
        'square_mm': args.square_mm,
        'frame_count': kayit_sayisi,
        'dist_coeffs': dist.flatten().tolist(),
    }
    with open(args.out, 'w') as f:
        json.dump(sonuc, f, indent=2)
    print(f'Kaydedildi: {args.out}')

    if rms > 1.0:
        print('UYARI: RMS hatasi 1.0 px\'in uzerinde - checkerboard duz/sabit degildi ya da '
              'kareler yeterince cesitli acidan gorulmedi olabilir. Tekrar denemek isteyebilirsin.')


if __name__ == '__main__':
    main()
