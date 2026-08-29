#!/usr/bin/env python3
"""Kamera kalibrasyonu icin satranc tahtasi (checkerboard) deseni uretir.

10x7 kare (9x6 IC kose) standart OpenCV kalibrasyon deseni. Uretilen PNG
ya yazicidan cikti alinip duz bir yuzeye yapistirilmali, ya da bir
telefon/tablet ekraninda TAM EKRAN gosterilmeli.

ONEMLI: PNG'deki kare boyutu SADECE goruntu icindir, gercek fiziksel
boyutu belirlemez (yazici olcegi / ekran cozunurlugu farkli olabilir).
Cikti aldiktan/gosterdikten SONRA cetvelle TEK bir karenin bir kenarini
mm cinsinden olcup bu degeri camera_calibrate.py --square-mm parametresine
ver.

Kullanim:
    python3 generate_checkerboard.py
    python3 generate_checkerboard.py --out checkerboard.png --square-px 120
"""

import argparse

import cv2
import numpy as np


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', default='checkerboard.png')
    parser.add_argument('--cols', type=int, default=10, help='Yatay kare sayisi (ic kose sayisi cols-1 olur)')
    parser.add_argument('--rows', type=int, default=7, help='Dikey kare sayisi (ic kose sayisi rows-1 olur)')
    parser.add_argument('--square-px', type=int, default=120, help='Her karenin piksel kenar uzunlugu')
    parser.add_argument('--border-px', type=int, default=100, help='Kenar bosluk (tespit icin sessiz bolge)')
    args = parser.parse_args()

    w = args.cols * args.square_px
    h = args.rows * args.square_px
    board = np.zeros((h, w), dtype=np.uint8)

    for r in range(args.rows):
        for c in range(args.cols):
            if (r + c) % 2 == 0:
                y0, y1 = r * args.square_px, (r + 1) * args.square_px
                x0, x1 = c * args.square_px, (c + 1) * args.square_px
                board[y0:y1, x0:x1] = 255

    canvas = np.full((h + 2 * args.border_px, w + 2 * args.border_px), 255, dtype=np.uint8)
    canvas[args.border_px:args.border_px + h, args.border_px:args.border_px + w] = board

    canvas_bgr = cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)
    ic_kose = f'{args.cols - 1}x{args.rows - 1}'
    cv2.putText(canvas_bgr, f'Ic kose: {ic_kose}  -  cikti/ekran SONRASI cetvelle 1 kareyi mm olc',
                (10, args.border_px - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

    cv2.imwrite(args.out, canvas_bgr)
    print(f'Yazildi: {args.out}  (boyut: {canvas_bgr.shape[1]}x{canvas_bgr.shape[0]} px)')
    print(f'Ic kose sayisi (camera_calibrate.py --cols --rows icin): cols={args.cols - 1} rows={args.rows - 1}')


if __name__ == '__main__':
    main()
