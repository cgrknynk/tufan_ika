#!/usr/bin/env python3
"""İZOLE KAMERA TESTİ (2026-09-04, kullanıcı isteği: "kamera görüntüsünün
kasma sebebi göndermeden mi yoksa çekmeden mi olduğunu anlamak için sadece
kamera görüntüsünü gösterecek bir kod yaz").

Amaç: arayüzün TÜM diğer yükünü (PyQt penceresi, harita, telemetri, joystick,
NTRIP, diğer kameralar, 4x QPainter bindirmesi vb.) DENKLEMDEN TAMAMEN
ÇIKARIP, SADECE "UDP'den JPEG oku -> ekrana bas" işini yapan minimal bir
araç. cv2.imshow kullanıyor - PyQt YOK, olay döngüsü YOK, başka hiçbir
iş parçacığı YOK.

Nasıl yorumlanır:
  - Bu script DE kasıyorsa/geç geliyorsa -> sorun ARAÇ TARAFINDA (gönderim:
    kameranın kendi FPS'i, YOLO/işleme süresi, JPEG encode, WiFi/ağ) - arayüz
    tarafında yapılacak hiçbir optimizasyon bunu düzeltemez.
  - Bu script AKICI çalışıyorsa -> sorun ARAYÜZ TARAFINDAYDI (çekme/render
    işlem hattı) - kamera_sistemi.py/main.py'de hâlâ iyileştirilecek bir yer
    var demektir.

Ekranda pencerenin başlığında ANLIK ve ORTALAMA kare-arası süre (ms) yazar;
konsola da her kare geldiğinde zaman damgası + aradaki süre basılır, ayrıca
ortalamanın 3 katından uzun süren (yani "duraksama" gösteren) aralıklar
"!!! GECİKME" diye ayrıca işaretlenir - böylece sadece göz kararıyla değil,
somut sayılarla da karşılaştırma yapılabilir.

Kullanım:
  python3 test_kamera_izole.py            # varsayılan: silah/turret (5001)
  python3 test_kamera_izole.py --port 5000 # ön (tabela) kamerası
  python3 test_kamera_izole.py --port 5002 # arka kamera
  'q' tuşuna basınca ya da Ctrl+C ile çıkar.
"""
import argparse
import select
import socket
import time

import cv2
import numpy as np

PORT_ISIMLERI = {5000: "ON (tabela)", 5001: "SİLAH (turret)", 5002: "ARKA"}


def yeni_soket(ip, port):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((ip, port))
    s.setblocking(False)
    return s


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--ip", default="0.0.0.0", help="Bu makinede dinlenecek IP (genelde 0.0.0.0)")
    ap.add_argument("--port", type=int, default=5001,
                     help="5000=ON, 5001=SİLAH, 5002=ARKA (kamera_sistemi.py ile aynı atama)")
    args = ap.parse_args()

    isim = PORT_ISIMLERI.get(args.port, f"port {args.port}")
    sock = yeni_soket(args.ip, args.port)
    pencere = f"IZOLE TEST - {isim} ({args.port})"
    cv2.namedWindow(pencere, cv2.WINDOW_NORMAL)

    print(f"[*] {isim} kamerası dinleniyor: {args.ip}:{args.port}")
    print("[*] 'q' tuşuna bas ya da Ctrl+C ile çık.\n")

    son_kare_zamani = None
    aralik_gecmisi = []  # son N aralık, ortalama/jitter icin
    kare_sayaci = 0
    baslangic = time.time()

    try:
        while True:
            hazir, _, _ = select.select([sock], [], [], 0.5)
            if not hazir:
                continue

            # Aynı drain-to-latest mantığı (kamera_sistemi.py'deki
            # düzeltmeyle TUTARLI) - soket boşaltılır, sadece en yeni
            # paket kullanılır. Bu ARAYÜZ tarafı optimizasyonu - araç
            # tarafındaki gecikmeyi ETKİLEMEZ, sadece bu test aracının
            # KENDİSİ backlog biriktirip yanlış sonuç vermesin diye.
            data = None
            while True:
                try:
                    data, _addr = sock.recvfrom(65535)
                except OSError:
                    break
            if data is None:
                continue

            simdi = time.time()
            if son_kare_zamani is not None:
                aralik_ms = (simdi - son_kare_zamani) * 1000.0
                aralik_gecmisi.append(aralik_ms)
                if len(aralik_gecmisi) > 200:
                    aralik_gecmisi.pop(0)
                ortalama = sum(aralik_gecmisi) / len(aralik_gecmisi)
                bayrak = ""
                if len(aralik_gecmisi) > 10 and aralik_ms > ortalama * 3:
                    bayrak = "  !!! GECİKME (ani duraksama)"
                print(f"[{simdi - baslangic:7.2f}s] kare#{kare_sayaci:5d}  "
                      f"aralık={aralik_ms:6.1f}ms  ort={ortalama:6.1f}ms{bayrak}")
            son_kare_zamani = simdi

            kare = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
            if kare is None:
                continue
            kare_sayaci += 1

            ortalama_txt = f"{sum(aralik_gecmisi) / len(aralik_gecmisi):.1f}ms" if aralik_gecmisi else "..."
            cv2.setWindowTitle(
                pencere,
                f"{pencere} | kare #{kare_sayaci} | ort. aralık: {ortalama_txt}")
            cv2.imshow(pencere, kare)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()
        cv2.destroyAllWindows()
        if aralik_gecmisi:
            print(f"\n[*] Toplam {kare_sayaci} kare, ortalama aralık: "
                  f"{sum(aralik_gecmisi) / len(aralik_gecmisi):.1f}ms "
                  f"(~{1000.0 / (sum(aralik_gecmisi) / len(aralik_gecmisi)):.1f} fps)")
            print(f"[*] En uzun aralık: {max(aralik_gecmisi):.1f}ms")


if __name__ == "__main__":
    main()
