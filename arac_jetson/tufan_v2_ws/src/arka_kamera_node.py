#!/usr/bin/env python3
"""Arka kamera - SADECE acar ve goruntuyu UDP ile yayinlar, HICBIR YOLO
modeli/tespit CALISTIRMAZ (2026-08-31, kullanici istegi: "sadece ac ve
goruntu yayinla"). tabela_node.py/turret_node.py'nin kamera-acma kismiyla
ayni deseni kullanir, model kismini hic icermez.
"""
import os
import socket
import time

import cv2
import rclpy
from rclpy.node import Node


def _kamera_indexini_otomatik_bul(arama_metni, usb_port_ipucu=None):
    """CANLI TESTTE BULUNDU (2026-09-01): sabit camera_index kirilgan -
    /dev/videoN numaralari USB yeniden-enumerasyonunda DEGISEBILIYOR, bu
    yuzden tabela modeli YANLISLIKLA arka kameraya baglanmisti. isminde
    arama_metni (kucuk/buyuk harf duyarsiz, orn. "C270") GECEN /dev/videoN
    GERCEK video-capture dugumlerini (/sys/.../videoN/index=="0") bulur.
    AYNI MODELDEN BIRDEN FAZLA kamera varsa (iki C270 gibi, ISIM ILE
    AYIRT EDILEMEZLER - ID_SERIAL bile Logitech tarafindan paylasilmis/
    sahte) usb_port_ipucu (fiziksel USB port yolunun TAM son segmenti,
    orn "1-2.2.3") ile filtrelenir - kablo AYNI FIZIKSEL PORTA takili
    kaldigi surece bu deger USB KESIF SIRASINDAN BAGIMSIZ SABIT kalir.
    Eslesme yoksa None doner (cagiran taraf camera_index parametresine
    geri duser). tabela_node.py'deki AYNI fonksiyonun kopyasidir (bu
    projede paylasilan modul deseni yok, bkz. arduino_uno_portu_bul)."""
    import glob
    adaylar = []  # (index, usb_port_adi)
    for isim_dosyasi in sorted(glob.glob('/sys/class/video4linux/video*/name')):
        try:
            with open(isim_dosyasi) as f:
                isim = f.read().strip()
        except OSError:
            continue
        if arama_metni.lower() not in isim.lower():
            continue
        video_dizin = os.path.dirname(isim_dosyasi)
        try:
            with open(os.path.join(video_dizin, 'index')) as f:
                if f.read().strip() != '0':
                    continue  # metadata dugumu, gercek capture DEGIL
            index = int(os.path.basename(video_dizin).replace('video', ''))
        except (OSError, ValueError):
            continue
        usb_port = None
        if usb_port_ipucu:
            try:
                device_link = os.path.realpath(os.path.join(video_dizin, 'device'))
                usb_port = os.path.basename(os.path.dirname(device_link))
            except OSError:
                pass
        adaylar.append((index, usb_port))

    if not adaylar:
        return None
    if usb_port_ipucu:
        # TAM esitlik VEYA usb_port_ipucu'nun bir "ust dal" (hub seviyesi)
        # olmasi - bkz. tabela_node.py'deki AYNI fonksiyonun notu (USB
        # yeniden-enumerasyonunda port yolunun son basamagi kayabiliyor).
        for index, usb_port in adaylar:
            if usb_port == usb_port_ipucu or (
                    usb_port and usb_port.startswith(usb_port_ipucu + '.')):
                return index
        return None
    return min(index for index, _ in adaylar)


class ArkaKameraNode(Node):
    def __init__(self):
        super().__init__('arka_kamera_node')

        # camera_index SADECE otomatik bulma (asagida) BASARISIZ olursa
        # kullanilir (yedek) - bkz. _kamera_indexini_otomatik_bul.
        self.declare_parameter('camera_index', 4)
        self.declare_parameter('kamera_arama_ismi', 'C270')
        # GORSEL OLARAK KESIN DOGRULANAN fiziksel USB port yolu (ARKA
        # kamera) - bkz. tabela_node.py'deki AYNI tarihli not. tabela'nin
        # kullandigi C270'ten FARKLI port. Kablo baska bir porta tasinirsa
        # GUNCELLENMELI - bu donanimda port numaralandirmasi TAM KARARLI
        # DEGIL (2026-09-01 gece: 1-2.2.3 -> 1-2.2.4 kaydigi GORSEL olarak
        # tekrar dogrulandi, bkz. tabela_node.py'deki ayni tarihli not).
        self.declare_parameter('kamera_usb_port', '1-2.2.4')
        self.declare_parameter('video_target_ip', '10.40.64.44')
        self.declare_parameter('video_target_port', 5002)  # tabela(5000)/turret(5001) ile cakismasin

        self._camera_index = self.get_parameter('camera_index').value
        self._kamera_arama_ismi = self.get_parameter('kamera_arama_ismi').value
        self._kamera_usb_port = self.get_parameter('kamera_usb_port').value
        self._video_target_ip = self.get_parameter('video_target_ip').value
        self._video_target_port = self.get_parameter('video_target_port').value

        self._sock_video = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._calisiyor = True

        if self._kamera_arama_ismi:
            otomatik_index = _kamera_indexini_otomatik_bul(
                self._kamera_arama_ismi, self._kamera_usb_port)
            if otomatik_index is not None:
                if otomatik_index != self._camera_index:
                    self.get_logger().info(
                        f'Kamera otomatik bulundu: "{self._kamera_arama_ismi}" '
                        f'(port={self._kamera_usb_port}) -> /dev/video{otomatik_index} '
                        f'(parametre camera_index={self._camera_index} yerine kullanildi).')
                self._camera_index = otomatik_index
            else:
                self.get_logger().warn(
                    f'"{self._kamera_arama_ismi}" (port={self._kamera_usb_port}) isminde '
                    f'kamera bulunamadi, camera_index parametresi (={self._camera_index}) '
                    'YEDEK olarak kullaniliyor.')

        cap = cv2.VideoCapture(self._camera_index, cv2.CAP_V4L2)
        # DUZELTME (2026-09-01, canli bulundu): turret (C922) + bu kamera
        # (C270) AYNI USB hub'ini HAM YUYV formatinda paylasinca, UCUNCU
        # kamera (tabela) icin USB izokron bant genisligi HIC kalmiyordu -
        # tabela MJPG'ye gecirilse bile No space left on device (klasik
        # UVC bant genisligi hatasi) aliyordu; arka_kamera_node durdurulup
        # dogrudan ffmpeg ile izole test edilince (turret tek basinayken
        # tabela BASARILI acildi) kesin kanitlandi. Bu kamera SADECE
        # izleme icin (model yok, dusuk oncelikli) - MJPG'ye gecirmek en
        # dusuk riskli secim, tabela icin gereken bant genisligini acar.
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not cap.isOpened():
            self.get_logger().error(f'Arka kamera acilamadi (index={self._camera_index})')
        self._cap = cap

        self._timer = self.create_timer(1.0 / 15.0, self._kare_gonder)

        self.get_logger().info(
            f'arka_kamera_node aktif: kamera index={self._camera_index}, '
            f'goruntu UDP ile {self._video_target_ip}:{self._video_target_port} adresine gonderiliyor '
            '(model/tespit YOK, sadece izleme)'
        )

    def _kare_gonder(self):
        if not self._calisiyor:
            return
        ret, frame = self._cap.read()
        if not ret:
            return
        ret_enc, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
        if not ret_enc:
            return
        data = buffer.tobytes()
        if len(data) < 65000:
            try:
                self._sock_video.sendto(data, (self._video_target_ip, self._video_target_port))
            except Exception:
                pass

    def destroy_node(self):
        self._calisiyor = False
        try:
            self._cap.release()
        except Exception:
            pass
        self._sock_video.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArkaKameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
