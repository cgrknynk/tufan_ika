#!/usr/bin/env python3
"""Turret (hedef takip) ROS 2 dugumu.

Kamera + YOLO modeli ile hedefi tespit edip 2 eksenli turret'i (pan/tilt)
hedefe dogru yonlendirir.

ONEMLI: Bu dugum artik silah_ws/sketch_jul28b.ino'daki GERCEK Arduino
firmware'inin protokolune gore calisir:
  - Tek karakter komutlar: 'w' (yukari/tilt), 's' (asagi/tilt),
    'a' (sol/pan), 'd' (sag/pan), 'x' (dur). Satir sonu YOK.
  - Her komut sabit (firmware'de tanimli, ~50 adim) bir "durtme" yapar;
    surekli/orantili hareket YOK, ayni anda sadece TEK eksen hareket eder.
  - Bu firmware'de LAZER pini/komutu YOK - bu yuzden lazer/ates ozelligi
    su an devre disi (yalnizca nisan alma calisir). Lazer donanimi/firmware'i
    eklendiginde bu dugume geri eklenebilir.

Otonom modda: hedefin merkeze gore hangi yonde oldugu her donguide
hesaplanir, en buyuk hatali eksene tek bir yon komutu ('w'/'a'/'s'/'d')
gonderilir, merkeze yeterince yaklasinca 'x' (dur) gonderilir. Komutlar
firmware'in fiziksel hareket suresini asmamak icin belirli bir araliktan
daha sik gonderilmez (nudge_araligi).
"""

import os
import queue
import socket
import threading
import time

import cv2
import rclpy
import serial
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String
from ultralytics import YOLO

# Varsayilan model yolu: launch dosyasi olmadan (ör. "ros2 run tufan_v2_ws
# turret_node.py") dogrudan calistirildiginda da model_path bos kalmasin diye.
_VARSAYILAN_MODEL_YOLU = os.path.join(
    get_package_share_directory('tufan_v2_ws'), 'models', 'Target26last.pt'
)


class TurretNode(Node):
    def __init__(self):
        super().__init__('turret_node')

        self.declare_parameter('camera_index', 0)
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('model_path', _VARSAYILAN_MODEL_YOLU)
        self.declare_parameter('hedef_confidence', 0.15)
        self.declare_parameter('merkez_toleransi_px', 15)
        self.declare_parameter('nudge_araligi', 0.15)
        self.declare_parameter('komut_timeout', 0.5)
        self.declare_parameter('x_yonu_ters_mi', True)
        self.declare_parameter('y_yonu_ters_mi', True)
        self.declare_parameter('motor_eksenlerini_yer_degistir', False)
        self.declare_parameter('video_target_ip', '10.40.64.48')
        self.declare_parameter('video_target_port', 5000)

        self._camera_index = self.get_parameter('camera_index').value
        self._serial_port = self.get_parameter('serial_port').value
        self._baudrate = self.get_parameter('baudrate').value
        self._model_path = self.get_parameter('model_path').value
        self._conf = self.get_parameter('hedef_confidence').value
        self._merkez_tol = self.get_parameter('merkez_toleransi_px').value
        self._nudge_araligi = self.get_parameter('nudge_araligi').value
        self._timeout = self.get_parameter('komut_timeout').value
        self._x_ters = self.get_parameter('x_yonu_ters_mi').value
        self._y_ters = self.get_parameter('y_yonu_ters_mi').value
        self._eksen_yer_degistir = self.get_parameter('motor_eksenlerini_yer_degistir').value
        self._video_target_ip = self.get_parameter('video_target_ip').value
        self._video_target_port = self.get_parameter('video_target_port').value

        # --- Durum: guvenli varsayilan MANUEL - /silah_modu mesaji gelene
        # kadar otonom hedefleme ASLA baslamaz. ---
        self._mod = 'MANUEL'
        self._manuel_x = 0.0
        self._manuel_y = 0.0
        self._manuel_zaman = None

        self.create_subscription(String, 'silah_modu', self._mod_cb, 10)
        self.create_subscription(Twist, 'turret_manuel_cmd', self._manuel_cmd_cb, 10)

        self._sock_video = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self._arduino = None
        self._arduino_kuyruk = queue.Queue(maxsize=1)
        self._baglanti_kur()

        self.get_logger().info(f'YOLO modeli yukleniyor: {self._model_path}')
        self._model = YOLO(self._model_path, task='detect')
        self._model.to('cuda')

        self._calisiyor = True
        threading.Thread(target=self._arduino_yazici_thread, daemon=True).start()
        threading.Thread(target=self._ana_dongu, daemon=True).start()

        self.get_logger().info(
            'turret_node aktif (lazer devre disi - sadece nisan alma): '
            'silah_modu / turret_manuel_cmd dinleniyor'
        )

    def _baglanti_kur(self):
        try:
            self._arduino = serial.Serial(self._serial_port, self._baudrate, timeout=0)
            time.sleep(2)
            self.get_logger().info(f'Turret Arduino baglandi: {self._serial_port}')
        except Exception as e:
            self._arduino = None
            self.get_logger().error(f'Turret Arduino baglanti hatasi ({self._serial_port}): {e}')

    def _mod_cb(self, msg: String):
        yeni = msg.data.strip().upper()
        if yeni in ('OTONOM', 'MANUEL'):
            if yeni != self._mod:
                self.get_logger().info(f'silah modu degisti -> {yeni}')
            self._mod = yeni

    def _manuel_cmd_cb(self, msg: Twist):
        self._manuel_x = msg.linear.x
        self._manuel_y = msg.linear.y
        self._manuel_zaman = time.time()

    def _canli_mi(self, zaman):
        return zaman is not None and (time.time() - zaman) < self._timeout

    def _arduino_yazici_thread(self):
        while self._calisiyor:
            try:
                komut = self._arduino_kuyruk.get(timeout=0.5)
                if self._arduino and self._arduino.is_open:
                    self._arduino.write(komut.encode('utf-8'))
                self._arduino_kuyruk.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Seri port yazma hatasi: {e}')
                time.sleep(0.1)

    def _komut_gonder(self, karakter):
        """Tek karakterlik firmware komutunu (satir sonu OLMADAN) kuyruga koyar."""
        if self._arduino_kuyruk.full():
            try:
                self._arduino_kuyruk.get_nowait()
            except queue.Empty:
                pass
        self._arduino_kuyruk.put(karakter)

    def _yon_komutunu_hesapla(self, gx, gy):
        """Isaretli gx (pan) / gy (tilt) hatasindan tek eksenli firmware
        komutunu ('w'/'a'/'s'/'d'/'x') secer - buyuk mutlak hataya sahip
        eksen once duzeltilir (firmware ayni anda tek eksen kaldirir)."""
        if gx == 0 and gy == 0:
            return 'x'
        if abs(gx) >= abs(gy):
            return 'd' if gx > 0 else 'a'
        return 's' if gy > 0 else 'w'

    def _ana_dongu(self):
        cap = cv2.VideoCapture(self._camera_index)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not cap.isOpened():
            self.get_logger().error(f'Turret kamerasi acilamadi (index={self._camera_index})')

        aim_center_x, aim_center_y = 320, 240
        son_komut = 'x'
        son_nudge_zamani = 0.0

        try:
            while self._calisiyor and rclpy.ok():
                ret, frame = cap.read()
                if not ret:
                    time.sleep(0.1)
                    continue

                try:
                    su_an = time.time()
                    results = self._model(frame, imgsz=480, device=0, conf=self._conf, verbose=False)

                    hedef_bulundu = False
                    gx, gy = 0, 0

                    for result in results:
                        if len(result.boxes) > 0:
                            hedef_bulundu = True
                            box = result.boxes[0].xyxy[0].cpu().numpy()
                            tcx, tcy = int((box[0] + box[2]) / 2), int((box[1] + box[3]) / 2)

                            cv2.rectangle(frame, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 255, 0), 2)
                            ocx, ocy = tcx + 12, tcy - 40
                            cv2.circle(frame, (ocx, ocy), 5, (255, 0, 0), -1)

                            if self._mod == 'OTONOM':
                                ex, ey = aim_center_x - ocx, aim_center_y - ocy
                                if abs(ex) > self._merkez_tol:
                                    gx = ex
                                if abs(ey) > self._merkez_tol:
                                    gy = ey
                        break

                    if self._mod == 'OTONOM':
                        if self._x_ters:
                            gx = -gx
                        if self._y_ters:
                            gy = -gy
                        if self._eksen_yer_degistir:
                            gx, gy = gy, gx

                        if not hedef_bulundu:
                            gx, gy = 0, 0

                        komut = self._yon_komutunu_hesapla(gx, gy)
                    else:
                        # MANUEL: komut bir sure icinde gelmediyse guvenli sekilde dur
                        if self._canli_mi(self._manuel_zaman):
                            komut = self._yon_komutunu_hesapla(self._manuel_x, self._manuel_y)
                        else:
                            komut = 'x'
                        if komut != 'x':
                            cv2.putText(frame, 'MANUEL NISAN AKTIF', (50, 50), 1, 2, (255, 165, 0), 2)

                    # 'x' (dur): sadece harekete gecerken bir kez gonder.
                    # Hareket komutlari: yon degisince hemen, ayni yondeyse
                    # firmware'i bogmamak icin nudge_araligi kadar bekleyip tekrar gonder.
                    komut_degisti = komut != son_komut
                    zaman_doldu = (su_an - son_nudge_zamani) >= self._nudge_araligi
                    if (komut == 'x' and komut_degisti) or (komut != 'x' and (komut_degisti or zaman_doldu)):
                        self._komut_gonder(komut)
                        son_komut = komut
                        son_nudge_zamani = su_an

                    cv2.drawMarker(frame, (aim_center_x, aim_center_y), (0, 255, 255), 1, 40, 2)

                    ret_enc, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 40])
                    if ret_enc:
                        data = buffer.tobytes()
                        if len(data) < 65000:
                            try:
                                self._sock_video.sendto(data, (self._video_target_ip, self._video_target_port))
                            except Exception:
                                pass

                except Exception as e:
                    self.get_logger().error(f'Kare isleme hatasi, atlaniyor: {e}')
                    self._komut_gonder('x')
                    time.sleep(0.2)

        finally:
            self._komut_gonder('x')
            cap.release()

    def destroy_node(self):
        self._calisiyor = False
        self._komut_gonder('x')
        time.sleep(0.1)
        if self._arduino and self._arduino.is_open:
            self._arduino.close()
        self._sock_video.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TurretNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
