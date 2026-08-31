#!/usr/bin/env python3
"""Tabela (trafik levhasi) algilama ROS 2 dugumu.

tabela_yolo26/jetson_server_2kamerali.py'nin tabela-tespit kismindan
tasindi. Ayri bir kameradan (turret'in kamerasindan bagimsiz) surekli
YOLO tespiti yapar; en yuksek guvenli tespiti /tabela_tespit topic'ine
yayinlar ve isaretlenmis kareyi UDP ile bir goruntuleyiciye gonderir.

Bu dugum herhangi bir motor/tetikleyici kontrol etmez - sadece algilama
yapar; surus mantigina baglamak istenirse /tabela_tespit dinlenerek
ayri bir karar dugumunde (bkz. tabela_etap_yoneticisi.py) yapilmalidir.

MODEL AC/KAPA (2026-08-31, gorev sekansi): /tabela_model_aktif (Bool)
dinler - KAPALI iken kamera ACIK KALIR (cap.read() calismaya devam eder,
UDP goruntu akisi kesilmez) ama YOLO predict() cagrilmaz ve /tabela_tespit'e
yayin YAPILMAZ. Gorev sekansinda Stop tabelasindan sonra "kamerayi kapatma,
sadece modeli durdur" ihtiyacini karsilar (bkz. tabela_etap_yoneticisi.py).
"""

import os
import socket
import threading
import time

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Bool, String
from ultralytics import YOLO

# Varsayilan model yolu: launch dosyasi olmadan (ör. "ros2 run tufan_v2_ws
# tabela_node.py") dogrudan calistirildiginda da model_path bos kalmasin diye.
# tabela_ana.engine (TensorRT, tabela_yolo26/okubeni.txt'de belirtilen
# "kullanilacak model") - .pt'den DAHA HIZLI cikarim icin (2026-08-31,
# kullanici istegi). .engine onceden derlenmis oldugu icin .to('cuda')
# GEREKMEZ (asagida kosullu atlaniyor).
_VARSAYILAN_MODEL_YOLU = os.path.join(
    get_package_share_directory('tufan_v2_ws'), 'models', 'tabela_ana.engine'
)


class TabelaNode(Node):
    def __init__(self):
        super().__init__('tabela_node')

        self.declare_parameter('camera_index', 0)
        self.declare_parameter('model_path', _VARSAYILAN_MODEL_YOLU)
        self.declare_parameter('hedef_confidence', 0.5)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('video_target_ip', '10.40.64.44')
        self.declare_parameter('video_target_port', 5001)

        self._camera_index = self.get_parameter('camera_index').value
        self._model_path = self.get_parameter('model_path').value
        self._conf = self.get_parameter('hedef_confidence').value
        self._imgsz = self.get_parameter('imgsz').value
        self._video_target_ip = self.get_parameter('video_target_ip').value
        self._video_target_port = self.get_parameter('video_target_port').value

        self._tespit_pub = self.create_publisher(String, 'tabela_tespit', 10)
        self._sock_video = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.get_logger().info(f'YOLO modeli yukleniyor: {self._model_path}')
        self._model = YOLO(self._model_path, task='detect')
        if not str(self._model_path).endswith(('.engine', '.onnx')):
            self._model.to('cuda')

        self._model_aktif = True
        self.create_subscription(Bool, '/tabela_model_aktif', self._model_aktif_cb, 10)

        self._calisiyor = True
        threading.Thread(target=self._ana_dongu, daemon=True).start()

        self.get_logger().info(
            'tabela_node aktif: tabela_tespit yayinlaniyor, goruntu UDP ile '
            f'{self._video_target_ip}:{self._video_target_port} adresine gonderiliyor'
        )

    def _model_aktif_cb(self, msg: Bool):
        yeni = bool(msg.data)
        if yeni != self._model_aktif:
            self.get_logger().info(f'tabela modeli {"AKTIF" if yeni else "DURDURULDU (kamera acik kalir)"}')
        self._model_aktif = yeni

    def _ana_dongu(self):
        cap = cv2.VideoCapture(self._camera_index, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not cap.isOpened():
            self.get_logger().error(f'Tabela kamerasi acilamadi (index={self._camera_index})')

        try:
            while self._calisiyor and rclpy.ok():
                ret, frame = cap.read()
                if not ret:
                    time.sleep(0.1)
                    continue

                if not self._model_aktif:
                    # Model KAPALI: sadece ham kareyi UDP'ye gonder, YOLO
                    # calistirma / /tabela_tespit'e yayin yok - kamera
                    # yine de ACIK/canli kalsin diye.
                    ret_enc, buffer = cv2.imencode(
                        '.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
                    if ret_enc:
                        data = buffer.tobytes()
                        if len(data) < 65000:
                            try:
                                self._sock_video.sendto(
                                    data, (self._video_target_ip, self._video_target_port))
                            except Exception:
                                pass
                    continue

                try:
                    results = self._model.predict(
                        source=frame, imgsz=self._imgsz, conf=self._conf, half=True, verbose=False
                    )
                    result = results[0]
                    cizili_frame = result.plot()

                    if len(result.boxes) > 0:
                        # En yuksek guvenli tespiti bildir
                        confs = result.boxes.conf.cpu().numpy()
                        en_iyi_idx = int(confs.argmax())
                        cls_id = int(result.boxes.cls[en_iyi_idx].item())
                        cls_adi = self._model.names.get(cls_id, str(cls_id))
                        guven = float(confs[en_iyi_idx])
                        self._tespit_pub.publish(String(data=f'{cls_adi}:{guven:.2f}'))
                    else:
                        self._tespit_pub.publish(String(data='YOK'))

                    ret_enc, buffer = cv2.imencode('.jpg', cizili_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
                    if ret_enc:
                        data = buffer.tobytes()
                        if len(data) < 65000:
                            try:
                                self._sock_video.sendto(data, (self._video_target_ip, self._video_target_port))
                            except Exception:
                                pass

                except Exception as e:
                    self.get_logger().error(f'Kare isleme hatasi, atlaniyor: {e}')
                    time.sleep(0.2)

        finally:
            cap.release()

    def destroy_node(self):
        self._calisiyor = False
        self._sock_video.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TabelaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
