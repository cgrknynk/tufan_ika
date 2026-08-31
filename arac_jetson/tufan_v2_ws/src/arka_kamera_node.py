#!/usr/bin/env python3
"""Arka kamera - SADECE acar ve goruntuyu UDP ile yayinlar, HICBIR YOLO
modeli/tespit CALISTIRMAZ (2026-08-31, kullanici istegi: "sadece ac ve
goruntu yayinla"). tabela_node.py/turret_node.py'nin kamera-acma kismiyla
ayni deseni kullanir, model kismini hic icermez.
"""
import socket
import time

import cv2
import rclpy
from rclpy.node import Node


class ArkaKameraNode(Node):
    def __init__(self):
        super().__init__('arka_kamera_node')

        self.declare_parameter('camera_index', 2)
        self.declare_parameter('video_target_ip', '10.40.64.44')
        self.declare_parameter('video_target_port', 5002)  # tabela(5001)/turret(5000) ile cakismasin

        self._camera_index = self.get_parameter('camera_index').value
        self._video_target_ip = self.get_parameter('video_target_ip').value
        self._video_target_port = self.get_parameter('video_target_port').value

        self._sock_video = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._calisiyor = True

        cap = cv2.VideoCapture(self._camera_index, cv2.CAP_V4L2)
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
