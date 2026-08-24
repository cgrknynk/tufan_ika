#!/usr/bin/env python3
import os
import time
try:
    import rclpy
    from std_msgs.msg import String
except Exception as e:
    rclpy = None
    String = None
    ROS2_IMPORT_ERROR = str(e)
else:
    ROS2_IMPORT_ERROR = None

try:
    import serial
except Exception:
    serial = None


class TurretCmdListener:
    def __init__(self, serial_port='/dev/ttyACM1', baudrate=115200, ros_domain_id=10):
        os.environ.setdefault("ROS_DOMAIN_ID", str(ros_domain_id))
        os.environ.setdefault("ROS_LOCALHOST_ONLY", "0")
        self.serial_port = serial_port
        self.baudrate = baudrate
        self.ser = None
        self._serial_baglan()

    def _serial_baglan(self):
        if serial is None:
            print("[LISTENER] pyserial bulunamadı, seri bağlantı kurulamıyor.")
            return
        try:
            self.ser = serial.Serial(self.serial_port, self.baudrate, timeout=0.1)
            time.sleep(2)
            print(f"[LISTENER] Seri port bağlandı: {self.serial_port}@{self.baudrate}")
        except Exception as e:
            print(f"[LISTENER] Seri port bağlanamadı: {e}")

    def cb(self, msg):
        komut = msg.data.strip()
        print(f"[LISTENER] Gelen komut: {komut}")
        if self.ser is not None:
            try:
                self.ser.write((komut + "\n").encode('utf-8'))
                print(f"[LISTENER] Arduino'ya gönderildi: {komut}")
            except Exception as e:
                print(f"[LISTENER] Seri yazma hatası: {e}")
        else:
            print("[LISTENER] Seri port yok, sadece loglandı.")


def main():
    if rclpy is None or String is None:
        print("[LISTENER] ROS2 bulunamadığı için dinleyici başlatılamadı.")
        return

    rclpy.init()
    node = rclpy.create_node('turret_cmd_vel')
    listener = TurretCmdListener()

    node.create_subscription(String, '/turret_cmd_vel', listener.cb, 10)
    print("[LISTENER] /turret_cmd_vel dinleniyor...")
    rclpy.spin(node)


if __name__ == '__main__':
    main()
