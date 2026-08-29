#!/usr/bin/env python3
"""Turret icin klavyeden manuel nisan alma testi.

turret_node.py 'MANUEL' modda /turret_manuel_cmd (geometry_msgs/Twist)
topic'ini dinliyor. Bu script klavyeden okudugu tuslari o mesaja cevirip
yayinlar. turret_node.py'nin kendi guvenlik zaman asimi (komut_timeout,
varsayilan 0.5s) sayesinde tus birakildiginda / script durduruldugunda
motor otomatik olarak durur - ayrica bu scriptte de her tus basisinda
sadece KISA bir "durtme" gonderilir, surekli motor calismaz.

Tuslar:
  w : yukari (tilt)
  s : asagi  (tilt)
  a : sola   (pan)
  d : saga   (pan)
  x / bosluk : hemen dur
  q / CTRL+C : cik (cikmadan once dur komutu gonderir)

Kullanim:
  cd ~/Desktop/tufan_v2_ws
  source /opt/ros/humble/setup.bash
  source install/setup.bash
  python3 src/tufan_v2_ws/tools/turret_keyboard_teleop.py
"""
import sys
import select
import termios
import tty

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

YON_HARITASI = {
    'w': (0.0, -1.0),   # yukari
    's': (0.0, 1.0),    # asagi
    'a': (-1.0, 0.0),   # sola
    'd': (1.0, 0.0),    # saga
    'x': (0.0, 0.0),    # dur
    ' ': (0.0, 0.0),    # dur
}

YARDIM_METNI = """
Turret klavye teleop - MANUEL nisan alma testi
------------------------------------------------
  w : yukari       s : asagi
  a : sola         d : saga
  x / bosluk : hemen dur
  q : cik (durup cikar)

Her tus basisinda TEK bir kisa durtme komutu gonderilir.
turret_node.py 0.5sn boyunca yeni komut gelmezse zaten otomatik durur.
------------------------------------------------
"""


def tus_oku(ayarlar, timeout=0.1):
    """Terminalden non-blocking tek karakter okur, basilan tus yoksa None doner."""
    hazir, _, _ = select.select([sys.stdin], [], [], timeout)
    if hazir:
        return sys.stdin.read(1)
    return None


class TurretKeyboardTeleop(Node):
    def __init__(self):
        super().__init__('turret_keyboard_teleop')
        self._pub = self.create_publisher(Twist, 'turret_manuel_cmd', 10)

    def komut_gonder(self, x, y):
        msg = Twist()
        msg.linear.x = x
        msg.linear.y = y
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = TurretKeyboardTeleop()

    ayarlar = termios.tcgetattr(sys.stdin)
    print(YARDIM_METNI)

    try:
        tty.setraw(sys.stdin.fileno())
        while True:
            tus = tus_oku(ayarlar, timeout=0.5)

            if tus is None:
                # Belirli araliklarla yeni tus gelmezse de guvenlik icin
                # acikca 'dur' gonderelim (redundant ama zararsiz).
                node.komut_gonder(0.0, 0.0)
                continue

            if tus == 'q':
                break

            if tus in YON_HARITASI:
                x, y = YON_HARITASI[tus]
                node.komut_gonder(x, y)
                sys.stdout.write(f"[TUS] '{tus}' okundu -> yayinlandi: linear.x={x} linear.y={y}\r\n")
                sys.stdout.flush()
            else:
                sys.stdout.write(f"[TUS] taninmayan tus: {tus!r} (kod={ord(tus)})\r\n")
                sys.stdout.flush()

    except Exception as e:
        print(f"\nHata: {e}")
    finally:
        node.komut_gonder(0.0, 0.0)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, ayarlar)
        node.destroy_node()
        rclpy.shutdown()
        print("\nCikildi, dur komutu gonderildi.")


if __name__ == '__main__':
    main()
