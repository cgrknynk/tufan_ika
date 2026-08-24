#!/usr/bin/env python3
"""goal_manager_node için son çare canlılık bekçisi.

GERCEK ARAC PORTU (kaynak: ros2_ws/src/tufan_simulation/scripts/
goal_manager_watchdog.py). Mantik AYNEN korunuyor, hicbir uyarlama yok.

goal_manager_node.py, canlı testlerde bazen (kök nedeni bu ortamda kesin
olarak izole edilemeyen, muhtemelen rclpy/ActionClient seviyesinde nadir bir
executor kilitlenmesi) TÜM callback'lerinin (timer ve subscription) tetiklenmeyi
durdurduğu bir duruma düşüyor: process CPU tüketimi sıfıra iner, hiçbir yeni
hedef veya geri bildirim işlenmez, ama process kendisi ÇÖKMEZ (bu yüzden
launch'ın respawn=True'su tek başına yeterli değil - hiçbir zaman "exit"
etmiyor ki yeniden başlatılsın).

Bu node, goal_manager_node'un her _tick()'te yayınladığı /goal_manager_heartbeat
sayacını izler. Sayaç belirli bir süre artmazsa, goal_manager_node.py process'ini
bulup SIGTERM gönderir; launch dosyasındaki respawn=True bunu ardından otomatik
olarak yeniden başlatır.
"""
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class GoalManagerWatchdog(Node):

    def __init__(self):
        super().__init__('goal_manager_watchdog')
        self.declare_parameter('timeout_sec', 6.0)
        self.declare_parameter('check_period_sec', 1.0)
        self._timeout = self.get_parameter('timeout_sec').value
        self._last_count = None
        self._last_change_time = time.monotonic()

        self.create_subscription(Int32, '/goal_manager_heartbeat', self._on_heartbeat, 10)
        self.create_timer(self.get_parameter('check_period_sec').value, self._check)
        self.get_logger().info(
            f'goal_manager_watchdog hazir (timeout={self._timeout}s)')

    def _on_heartbeat(self, msg: Int32):
        if msg.data != self._last_count:
            self._last_count = msg.data
            self._last_change_time = time.monotonic()

    def _check(self):
        if self._last_count is None:
            # Henuz ilk heartbeat gelmedi (baslangic), bekle.
            return
        stale_for = time.monotonic() - self._last_change_time
        if stale_for > self._timeout:
            self.get_logger().error(
                f'goal_manager_node {stale_for:.1f}s dir yanit vermiyor '
                '(heartbeat donmus) - process sonlandiriliyor, launch '
                'yeniden baslatacak.')
            self._kill_goal_manager()
            self._last_change_time = time.monotonic()

    def _kill_goal_manager(self):
        try:
            out = subprocess.run(
                ['pgrep', '-f', 'goal_manager_node.py'],
                capture_output=True, text=True, timeout=2.0)
            pids = [p for p in out.stdout.split() if p.strip()]
            for pid in pids:
                subprocess.run(['kill', '-TERM', pid], timeout=2.0)
                self.get_logger().warn(f'SIGTERM gonderildi: PID {pid}')
        except Exception as e:
            self.get_logger().error(f'goal_manager_node sonlandirilamadi: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = GoalManagerWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
