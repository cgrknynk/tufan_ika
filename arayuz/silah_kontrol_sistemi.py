import os
import sys
from PyQt5.QtCore import Qt, QTimer, QObject, pyqtSignal
from PyQt5.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String as StdString
except Exception:  # ROS2 kurulu değilse bile dosya çalışsın
    rclpy = None
    Node = None
    StdString = None


class SilahKontrolSistemi(QObject):
    """Yön tuşlarıyla silahı kontrol eden ve ROS2 üzerinden /turret_cmd_vel topic'ine veri gönderen sınıf."""

    komut_sinyali = pyqtSignal(str, int)

    def __init__(self, parent=None, topic_name="/turret_cmd_vel", ros_domain_id=None):
        super().__init__(parent)
        self.aktif_yonlar = set()
        self.adim_boyu = 1
        self.publisher = None
        self.ros_durumu = "ROS2 bekleniyor"
        self.topic_name = topic_name
        # ros_domain_id AÇIKÇA verilmediyse SİSTEMİN VARSAYILAN DOMAIN'İNE DOKUNMA!
        # Önceden burada verilmediğinde "10"a sabitleniyordu; bu da bu süreçte
        # oluşturulan TÜM ROS2 node'larını (silah, lidar, harita) gerçek
        # /scan, /bno055/imu yayıncılarının ve Nav2'nin bulunduğu domain 0'dan
        # tamamen izole ediyordu -> arayüzde hiç veri görünmüyordu.
        self.ros_domain_id = ros_domain_id
        if self.ros_domain_id is not None:
            self._ros_ortamina_ayarla()
        self._ros_baslat()

        self.hareket_zamani = QTimer(self)
        self.hareket_zamani.setInterval(50)
        self.hareket_zamani.timeout.connect(self._hareket_et)
        self.hareket_zamani.start()

    def _ros_ortamina_ayarla(self):
        os.environ["ROS_DOMAIN_ID"] = str(self.ros_domain_id)
        os.environ.setdefault("ROS_LOCALHOST_ONLY", "0")
        os.environ["ROS_LOCALHOST_ONLY"] = "0"

    def _ros_baslat(self):
        if rclpy is None or StdString is None:
            self.ros_durumu = "ROS2 bulunamadı - yayın yapılamadı"
            return

        try:
            if not rclpy.ok():
                rclpy.init()
            self._node = rclpy.create_node('silah_kontrol_sistemi')
            self.publisher = self._node.create_publisher(StdString, self.topic_name, 10)
            self.ros_durumu = f"ROS2 hazır - {self.topic_name}"
            print(f"[SILAH] ROS2 publisher hazır: {self.topic_name}")
        except Exception as e:
            self.ros_durumu = f"ROS2 hatası: {e}"
            print(f"[SILAH] ROS2 publisher kurulamadı: {e}")

    def yon_bas(self, yon):
        if yon not in self.aktif_yonlar:
            self.aktif_yonlar.add(yon)
            self._gonder(yon, self.adim_boyu)

    def yon_birak(self, yon):
        if yon in self.aktif_yonlar:
            self.aktif_yonlar.remove(yon)

    def dur(self):
        self.aktif_yonlar.clear()

    def _hareket_et(self):
        if not self.aktif_yonlar:
            return
        for yon in list(self.aktif_yonlar):
            self._gonder(yon, self.adim_boyu)

    def _gonder(self, yon, adim):
        self.komut_sinyali.emit(yon, adim)
        self._ros_yayinla(yon)

    def _ros_yayinla(self, yon):
        if self.publisher is not None and StdString is not None:
            try:
                msg = StdString()
                msg.data = yon
                self.publisher.publish(msg)
                print(f"[SILAH] {self.topic_name} -> {yon}")
            except Exception as e:
                print(f"[SILAH] ROS2 yayın hatası: {e}")
        else:
            print(f"[SILAH] ROS2 publisher yok: {yon}")


class SilahKontrolWidget(QWidget):
    """Yön tuşlarıyla silah kontrolünü test etmek için arayüz."""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Silah Kontrol Paneli")
        self.resize(380, 220)
        self.setFocusPolicy(Qt.StrongFocus)

        self.kontrol = SilahKontrolSistemi(self)
        self.kontrol.komut_sinyali.connect(self._komut_guncelle)

        self.durum_label = QLabel("Yön tuşlarını kullanın\nYukarı / Aşağı / Sol / Sağ")
        self.durum_label.setAlignment(Qt.AlignCenter)
        self.durum_label.setStyleSheet("font-size: 16px; padding: 12px;")

        layout = QVBoxLayout(self)
        layout.addWidget(self.durum_label)
        self.setLayout(layout)

    def keyPressEvent(self, event):
        yon = self._yon_oku(event.key())
        if yon:
            self.kontrol.yon_bas(yon)
            event.accept()
            return
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        yon = self._yon_oku(event.key())
        if yon:
            self.kontrol.yon_birak(yon)
            event.accept()
            return
        super().keyReleaseEvent(event)

    def showEvent(self, event):
        super().showEvent(event)
        self.setFocus()

    def closeEvent(self, event):
        self.kontrol.dur()
        super().closeEvent(event)

    def _yon_oku(self, key):
        yon_map = {
            Qt.Key_Up: "YUKARI",
            Qt.Key_Down: "ASAGI",
            Qt.Key_Left: "SOL",
            Qt.Key_Right: "SAG",
        }
        return yon_map.get(key)

    def _komut_guncelle(self, yon, adim):
        self.durum_label.setText(
            f"Son komut: {yon} ({adim})\n{self.kontrol.ros_durumu}"
        )


if __name__ == "__main__":
    app = QApplication(sys.argv)
    pencere = SilahKontrolWidget()
    pencere.show()
    sys.exit(app.exec_())
