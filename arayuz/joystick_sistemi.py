from PyQt5.QtWidgets import QWidget, QVBoxLayout, QSlider, QLabel, QPushButton, QHBoxLayout
from PyQt5.QtCore import Qt, pyqtSignal

class SanalJoystickPaneli(QWidget):
    """
    Fiziksel donanım gelene kadar Arduino'yu taklit eden (Mock) Sanal Test Paneli.
    Donanım geldiğinde bu sınıfı pyserial kullanan bir QThread ile değiştireceğiz.
    """
    
    # Müdüre (main.py) veya Telemetriye gidecek sinyaller
    hareket_sinyali = pyqtSignal(str) 
    durum_sinyali = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("🛠️ TUFAN - Sanal Donanım Simülatörü")
        self.setFixedSize(350, 250)
        self.setStyleSheet("background-color: #2c3e50; color: white; font-weight: bold;")

        layout = QVBoxLayout()

        # --- ARAÇ İLERİ / GERİ EKSENİ (Y EKSENİ) ---
        self.lbl_arac_y = QLabel("Araç İleri / Geri (Y Ekseni): BOŞTA")
        self.lbl_arac_y.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.lbl_arac_y)

        # Slider (0 ile 1023 arası - 512 Orta Nokta)
        self.slider_y = QSlider(Qt.Horizontal)
        self.slider_y.setMinimum(0)
        self.slider_y.setMaximum(1023)
        self.slider_y.setValue(512) 
        self.slider_y.valueChanged.connect(self.y_ekseni_degisti)
        layout.addWidget(self.slider_y)

        # --- ARAÇ SAĞ / SOL EKSENİ (X EKSENİ) ---
        self.lbl_arac_x = QLabel("Araç Sağ / Sol (X Ekseni): BOŞTA")
        self.lbl_arac_x.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.lbl_arac_x)

        self.slider_x = QSlider(Qt.Horizontal)
        self.slider_x.setMinimum(0)
        self.slider_x.setMaximum(1023)
        self.slider_x.setValue(512) 
        self.slider_x.valueChanged.connect(self.x_ekseni_degisti)
        layout.addWidget(self.slider_x)

        # --- KOKPİT ŞALTERLERİ ---
        btn_layout = QHBoxLayout()
        self.btn_salter1 = QPushButton("Silah Güç Şalteri (KAPALI)")
        self.btn_salter1.setStyleSheet("background-color: darkred; padding: 10px; border-radius: 5px;")
        self.btn_salter1.setCheckable(True)
        self.btn_salter1.toggled.connect(self.salter_tetikle)
        btn_layout.addWidget(self.btn_salter1)
        layout.addLayout(btn_layout)

        self.setLayout(layout)

    def y_ekseni_degisti(self, deger):
        # Arduino'dan gelen 0-1023 verisini mantıksal komuta çeviriyoruz
        if deger > 800:
            self.lbl_arac_y.setText("Araç İleri / Geri: İLERİ GİDİYOR")
            self.hareket_sinyali.emit("İLERİ (JOYSTICK)")
        elif deger < 200:
            self.lbl_arac_y.setText("Araç İleri / Geri: GERİ GİDİYOR")
            self.hareket_sinyali.emit("GERİ (JOYSTICK)")
        else:
            self.lbl_arac_y.setText("Araç İleri / Geri: BOŞTA")

    def x_ekseni_degisti(self, deger):
        if deger > 800:
            self.lbl_arac_x.setText("Araç Sağ / Sol: SAĞA DÖNÜYOR")
            self.hareket_sinyali.emit("SAĞA DÖN (JOYSTICK)")
        elif deger < 200:
            self.lbl_arac_x.setText("Araç Sağ / Sol: SOLA DÖNÜYOR")
            self.hareket_sinyali.emit("SOLA DÖN (JOYSTICK)")
        else:
            self.lbl_arac_x.setText("Araç Sağ / Sol: BOŞTA")

    def salter_tetikle(self, durum):
        if durum:
            self.btn_salter1.setStyleSheet("background-color: green; padding: 10px; border-radius: 5px;")
            self.btn_salter1.setText("Silah Güç Şalteri (AÇIK)")
            self.durum_sinyali.emit("DONANIM: Silah Kulesi aktif edildi!")
        else:
            self.btn_salter1.setStyleSheet("background-color: darkred; padding: 10px; border-radius: 5px;")
            self.btn_salter1.setText("Silah Güç Şalteri (KAPALI)")
            self.durum_sinyali.emit("DONANIM: Silah Kulesi pasif.")