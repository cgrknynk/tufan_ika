# TUFAN Yer Kontrol İstasyonu (Arayüz) — Teknik Dokümantasyon

Bu doküman, arayüz Jetson'da çalışan PyQt5 tabanlı yer kontrol istasyonu (GCS)
uygulamasının mimarisini, kullanılan kütüphaneleri, veri akışlarını ve kod
organizasyonunu detaylı şekilde açıklar.

## 1. Genel Mimari

Sistem iki ayrı Jetson üzerinde çalışır:

- **Arayüz Jetson** (bu doküman bunu anlatıyor): PyQt5 tabanlı yer kontrol
  istasyonu, tam ekran çalışır, hiçbir zaman araç Jetson'a fiziksel monitör
  bağlanmadığı için tüm izleme/kontrol buradan yapılır.
- **Araç Jetson** (`10.40.64.43`): gerçek otonom sürüş yığını (Nav2/MPPI,
  `goal_manager_node`), manuel sürüş köprüsü (`arduino_motor_kontrol.py`),
  IMU (`bno055`), LiDAR (`sllidar`), silah/turret kontrolü (`turret_node.py`)
  burada çalışır. Arayüz, ROS2 ağı üzerinden (aynı `ROS_DOMAIN_ID`, aynı LAN)
  bu Jetson'daki node'lara doğrudan abone olur/yayın yapar.

İki makine arasında üç farklı iletişim kanalı kullanılır:

1. **ROS2 (DDS)** — telemetri, sürüş komutları, harita/lidar/IMU/GNSS verisi
2. **UDP (ham soket)** — kamera görüntüleri (JPEG kare akışı)
3. **SSH** — arayüzdeki gömülü terminal üzerinden araç Jetson'a doğrudan
   komut satırı erişimi

## 2. Thread Mimarisi

PyQt5'in ana (GUI) thread'i asla bloklanmamalı; bu yüzden her sürekli veri
kaynağı kendi `QThread`'i içinde çalışır ve GUI'ye sadece `pyqtSignal` ile
(thread-safe) veri gönderir:

| Thread/Sınıf | Dosya | Görevi |
|---|---|---|
| `TelemetriThread` | `telemetri_sistemi.py` | Genel telemetri: hız, batarya, IMU/LiDAR/motor heartbeat, sürüş modu, manuel PWM yayını |
| `Ros2GcsMotoru` | `harita_sistemi.py` | Navigasyon: LiDAR, costmap, IMU (yön/pitch), GNSS, Nav2 plan/goal |
| `LidarThread` | `main.py` | Ana menüdeki dekoratif radar çemberi için ayrı, izole `/scan` aboneliği |
| `KameraThread` | `kamera_sistemi.py` | UDP üzerinden gelen JPEG kamera karelerini çözüp `QImage`'e çevirir |
| `SurusJoystickThread` | `surus_joystick_sistemi.py` | Fiziksel joystick Arduino'sundan seri port okuma |
| `SshTerminalWidget` içindeki pty okuyucu | `terminal_widget.py` | SSH oturumunun `pty` çıktısını okur |

**Neden `Ros2GcsMotoru` ve `TelemetriThread` ayrı ROS2 node'ları?**
Aynı `rclpy` executor'ına birden fazla thread aynı anda `spin` ederse
(`IndexError: wait set index too big` hatası canlı olarak gözlendi) çöküyor.
Her thread kendi `SingleThreadedExecutor`'ını kullanarak izole ediliyor.

## 3. Kullanılan Kütüphaneler — Ne İşe Yarar, Nasıl Kullanılır

Özet tablo:

| Kütüphane | Neden kullanıldı |
|---|---|
| **PyQt5** | Ana GUI çatısı — widget'lar, `QThread`/`pyqtSignal` ile thread-safe cross-thread haberleşme, `QPainter` ile özel çizim (radar, uydu harita, kamera overlay) |
| **rclpy** (ROS2 Humble) | Araç Jetson'daki gerçek sürüş yığınıyla aynı ROS2 ağında konuşmak — telemetri, sürüş komutları, sensör verisi |
| **OpenCV (cv2)** | UDP üzerinden gelen JPEG baytlarını (`cv2.imdecode`) çözüp `QImage`'e çevirmek için |
| **NumPy** | OpenCV'nin bayt dizilerini işlemesi için (`np.frombuffer`) |
| **pyte** | Saf Python VT100/ANSI terminal emülatörü — gömülü SSH terminalinin renk/imleç/ekran durumunu yönetir |
| **pyserial (`serial`)** | Joystick Arduino'sundan (CH340 üzerinden USB) seri veri okumak |
| **`pty` + `os` (Python stdlib)** | Gerçek bir SSH oturumunu (interaktif shell, sudo şifre sorma dahil) bir pseudo-terminal içinde çalıştırmak |
| **`QtNetwork` (QNetworkAccessManager)** | Uydu harita tile'larını (Esri World Imagery) asenkron, GUI'yi bloklamadan indirmek |
| **`socket` (UDP)** | Kamera görüntü akışını almak — düşük gecikme, bağlantısız, kare kaybı kabul edilebilir senaryo için TCP yerine tercih edildi |

Aşağıda her biri tek tek, ne olduğu ve bu projede gerçekte nasıl kullanıldığı
(gerçek kod örnekleriyle) açıklanıyor.

### 3.1 PyQt5

**Nedir?** Qt (C++ yazılmış, endüstri standardı bir GUI framework'ü) için
Python bağlayıcısı (binding). Qt'nin tüm sınıflarını (`QWidget`, `QThread`,
`QPainter` vb.) doğrudan Python'dan kullanmanı sağlar. Masaüstü uygulaması
yazmak için `tkinter`'dan çok daha güçlü/profesyonel bir alternatif.

PyQt5 birkaç alt modülden oluşur, bu projede üçü kullanılıyor:
- `PyQt5.QtWidgets` — görünür arayüz elemanları (buton, etiket, pencere...)
- `PyQt5.QtCore` — görsel olmayan çekirdek sınıflar (thread, sinyal, zamanlayıcı...)
- `PyQt5.QtGui` — çizim/görsel altyapı (`QPainter`, `QImage`, `QFont`...)
- `PyQt5.QtNetwork` — ağ istekleri (`QNetworkAccessManager`)

#### 3.1.1 Widget'lar ve pencere yapısı

Her görünür arayüz parçası bir `QWidget` alt sınıfıdır. Ana pencere
`QMainWindow`'dan türetilir:

```python
# main.py
class TufanGCS(QMainWindow):
    def __init__(self):
        super().__init__()
        self.ui = Ui_MainWindow()   # arayuz.py'de tanımlı, pyuic5 üretimi
        self.ui.setupUi(self)        # tüm widget'ları bu pencereye yerleştirir
```

`arayuz.py` içinde tipik bir widget oluşturma sözdizimi şöyle görünür
(Qt Designer'dan otomatik üretilmiştir):

```python
self.label_hizYazi = QtWidgets.QLabel(self.centralwidget)
self.label_hizYazi.setGeometry(QtCore.QRect(40, 20, 200, 40))
self.label_hizYazi.setStyleSheet("color: white; font-weight: bold;")
```

Bu projede ayrıca **elle** (Qt Designer olmadan, doğrudan Python'da) widget
üretimi de var — örn. `main.py`'deki silah kontrol sayfası eskiden böyle
kuruluyordu (artık kaldırıldı), joystick'in silah şalteri paneli hâlâ böyle:

```python
# joystick_sistemi.py (artık silinmiş örnek — üslup gösterimi için)
self.slider_y = QSlider(Qt.Horizontal)
self.slider_y.setMinimum(0)
self.slider_y.setMaximum(1023)
layout.addWidget(self.slider_y)
```

#### 3.1.2 Layout sistemi

Widget'ları elle piksel piksel konumlandırmak yerine (yukarıdaki
`setGeometry` örneği gibi), **layout** sınıfları otomatik dizilim sağlar —
pencere yeniden boyutlandığında içerik orantılı şekilde yeniden yerleşir.

```python
# main.py - kamera küçük önizleme kutusu için dikey layout örneği
self.kamera_layout_sag1 = QVBoxLayout()
self.tuval_sag1 = QLabel()
self.kamera_layout_sag1.addWidget(self.tuval_sag1)
```

`QVBoxLayout` (dikey), `QHBoxLayout` (yatay) ve `QGridLayout` (satır/sütun
tablosu) en çok kullanılanlar. Bir layout, bir konteyner widget'a
`QWidget(layout=...)` ya da `widget.setLayout(layout)` ile bağlanır.

#### 3.1.3 Sinyal & Slot (Signal & Slot) mekanizması

PyQt5'in en kritik özelliği: iki nesne (örn. bir düğme ve bir fonksiyon,
ya da bir arka plan thread'i ve GUI) arasında **gevşek bağlı** (loosely
coupled) iletişim kurar. Bir nesne bir "sinyal" (`pyqtSignal`) tanımlar,
başka bir nesne bu sinyale bir fonksiyon (`slot`) "bağlar" (`.connect()`).
Sinyal `.emit()` edildiğinde bağlı tüm fonksiyonlar otomatik çağrılır.

```python
# telemetri_sistemi.py - sinifin en ustunde sinyal TANIMI
class TelemetriThread(QThread):
    hedef_mesafe_sinyali = pyqtSignal(float)   # bir 'float' tasiyan sinyal

    def hedef_mesafe_cb(self, msg):
        self.hedef_mesafe_sinyali.emit(float(msg.data))   # sinyali GONDER
```

```python
# main.py - baska bir yerde sinyale bir fonksiyon BAGLA
self.telemetri_motoru.hedef_mesafe_sinyali.connect(self.hedef_mesafe_guncelle)

def hedef_mesafe_guncelle(self, mesafe):    # sinyal emit edilince OTOMATIK cagrilir
    self.son_hedef_mesafe = mesafe
```

Bu mekanizma **thread-safe**'tir — Qt, sinyal başka bir thread'den
gönderilse bile bağlı fonksiyonun (slot) her zaman kendi ait olduğu thread'de
(genelde GUI thread'i) çalışmasını garanti eder (`Qt.QueuedConnection`).
Bu yüzden arka planda çalışan `TelemetriThread`, `Ros2GcsMotoru` gibi
ROS2 thread'leri doğrudan widget'lara dokunmaz, her zaman sinyal üzerinden
"haber verir" — GUI güncellemesini ana thread kendi yapar.

Düğme tıklamaları da aynı mekanizmayla çalışır — her `QPushButton`'ın
hazır `clicked` sinyali vardır:

```python
self.ui.pushButton_joystick.clicked.connect(self.ayar_joystick_tetikle)
```

#### 3.1.4 QThread — arka plan iş parçacıkları

`QThread`'den türeyen bir sınıf, `run()` metodunu override eder; `.start()`
çağrıldığında bu metot **ayrı bir işletim sistemi thread'inde** çalışmaya
başlar, ana GUI thread'i bloklanmaz:

```python
# surus_joystick_sistemi.py
class SurusJoystickThread(QThread):
    pwm_sinyali = pyqtSignal(float, float)

    def run(self):
        while self._calisiyor:
            satir = ser.readline()      # bu satır YAVAS/BLOKLAYICI olabilir
            ...                          # ama GUI'yi DONDURMAZ, cunku ayri thread'de
            self.pwm_sinyali.emit(sol, sag)

# main.py - thread'i baslatmak
self.surus_joystick_motoru = SurusJoystickThread()
self.surus_joystick_motoru.start()   # run() ARKA PLANDA calismaya baslar
```

Bu projede ROS2'nin `rclpy.spin()` çağrısı da (sürekli, bloklayan bir
döngü) her zaman bir `QThread.run()` içinde yapılır — aksi halde GUI
tamamen kilitlenirdi.

#### 3.1.5 QPainter — özel çizim

Hazır widget'ların çizemeyeceği özel görseller (radar noktaları, uydu
harita, suni ufuk, yuvarlak köşeli kamera kutuları) `QPainter` ile elle
çizilir. Her `QWidget`'in `paintEvent(self, event)` metodu override
edilerek içine çizim kodu yazılır — Qt bu metodu ekran her yenilenmesi
gerektiğinde otomatik çağırır:

```python
# uydu_harita_widget.py (basitlestirilmis)
def paintEvent(self, event):
    ressam = QPainter(self)                          # bu widget uzerinde "firca" ac
    ressam.setRenderHint(QPainter.Antialiasing)       # kenarlari yumusat
    ressam.fillRect(self.rect(), QColor("#0d1117"))   # arka plani doldur
    ressam.drawPixmap(QPointF(px, py), tile_resmi)    # bir gorseli konuma ciz
    ressam.setPen(QPen(QColor(0, 200, 255, 200), 3))  # kalem rengi/kalinligi
    ressam.drawPolyline(poly)                          # iz cizgisini ciz
    ressam.save(); ressam.translate(x, y); ressam.rotate(aci)
    ressam.drawPolygon(ok_ucgeni)                       # dondurulmus arac oku
    ressam.restore()
```

`paintEvent` içinde çizim yapmak yerine, kod dışarıdan `widget.update()`
çağırarak "bir sonraki uygun anda yeniden çiz" talebinde bulunur — bu,
verimli, Qt'nin kendi zamanlamasına bırakılmış bir yaklaşımdır.

#### 3.1.6 QTimer

Belirli aralıklarla tekrar eden işler (örn. her 100ms'de bir PWM yayını,
her 3sn'de bir WiFi kontrolü) `QTimer` ile yapılır — Python'un `time.sleep`
döngüsü YERİNE, çünkü `sleep` GUI thread'inde kullanılırsa donmaya sebep olur:

```python
# telemetri_sistemi.py
self.node.create_timer(0.1, self.surekli_yayin_dongusu)   # ROS2 tarafi timer'i
```

```python
# main.py - Qt tarafi timer ornegi (OTONOM/MANUEL yazisi donusumlu gostermek icin)
self.mod_cycle_zamanlayici = QTimer()
self.mod_cycle_zamanlayici.timeout.connect(self._mod_cycle_tetikle)
self.mod_cycle_zamanlayici.start(2000)   # her 2000ms'de bir tetikle
```

#### 3.1.7 QtNetwork — asenkron ağ istekleri

`QNetworkAccessManager`, HTTP isteklerini (uydu harita tile indirme gibi)
**bloklamadan** yapar — istek gönderilir, cevap geldiğinde bir sinyal
(`finished`) otomatik tetiklenir:

```python
# uydu_harita_widget.py
self.net_yoneticisi = QNetworkAccessManager(self)
self.net_yoneticisi.finished.connect(self._tile_indi)   # cevap gelince cagrilacak fonksiyon

istek = QNetworkRequest(QUrl(url))
self.net_yoneticisi.get(istek)    # istegi GONDER, burada BEKLEME yok

def _tile_indi(self, reply):
    veri = reply.readAll()         # cevap hazir oldugunda calisir
    pix = QPixmap()
    pix.loadFromData(veri)
```

### 3.2 rclpy — ROS2 Python İstemcisi

**Nedir?** ROS2'nin (Robot Operating System 2) resmi Python API'si. ROS2,
birden fazla bağımsız programın ("node") birbirine `topic` adı verilen
isimlendirilmiş kanallar üzerinden mesaj gönderip alabildiği bir
"yayıncı/abone" (publisher/subscriber) mimarisi sunar — ağ üzerinden
otomatik keşif (discovery) yapar, aynı `ROS_DOMAIN_ID`'deki tüm node'lar
birbirini bulur.

Temel sözdizimi kalıbı — bir node oluşturmak, abone olmak, yayın yapmak:

```python
# telemetri_sistemi.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

rclpy.init()
self.node = Node('tufan_yer_istasyonu')          # bir ROS2 node'u olustur

# ABONE OL: bu topic'e mesaj geldikce hiz_cb otomatik cagrilir
self.node.create_subscription(Float32, '/arac_hiz', self.hiz_cb, 10)

# YAYINCI olustur
self.palet_pub = self.node.create_publisher(Float32MultiArray, '/palet_hizlari', 10)

# YAYIN YAP
msg = Float32MultiArray()
msg.data = [sol_pwm, sag_pwm]
self.palet_pub.publish(msg)
```

`10` parametresi **QoS (Quality of Service) kuyruk derinliği** — kaç
mesajın tampon belleğe alınacağı. Bazı kritik veriler (costmap gibi) için
daha detaylı QoS profili tanımlanır:

```python
# harita_sistemi.py
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
qos = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
```

Node'un mesaj almaya devam etmesi için sürekli "dinlemesi" (spin) gerekir
— bu bloklayan bir çağrıdır, bu yüzden her zaman ayrı bir `QThread` içinde
çalıştırılır (bkz. §3.1.4):

```python
def run(self):   # QThread.run()
    self.executor = SingleThreadedExecutor()
    self.executor.add_node(self.node)
    while rclpy.ok() and self.calisiyor:
        self.executor.spin_once(timeout_sec=0.1)
```

### 3.3 OpenCV (cv2)

**Nedir?** Görüntü işleme için endüstri standardı kütüphane (C++ çekirdek,
Python bağlayıcısı). Bu projede sadece basit iki işlem için kullanılıyor:
gelen JPEG baytlarını gerçek bir görüntüye çözmek ve renk formatını
çevirmek.

```python
# kamera_sistemi.py
frame = cv2.imdecode(np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
# 'data': UDP'den gelen ham JPEG baytlari -> 'frame': piksel matrisi (BGR sirali)

frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
# Qt'nin QImage'i RGB sirasi bekler, OpenCV varsayilan olarak BGR kullanir
```

Araç tarafında (`turret_node.py`) ise OpenCV çok daha fazla iş görüyor:
kameradan kare okuma (`cv2.VideoCapture`), YOLO'nun tespit kutusunu çizme
(`cv2.rectangle`), hedef noktasını işaretleme (`cv2.circle`), kareyi
tekrar JPEG'e sıkıştırma (`cv2.imencode`).

### 3.4 NumPy

**Nedir?** Python'da büyük sayı dizileriyle hızlı çalışmayı sağlayan temel
kütüphane. Burada tek kullanım amacı: ham baytları (UDP'den gelen JPEG
verisi) OpenCV'nin anlayacağı bir sayı dizisine çevirmek:

```python
np.frombuffer(data, dtype=np.uint8)   # bayt dizisini 0-255 araliginda tam sayi dizisine cevirir
```

### 3.5 pyte

**Nedir?** Saf Python ile yazılmış bir terminal emülatörü kütüphanesi —
bir terminal programının (bash, vim, htop...) ürettiği ham ANSI/VT100
kaçış (escape) kodu dizisini yorumlayıp "ekranda şu anda ne görünmesi
gerektiği" bilgisine (karakterler + renkler + imleç konumu) çevirir.
Gerçek bir terminal penceresi ÇİZMEZ — sadece durumu hesaplar, çizimi
`terminal_widget.py` kendi yapar.

```python
# terminal_widget.py
import pyte
self._ekran = pyte.Screen(80, 24)          # 80 sutun x 24 satirlik sanal ekran
self._akis = pyte.Stream(self._ekran)      # ham bayt -> ekran durumu ceviricisi

def _pty_veri_geldi(self, ham_veri):
    self._akis.feed(ham_veri.decode('utf-8', errors='replace'))
    # artik self._ekran.buffer icinde guncel karakter/renk durumu var
    for satir_no in self._ekran.dirty:      # sadece DEGISEN satirlari al
        ...
    self._ekran.dirty.clear()
```

### 3.6 pyserial (`serial` modülü)

**Nedir?** Python'dan seri port (USB-UART, RS232 vb.) okuma/yazma
kütüphanesi. Arduino gibi mikrodenetleyicilerle USB üzerinden metin/bayt
alışverişi yapmanın standart yolu.

```python
# surus_joystick_sistemi.py
import serial
ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1.0)   # portu ac
satir = ser.readline().decode('utf-8', errors='ignore')     # bir satir OKU (bloklayici)
ser.write(b"1500,1500\n")                                    # veri YAZ (arac Jetson tarafinda ornegi)
```

`timeout=1.0`: `readline()` en fazla 1 saniye bekler, veri gelmezse boş
döner (sonsuza kadar takılı kalmaz).

### 3.7 socket (Python stdlib) — UDP

**Nedir?** Python'un yerleşik ağ soketi modülü. Bu projede **UDP** (User
Datagram Protocol) modunda kullanılıyor — TCP'nin aksine bağlantısız,
sıra garantisi yok, kayıp paket kabul edilir ama gecikme çok düşük
(canlı video için TCP'den daha uygun).

```python
# kamera_sistemi.py
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)   # SOCK_DGRAM = UDP
s.bind((ip, port))            # bu port'u dinlemeye basla
s.setblocking(False)          # veri yoksa BEKLEME, hemen devam et

# select() ile ayni anda birden fazla soketi (silah + tabela) verimli dinleme
hazir, _, _ = select.select(soketler, [], [], 0.2)
for sok in hazir:
    data, adres = sok.recvfrom(65535)   # bir UDP paketi al (max 65535 bayt)
```

### 3.8 pty (Python stdlib)

**Nedir?** Unix/Linux'a özgü "pseudo-terminal" (sahte uçbirim) oluşturma
modülü. Bir programı (burada `ssh`) sanki gerçek bir terminalden
çalıştırılıyormuş gibi başlatmayı sağlar — bu sayede şifre sorma, renkli
çıktı, imleç kontrolü gibi normalde sadece gerçek terminallerde çalışan
davranışlar `subprocess` ile değil, `pty` ile doğru çalışır.

```python
# terminal_widget.py
import pty, os
pid, master_fd = pty.fork()
if pid == 0:                                    # cocuk surec (child process)
    os.execvp("ssh", ["ssh", "arf203@10.40.64.43"])
else:                                            # ebeveyn surec (parent)
    self._master_fd = master_fd                  # bu fd'den okuyup yazarak ssh oturumuyla konusulur
```

`master_fd`, Qt'nin `QSocketNotifier`'ına bağlanarak yeni veri geldiğinde
otomatik callback tetiklenmesi sağlanır (bkz. §5.5).

## 4. Dosya Yapısı ve Görevleri

```
main.py                    Ana pencere (TufanGCS) - tüm alt sistemleri birbirine bağlar
harita_sistemi.py          Navigasyon ekranı: taktik radar (LiDAR), suni ufuk (IMU), Nav2 entegrasyonu
telemetri_sistemi.py       Genel telemetri + manuel sürüş (klavye/joystick) PWM üretimi ve yayını
kamera_sistemi.py          UDP JPEG kamera alıcısı (silah/turret + tabela kanalları)
uydu_harita_widget.py      Esri uydu görüntüsü tabanlı harita widget'ı (ArduPilot/Mission Planner tarzı)
terminal_widget.py         Gömülü gerçek SSH terminali (pyte + pty)
surus_joystick_sistemi.py  Fiziksel joystick Arduino'sundan PWM üretimi
arayuz.py                  pyuic5 ile OTOMATİK ÜRETİLMİŞ statik UI düzeni (elle DÜZENLENMEZ)
kaynak_rc.py                pyrcc5 ile OTOMATİK ÜRETİLMİŞ, ikon/görsel ikili verisi
stiller.py                  Qt Style Sheet (CSS benzeri) tanımları - buton renk/durum stilleri
diller.py                   Türkçe/İngilizce arayüz metinleri (i18n sözlüğü)
turret_cmd_listener.py      (Kullanılmıyor / eski) /turret_cmd_vel dinleyip Arduino'ya ileten test scripti
```

## 5. Görsel Gösterim Mekanizmaları

### 5.1 Kamera Görüntüleri
- Araç Jetson'daki `turret_node.py` (silah kamerası) ve `tabela_node.py`,
  işlenmiş kareyi JPEG'e sıkıştırıp UDP ile arayüz Jetson'un IP'sine gönderir
  (`sock.sendto(jpeg_bytes, (hedef_ip, port))`).
- `kamera_sistemi.py`'deki `KameraThread`, iki UDP soketini (`port 5000`
  silah, `port 5001` tabela) `select.select()` ile aynı anda dinler.
- Gelen bayt dizisi `cv2.imdecode` ile OpenCV kare formatına, `cv2.cvtColor`
  ile BGR→RGB'ye, sonra `QImage`'e çevrilir ve `kare_sinyali` (dict: `{1, 2, 3}`
  anahtarlı) sinyaliyle GUI thread'ine gönderilir.
- `main.py`'deki `video_ekrana_bas`, bu sözlüğü alıp `resmi_yuvarla()` ile
  köşeleri yuvarlatılmış küçük önizleme kutularına (`QPainter` + `QPainterPath`
  clip) çizer, üzerine kamera adı (ve silah kamerasında hedef mesafesini,
  bkz. §6) `drawText` ile bindirir.
- **Kanal-etiket eşlemesi**: anahtar `1` = SİLAH (turret) gerçek verisi,
  anahtar `2` = TABELA verisi (ön kameradan hedef/tabela tespiti), anahtar
  `3` = henüz bağlanmamış ARKA kamera için ayrılmış (şu an hep boş).

### 5.2 Taktik Radar (LiDAR görselleştirme)
- `harita_sistemi.py`'deki `TaktikRadarEkrani` (`QFrame` alt sınıfı),
  `/scan` (`LaserScan`) mesajındaki açı/mesafe çiftlerini kutup
  koordinatlarından ekran piksel koordinatına çevirip `QPainter.drawPoint`
  ile nokta bulutu olarak çizer.
- Rolling-window costmap (`/local_costmap/costmap`, `OccupancyGrid`) da aynı
  ekranda yarı saydam katman olarak render edilir; robot her zaman grid
  merkezinde kabul edildiği için TF lookup'a ihtiyaç duyulmaz (önceden
  `tf_buffer` kullanımı kararsızlığa/çökmeye sebep olduğu için kaldırıldı).
- Derotasyon (aracın anlık yönüne göre nokta bulutunu döndürme) IMU'nun
  **ham** (düzeltilmemiş) quaternion'ından hesaplanan yaw ile yapılır —
  `konum_birlestirici.py`'nin `/odom` TF yayınında kullandığı referansla
  birebir eşleşmesi için.

### 5.3 Suni Ufuk (IMU görselleştirme)
- `SuniUfukEkrani`, `/imu/data` mesajından çıkarılan roll/pitch açılarını
  klasik uçak suni ufuk göstergesi tarzında çizer (`QPainter.rotate`/`translate`).

### 5.4 Uydu Haritası
- `uydu_harita_widget.py`, ArduPilot/Mission Planner tarzı bir slippy-map
  (kaydırılabilir/yakınlaştırılabilir uydu görüntüsü) widget'ıdır.
- Tile kaynağı: **Esri World Imagery** (`server.arcgisonline.com/.../MapServer/tile/{z}/{y}/{x}`),
  standart Web Mercator tile matematiği (`enlem_boylam_to_tile`) ile
  enlem/boylamdan tile indekslerine çevrilir.
- Tile indirme tamamen **asenkron** (`QNetworkAccessManager`), GUI thread'i
  bloklanmaz. İndirilen tile'lar hem RAM'de (`tile_onbellek` dict) hem
  diskte (`~/.cache/tufan_harita_tiles/`) önbelleğe alınır — disk önbelleği
  **süresiz** saklanır, otomatik yenilenmez (bkz. §8).
- Araç konumu `/fix` (`NavSatFix`) mesajından gelir, ekran merkezinde dönen
  bir ok + kat edilen yolun izi (`iz_noktalari` polyline) olarak çizilir.

### 5.5 Gömülü SSH Terminali
- `terminal_widget.py`, Python `pty` modülüyle gerçek bir `ssh` alt sürecini
  bir pseudo-terminal içinde başlatır (`os.fork`/`pty.fork` benzeri düşük
  seviye API).
- `pty`'nin master file descriptor'ı `QSocketNotifier` ile Qt event loop'una
  bağlanır — yeni veri geldiğinde callback tetiklenir (polling yok).
- Gelen ham bayt akışı **`pyte.Screen`** (saf Python VT100/ANSI emülatörü)
  ile işlenir; ANSI renk kodları, imleç hareketleri, ekran temizleme gibi
  tüm terminal davranışları burada yorumlanır.
- Performans: her bayt geldiğinde tüm ekranı yeniden çizmek yerine, sadece
  `pyte.Screen.dirty` ile işaretlenen (değişen) satırlar güncellenir, ayrıca
  16ms'lik bir debounce timer'ı ile art arda gelen çok sayıda küçük güncelleme
  tek bir render'da toplanır (200 karakterlik bir yazımda ~200 yerine ~8
  render — canlı ölçüldü).
- Bağlantı kesilse bile komut geçmişinin kalıcı olması için, oturum başında
  `export PROMPT_COMMAND='history -a'` gönderilir — her komut çalışır
  çalışmaz `~/.bash_history`'e anında yazılır.

## 6. Veri Kanalları — Hangi Veri Nereden Geliyor

### 6.1 ROS2 Topic'leri (araç Jetson ↔ arayüz Jetson, DDS üzerinden)

| Topic | Tip | Yön | Kullanıldığı yer |
|---|---|---|---|
| `/odom` | `nav_msgs/Odometry` | araç → arayüz | Hız göstergesi (`telemetri_sistemi.py`) |
| `/scan` | `sensor_msgs/LaserScan` | araç → arayüz | Taktik radar, ana menü dekoratif radar, LİDAR heartbeat |
| `/imu/data` | `sensor_msgs/Imu` | araç → arayüz | Suni ufuk, costmap derotasyonu, IMU heartbeat/gösterge |
| `/fix` | `sensor_msgs/NavSatFix` | araç → arayüz | Uydu haritada araç konumu (Here4 RTK planlanan kaynak) |
| `/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | araç → arayüz | Taktik radarda engel/costmap katmanı |
| `/plan`, `/local_plan` | `nav_msgs/Path` | araç → arayüz | Nav2 global/local rota görselleştirme |
| `/goal_manager_heartbeat` | `std_msgs/Int32` | araç → arayüz | OTONOM HAZIR göstergesi |
| `/arduino_baglanti_durumu` | `std_msgs/Bool` | araç → arayüz | MANUEL HAZIR göstergesi (motor Arduino seri bağlantısı açık mı) |
| `/turret_hedef_mesafe` | `std_msgs/Float32` | araç → arayüz | SİLAH kamerası kutusunda hedef mesafesi (TF02-Pro lidar, entegrasyon sürüyor) |
| `/goal_pose` | `geometry_msgs/PoseStamped` | arayüz → araç | RViz benzeri tıkla-git hedef gönderimi |
| `/surus_modu` | `std_msgs/String` (`"MANUEL"`/`"OTONOM"`) | arayüz → araç | Aracın hangi sürüş modunda olduğunu araca bildirir |
| `/palet_hizlari` | `std_msgs/Float32MultiArray` `[sol, sağ]` | arayüz → araç | MANUEL modda ham PWM komutu (klavye veya joystick kaynaklı) |
| `/arac_komut` | `std_msgs/String` | arayüz → araç | Ham klavye tuş kodu (teşhis/log amaçlı, PWM üretimi ayrı) |

### 6.2 UDP Kanalları (ham soket, ROS2 dışı)

| Port | İçerik | Kaynak |
|---|---|---|
| `5000` | Silah/turret kamerası, işlenmiş JPEG kare | `turret_node.py` (YOLO tespit kutusu çizili) |
| `5001` | Tabela/ön kamera JPEG kare | `tabela_node.py` |

UDP'nin tercih edilme sebebi: video akışında ROS2/DDS'in mesaj başlığı
overhead'i ve QoS karmaşıklığı yerine, kare kaybını kabul edip düşük
gecikmeyi önceliklendiren basit, bağlantısız bir protokol.

### 6.3 Seri Port (USB)

| Cihaz | Port (arayüz Jetson'da) | Protokol |
|---|---|---|
| Sürüş joystick'i (Arduino Mega + analog joystick) | `/dev/ttyUSB0` (CH340) | `"X,Y,BTN\n"` metin satırı, 115200 baud, 50Hz |

### 6.4 SSH

`terminal_widget.py`, `arf203@10.40.64.43`'e otomatik bağlanır, proje
klasörüne (`~/Desktop/tufan_v2_ws`) `cd` yapar — araç Jetson'a hiç monitör
bağlanmadığı için TÜM komut satırı erişimi buradan sağlanır.

## 7. Manuel Sürüş ve Güvenlik Mekanizmaları

- **Tek kaynak kuralı**: `telemetri_sistemi.py`'deki `surus_kaynagi` alanı
  (`"KLAVYE"` / `"JOYSTICK"`) aynı anda sadece TEK giriş kaynağının
  `/palet_hizlari`'a yazmasına izin verir; diğer kaynaktan gelen komutlar
  sessizce yok sayılır (SPACE/acil durdurma hariç — o her zaman çalışır).
- **Watchdog**: MANUEL modda, PWM sıfır değilken `son_komut_zamani` 0.8
  saniyeden uzun süre güncellenmezse (tuş bırakılmış/joystick veri
  akışı kesilmiş) paletler otomatik olarak sıfırlanır.
- **Joystick kalibrasyonu**: bağlantı kurulduğunda ilk ~0.5 saniye (25
  örnek) joystick'e dokunulmadığı varsayılıp gerçek elektriksel merkez
  otomatik ölçülür (bazı joystick modüllerinde merkez tam 512 olmayabiliyor,
  sabit 512 varsayımı yanlış komut üretebilirdi).
- **Mod bazlı erişim**: klavye/joystick komutları sadece `arac_modu ==
  "MANUEL"` iken etkilidir; OTONOM modda `surus_koprusu.py` (araç tarafı)
  Nav2 çıktısını `/palet_hizlari`'a çevirir, arayüz sessiz kalır.

## 8. Bilinen Sınırlamalar / Gelecek İşler

- Uydu harita disk önbelleği (`~/.cache/tufan_harita_tiles/`) süresiz
  saklanıyor, otomatik yenilenmiyor — Esri görüntüyü güncellese bile eski
  tile diskte kalırsa gösterilmeye devam eder (manuel temizlik gerekir).
- `/turret_hedef_mesafe` şu an araç tarafında gerçek bir yayıncısı olmayan,
  arayüz tarafında hazır bekleyen bir sözleşme — TF02-Pro lidar entegrasyonu
  tamamlanınca otomatik çalışacak.
- ARKA kamera için henüz fiziksel donanım/UDP kaynağı yok, kutu şimdilik boş.
- `turret_cmd_listener.py` ve eski `/turret_cmd_vel` topic'i artık
  kullanılmıyor (silah kontrolü arayüzden kaldırıldı, ileride ayrı bir
  joystick ile yeniden yapılacak).

---
*Bu doküman, `~/Desktop/tufan` altındaki kodun mevcut haline göre otomatik
olarak (kod incelemesiyle) hazırlanmıştır.*
