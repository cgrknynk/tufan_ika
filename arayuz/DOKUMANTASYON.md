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
ntrip_rtk_sistemi.py       NTRIP (Swift Navigation Skylark) -> mavros RTCM köprüsü (Cube/ArduPilot RTK düzeltmesi)
arayuz.py                  pyuic5 ile OTOMATİK ÜRETİLMİŞ statik UI düzeni (elle DÜZENLENMEZ)
kaynak_rc.py                pyrcc5 ile OTOMATİK ÜRETİLMİŞ, ikon/görsel ikili verisi
stiller.py                  Qt Style Sheet (CSS benzeri) tanımları - buton renk/durum stilleri
diller.py                   Türkçe/İngilizce arayüz metinleri (i18n sözlüğü)
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
- Derotasyon (aracın anlık yönüne göre nokta bulutunu döndürme), hedef
  tıklama dönüşümü (`mouseReleaseEvent`) ve araç ikonu dönüşü hepsi
  `self.latest_yaw_rad`'ı kullanır — bu, **BNO055 montaj düzeltmesi
  UYGULANMIŞ** yaw'dır (bkz. §5.2.1), `konum_birlestirici.py`'nin `/odom`
  TF'inde kullandığı referansla **birebir aynı** (canlı ölçümle
  doğrulandı: 362 örnekte ortalama/min/maks fark 0.00°). **2026-08-30
  ÖNCESİ** burada kasıtlı olarak HAM (düzeltmesiz) yaw kullanılıyordu,
  çünkü o tarihe kadar `/odom` da HAM yönelimle yayınlanıyordu — BNO055'in
  fiziksel montaj hatası (~90°) araç tarafında düzeltilince bu artık
  YANLIŞ oldu ve "hedef aracın soluna gidiyor" şeklinde canlı bildirildi.

### 5.2.1 BNO055 Montaj Düzeltmesi (2026-08-30) — KRİTİK

BNO055 kartı aracın gerçek önüne değil, **~90° yanlış yöne** monte
edilmiş (kartın Y ekseni aracın önüne, X ekseni soluna bakıyor — REP-103
gövde çerçevesinde X=ön, Y=sol olmalıydı). Araç tarafında
(`konum_birlestirici.py` `_mount_fix()`) gerçek fiziksel PITCH testiyle
kanıtlandı: düzeltme kaldırılınca RViz'de kaldırma hareketi PITCH'e değil
ROLL'e yansıyordu (2 ayrı denemede tekrarlandı). **Düzeltme:** ham
`/imu/data` kuaterniyonuna, araç gövde çerçevesinde (ART-ÇARPIM/
post-multiply, ÖNCÜL-çarpım denendi ama roll hâlâ değişiyordu) **+90°
Z-ekseni dönüşü** uygulanır: `q_govde = q_imu ⊗ q_montaj`.

Bu düzeltme **hem araç tarafında (`konum_birlestirici.py`) hem arayüz
tarafında (`harita_sistemi.py` `imu_callback`)** ayrı ayrı ama **aynı
formülle** uygulanıyor — ikisi arasında tutarlılık canlı ölçümle
doğrulandı (bkz. yukarısı). Etkilenen HER ŞEY: costmap dönüşü, hedef
tıklama dönüşümü, araç ikonu dönüşü, uydu haritasındaki LiDAR/araç
dönüşü (`uydu_harita_widget.py`, aynı `taktik_radar.yaw`'ı kullanır).

**Kozmetik "SIFIRLA" offset'i BUNDAN AYRI tutulur:** Ayarlar/Navigasyon
sayfasındaki "Sensör Konumunu Sıfırla" ve arayüz açılışında otomatik
uygulanan sıfırlama (`HaritaYoneticisi.__init__`'te `sifirla_istendi =
True` ile varsayılan) SADECE Suni Ufuk ve Roll/Pitch/Yaw metin
etiketlerini etkiler (`net_roll/net_pitch/net_yaw`) — costmap/hedef/ikon
dönüşü HER ZAMAN mutlak (offset'siz) `yaw`'ı kullanır. Bunun nedeni:
Nav2'nin kendi `/odom` referansı kullanıcının arayüzden ne zaman
"SIFIRLA"ya bastığından habersizdir; offset'i navigasyon matematiğine
karıştırmak "SIFIRLA"ya basmayı hedef yerleşimini kaydıran bir işleme
çevirirdi.

### 5.2.2 Rota (plan) Çizgisi "Boşlukta Kalma" Düzeltmesi (2026-08-30) — KRİTİK

**Belirti:** Araç fiziksel olarak hareket ettikçe, arayüzdeki araç ikonu
(bilerek) ekran merkezinde sabit kalırken, çizilen rota (`/plan`,
`/local_plan`) gitgide ikonla bağlantısız, "boşlukta" görünmeye başlıyordu
— canlı bildirildi.

**Kök neden:** `global_plan_callback`/`local_plan_callback`, Nav2'den gelen
`/plan`'ı **hiçbir dönüşüm yapmadan**, ham/mutlak `/odom` koordinatlarıyla
`TaktikRadarEkrani`'ye iletiyordu. Costmap noktaları (`costmap_callback`)
zaten dünya→araç-gövdesi dönüşümü yapıyordu (araç grid merkezinde kabul
edilerek), ama rota çizgileri için bu dönüşüm **hiç yoktu**. Araç odom
sıfırındayken tesadüfen doğru görünüyordu; araç uzaklaştıkça mutlak
koordinatlar büyürken, ekranda HER ZAMAN merkezde sabit duran araç
ikonuna göre rota git gide "kayıyordu".

**Düzeltme:** `Ros2GcsMotoru` artık `/odom`'a doğrudan abone (`odom_callback`,
TF **DEĞİL** — tf_buffer çökme riski yüzünden kalıcı devre dışı, bkz. §5.2).
Yeni `_dunya_to_arac(world_x, world_y)` yardımcı fonksiyonu, costmap'teki
BİREBİR AYNI dünya→gövde rotasyon formülünü kullanarak her `/plan` noktasını
göndermeden ÖNCE araç-göreceli (ileri, sol) çiftine çevirir. `paintEvent`
artık rota çizgilerini de costmap/lidar ile AYNI döndürülmüş bağlamda,
aynı piksel formülüyle çizer (eskiden rota, döndürme bloğundan ÖNCE, ham
koordinatla çiziliyordu).

**Simülasyonla doğrulandı** (fiziksel araç gerekmeden, gerçek
`Ros2GcsMotoru` koduna sahte `/odom` + `/plan` mesajları vererek): araç
odom(0,0)'dan odom(20,5)'e "sürdürülürken", rota HER ADIMDA araç-göreceli
olarak aynı (0-3m ileri) aralıkta kaldı — eski koddaki gibi mutlak
koordinatla büyüyüp ekran dışına taşmadı.

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
| `/arac_komut` | `std_msgs/String` | arayüz ↔ araç | Ham klavye tuş kodu VE literal `"EMERGENCY_STOP_CMD"`/`"DEVAM_CMD"` kilit komutları (bkz. §7) — `tabela_etap_yoneticisi.py` de Stop tabelasında buraya yayınlıyor |
| `/yon_pid_aktif` | `std_msgs/Bool` | arayüz → araç | Yön düzeltme PID AÇ/KAPA anahtarı (bkz. §7) |
| `/mavros/gpsstatus/gps1/raw` | `mavros_msgs/GPSRAW` | araç → arayüz | GPS/GNSS ETKİN göstergesi (`fix_type`, bkz. §8) |
| `/tabela_tespit` | `std_msgs/String` (`"SinifAdi:güven"` / `"YOK"`) | araç içi (`tabela_node.py`→`tabela_etap_yoneticisi.py`) | Tabela algılama → etap/durdurma kararı (bkz. §7) |
| `/guncel_etap` | `std_msgs/Int32` | araç → arayüz | Algılanan son etap numarası (1-11) → dashboard'daki ETAP göstergesi (`label_etapNo`) — Designer'da sabit "8" yazıyordu, `etap_sinyali` TANIMLIYDI ama hiç emit edilmiyordu (`gps_durum_sinyali` ile AYNI durumdaydı); artık gerçek veriyle besleniyor, canlı doğrulandı |
| `/parkur_durumu`, `/otonom_dur_nedeni` | `std_msgs/String` | araç → arayüz (henüz arayüzde gösterilmiyor) | Parkur tamamlandı / otonom durdurma sebebi |

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

`terminal_widget.py`, araç Jetson'a (`VARSAYILAN_HOST`, `terminal_widget.py:29`
— ağ topolojisine göre değişebilir, örn. Ubiquiti üzerinden `192.168.1.22`
veya doğrudan WiFi üzerinden `10.40.64.43`) otomatik bağlanır, proje
klasörüne (`~/Desktop/tufan_v2_ws`) `cd` yapar — araç Jetson'a hiç monitör
bağlanmadığı için TÜM komut satırı erişimi buradan sağlanır.

### 6.5 mavros (Cube/ArduPilot) ve NTRIP/RTK köprüsü

Araç Jetson'a **USB ile bağlı bir Cube (ArduPilot çalıştıran uçuş
kontrolcüsü)** eklendi. Araç tarafında `mavros_node` çalışıyor
(`fcu_url:=/dev/ttyACM0:57600`) ve **`/mavros/global_position/global`
topic'i `/fix`'e yeniden adlandırılmış** durumda (launch parametresi:
`-r /mavros/global_position/global:=/fix`) — yani arayüzün zaten var olan
`/fix` tabanlı GNSS boru hattı (bkz. §6.1, §5.3, §5.5) hiçbir kod
değişikliği gerekmeden gerçek Cube GPS verisiyle çalışır.

**RTK düzeltmesi gönderimi** (`ntrip_rtk_sistemi.py`): arayüz Jetson,
internet üzerinden **Swift Navigation Skylark** NTRIP caster'ına
(`eu.l1l5.skylark.swiftnav.com:2101`, mountpoint `NXRTK-MSM5`) bağlanıp
RTCM3 düzeltme baytlarını indirir, `mavros_msgs/RTCM` mesajına sarıp
`/mavros/gps_rtk/send_rtcm` topic'ine yayınlar; mavros bunu otomatik
olarak MAVLink `GPS_RTCM_DATA` ile Cube'a, oradan DroneCAN üzerinden
Here4'e iletir. TUSAGA-Aktif'in ıslak imzalı kurumsal başvuru süreci
yerine tercih edildi (öğrenci/yarışma zaman kısıtı). **Canlı test edildi
(2026-08-28):** 18 saniyede
90 RTCM mesajı / 13.181 bayt gerçek Skylark akışından `/mavros/gps_rtk/
send_rtcm`'e başarıyla iletildi, GGA gerçek araç `/fix` konumunu (o an
38.7082, 35.5195 civarı) doğru kullandı.

- Kendi izole ROS2 node'unda çalışır (`tufan_ntrip_rtk_koprusu`) — diğer
  hiçbir abonelik/yayınla aynı node'u paylaşmaz (bkz. §2'deki DDS
  izolasyon dersi). Node ayrıca **doğrudan `/fix`'e abone olur**
  (`_gps_callback`, `ARAC_GPS_TOPIC`) — arayüz Jetson'da GPS olmadığı
  için GGA'daki konum, araç Jetson'un mavros'unun cross-machine ROS2
  discovery ile yayınladığı gerçek konumdan okunur; ilk bağlantı anında
  henüz veri gelmemişse geçici bir yer tutucu (Ankara civarı) kullanılır.
- **GGA gönderimi 1Hz'de** (`GGA_GONDERIM_ARALIGI_SN = 1.0`) — bu
  mountpoint konum tabanlı (MSM5) çalıştığından 10sn'de bir yeterli
  değil, akışı zayıflatıyor/kesiyor (canlı test edilerek doğrulandı).
- **Mesaj başına en fazla 700 bayt** (`MAKS_PARCA_BOYUTU`) — mavros'un
  `gps_rtk` eklentisi `mavros_msgs/RTCM.data`'da 720 baytın (4×180,
  MAVLink `GPS_RTCM_DATA` parçalama sınırı) üzerini **sessizce** atıyor
  (hata vermiyor, sadece kayboluyor); Skylark bazen tek `recv()`'de
  700+ baytlık parçalar gönderdiği için bu sınır gerçekten devreye
  giriyor (canlı testte `max_mesaj_boyutu=700` gözlendi).
- **İstek formatı PointOneNav referans istemcisinden doğrulanmış,
  Skylark'a özel:** `GET /<mountpoint> HTTP/1.0` + `User-Agent` + `Accept`
  + `Authorization: Basic` başlıkları, **`Host`/`Ntrip-Version`/
  `Connection` başlıkları YOK** — farklı bir sıra/başlık seti
  denenmemeli.
- **Yanıt iki farklı biçimde gelebilir** ve ikisi de ele alınıyor:
  NTRIP v1/ICY tarzı `ICY 200 OK\r\n` (TEK CRLF, hemen ardından binary
  RTCM — ikinci bir `\r\n\r\n` beklemek sonsuza kadar takılmaya sebep
  olurdu, bu yüzden `_ntrip_baglan` özel olarak bu tek-satır durumunu
  ayrıca kontrol ediyor) **veya** standart `HTTP/1.x 200 OK` + başlık
  bloğu (`\r\n\r\n` ile biter). HTTP durumunda `Transfer-Encoding:
  chunked` olabiliyor — `_ParcaliCozucu` sınıfı bunu soket akışından
  çözüyor (chunk uzunluk satırları + `\r\n` ayraçları RTCM verisine
  karışmasın diye); bu sınıfın parça-sonu `\r\n`'i besle() çağrıları
  arası bölünürse kilitlenen bir hatası bulunup düzeltildi (durum
  bayrağıyla, `_kuyrukta_crlf_bekleniyor`).
- **Yeniden bağlanma sadece gerçekten uzun süre (75sn, `STALE_ESIK_SN`)
  veri gelmezse tetiklenir** — kısa `recv()` zaman aşımları normal
  kabul edilir; çok agresif "stale" tespiti alıcının RTK Fixed'e
  kilitlenme sürecini sürekli sıfırlıyor (bu yüzden bilinçli olarak
  toleranslı tutuldu).
- **Tek bağlantı kuralı:** Skylark hesabıyla aynı anda sadece BİR
  bağlantı açılabilir — ikinci bir bağlantı (test scripti, ikinci
  `main.py` instance'ı vb.) sunucu tarafından "DUPLICATE" olarak
  kapatılabilir. `main.py`'yi yeniden başlatmadan/test etmeden önce
  önceki instance'ın gerçekten kapandığından emin olunmalı
  (`ps aux | grep main.py`).
- `mavros_msgs` paketi bu makinede **apt ile `/opt/ros/humble`'a**
  kurulu (kaynak-derlenmiş `~/ros2_humble` içinde değil) — bu yüzden bu
  bileşeni çalıştırırken/geliştirirken **her iki ROS2 kurulumu da art
  arda source edilmeli**: `source ~/ros2_humble/install/setup.bash &&
  source /opt/ros/humble/setup.bash` (bu sırayla `rclpy` yine
  kaynak-derlenmiş sürümden gelir, sadece eksik mesaj paketleri
  `/opt/ros/humble`'dan tamamlanır).

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
  sabit 512 varsayımı yanlış komut üretebilirdi). Sürüş sırasında da
  sürekli oto-merkezleme çalışır (`surus_joystick_sistemi.py`), X ve Y
  eksenleri **birbirinden bağımsız** kontrol edilir — eskiden ikisi birden
  aynı anda sabit olmadıkça hiçbiri düzelmiyordu; sürüş titreşimi bir
  eksende sürekli küçük gürültü yaratırsa diğeri gerçekten kaymış olsa
  bile hiçbir zaman düzelemiyordu (canlı bildirilen "joystick hareketsizken
  aracın durmaması" hatasının kök nedeni — watchdog bunu yakalayamaz,
  çünkü joystick veri göndermeye devam eder, sadece merkezi yanlıştır).
  Ayrıca `MAKS_MERKEZ_KAYMASI` (düzeltme kabul sınırı) `ADC_OLU_BOLGE`
  (ölü bölge) ile AYNI değerde (60) olduğu için düzeltilmesi gereken her
  kayma (>60) otomatik olarak düzeltme sınırının da dışında kalıyordu —
  hiçbir zaman düzeltilemiyordu; artık 150'ye çıkarıldı (gerçekçi kaymayı
  kapsar, tam itilmeden — ~400+ birim — hâlâ açıkça ayrışır).
- **SON GÜVENLİK AĞI (2026-08-31) — KRİTİK**: Kaptan tarafından canlı
  bildirildi: joystick fiziksel olarak hareketsiz kalmasına rağmen araç
  sürekli hareket etmeye devam ediyor, sadece acil durdurma + joystick'i
  biraz oynatmak düzeltiyordu (300kg araç için ciddi risk). Kök neden tam
  teşhis edilemese de (donanımsal gevşek bağlantı/kalibrasyon sırasında
  kazara dokunma gibi ihtimaller var), canlı kalibrasyon mantığından
  **tamamen bağımsız** ikinci bir güvenlik katmanı eklendi
  (`surus_joystick_sistemi.py`): ham ADC okuması ~1 saniye (neredeyse) hiç
  kıpırdamadıysa **VE** bu sabit değer **ilk (hiç değişmeyen) kalibrasyon
  merkezine** yakınsa (`GUVENLIK_SABIT_MAKS_UZAKLIK=220`), o eksenin çıkışı
  hesaplanan değer ne olursa olsun sıfıra zorlanır. "İlk merkeze yakınlık"
  şartı kasıtlı — sadece ham durgunluğa bakılsaydı, joystick'i tam ileride
  uzun süre BASILI TUTMAK (düz sürüş için normal) da yanlışlıkla
  sıfırlanırdı. Üç senaryoyla izole test edildi (gerçek araca gerek
  kalmadan): (1) hatalı kalibrasyon + gerçekten sabit joystick → doğru
  şekilde sıfırlandı, (2) tam ileri + 5 saniye sabit tutma → hiç
  etkilenmedi, (3) normal serbest bırakma → sorunsuz. Tetiklendiğinde
  `guvenlik_sinyali` üzerinden arayüz logına (`log_yaz`) yazılır — bir
  daha olursa adli/teşhis kaydı için.
- **Klavye/joystick geçişinde odak (focus) kaybı**: `ayar_klavye_tetikle()`
  ve `ayar_joystick_tetikle()` butonlarına tıklamak Qt odağını o butonda
  bırakıyordu, `keyPressEvent` de odak ana pencerede değilse tuşları hiç
  almıyordu — "joystick kapatıp klavye açtığımda süremiyorum" olarak canlı
  bildirildi. İkisi de artık sonunda `self.setFocus()` çağırıyor; ayrıca
  joystick kapatıldığında `surus_kaynagi` KLAVYE'ye dönerken klavye ayrıca
  kapalıysa otomatik açılıyor.
- **Mod bazlı erişim**: klavye/joystick komutları sadece `arac_modu ==
  "MANUEL"` iken etkilidir; OTONOM modda `surus_koprusu.py` (araç tarafı)
  Nav2 çıktısını `/palet_hizlari`'a çevirir, arayüz sessiz kalır.
- **ACİL DURDUR KİLİDİ ve DEVAM ET (2026-08-31) — KRİTİK**: Bu, aracın
  ARKA panelindeki fiziksel rotary switch acil stopundan (BLDC/silah
  motorlarını doğrudan keser, Jetson'a hiç bağlı değil) TAMAMEN AYRI bir
  yazılım katmanı — arayüzdeki ACİL DURDUR butonu artık araç tarafındaki
  motor komutlarını gerçekten ve kalıcı olarak keser. Önceden ciddi bir
  eksiklik vardı: `arduino_motor_kontrol.py` (araç) `/arac_komut` topic'ini
  HİÇ DİNLEMİYORDU, bu yüzden ACİL DURDUR sadece o anki PWM'i BİR KEZ
  sıfırlıyordu — hemen ardından gelen bir sonraki joystick/otonom
  `/palet_hizlari` mesajı bunu anında eziyordu (gerçek bir kilit yoktu).
  Ayrıca `telemetri_sistemi.py` içinde `hareket_emri_gonder("EMERGENCY_STOP_CMD")`
  çağrısı, genel "DUR" dalına düşüp `/arac_komut`'a **literal "EMERGENCY_STOP_CMD"
  yerine "X\n" gönderiyordu** — yani araç tarafının beklediği metinle hiç
  eşleşmiyordu, kilit sistemi kodda var olsa bile asla tetiklenemezdi. İki
  parça birden düzeltildi:
  - `arduino_motor_kontrol.py` (araç): yeni `_kilitli` durumu, `/arac_komut`
    (String) aboneliği eklendi. `"EMERGENCY_STOP_CMD"` gelince `_kilitli=True`
    olur, Arduino'ya anında nötr sinyal (1500,1500) yazılır, ve `_kilitli`
    açık kaldığı sürece `palet_callback` gelen HİÇBİR `/palet_hizlari`
    mesajını işlemez (joystick/klavye/otonom fark etmez) — sadece
    `"DEVAM_CMD"` mesajı kilidi açar. İzole testle doğrulandı (5/5 senaryo
    PASS) hem sahte donanımla hem de gerçek araç Jetson'ında.
  - `telemetri_sistemi.py` (arayüz): `hareket_emri_gonder()` artık
    `"EMERGENCY_STOP_CMD"`/`"DEVAM_CMD"` için ayrı, en baştaki bir dal —
    genel yön/harf komutlarından önce bu iki değeri **literal metin olarak**
    (harf kodlamasına uğratmadan) `/arac_komut`'a yayınlar; mod/kaynak
    kısıtlamalarını (MANUEL/KLAVYE şartı) her ikisi de atlar, tıpkı eski
    "DUR (SPACE)" gibi — acil durdurma ve devam her zaman çalışmalı.
  - `main.py`: `acil_durdurma()` artık kilitli durumu (`_arac_kilitli_mi`)
    işaretliyor ve sol menüde yeni yeşil "DEVAM ET" butonunu (`pushButton_devamEt`)
    görünür yapıyor (normalde gizli — yanlışlıkla basılmasın, kilidin o an
    aktif olduğu görsel olarak da net olsun). Butona basınca onay kutusu
    (`QMessageBox.question`, varsayılan "Hayır") çıkar, onaylanırsa
    `"DEVAM_CMD"` gönderilip buton tekrar gizlenir.
  - Kasıtlı olarak KAPSAM DIŞI: aracın donanım seviyesindeki rotary switch
    acil stopu (BLDC + silah motorlarını fiziksel olarak keser) — bu
    yazılıma hiç bağlı değil, kullanıcı tarafından net şekilde ayrı tutuldu.

- **DÜZ GİDİŞTE YÖN DÜZELTME PID (2026-08-31)** — manuel sürüşte (klavye VEYA
  joystick, ikisi de `/palet_hizlari`'a aynı şekilde yazıyor) düz gidişte
  IMU'nun gyro'suna bakıp sapmayı otomatik düzeltiyor. Önce CANLI VERİ
  DOĞRULAMASI yapıldı: `/imu/data` 99Hz'de gerçek/canlı, durgunken yaw
  0.0000° std sapmayla kararlı — AMA aynı testte kalibrasyon durumu
  `{sys:0, gyro:3, accel:0, mag:0}` bulundu, yani ivmeölçer/manyetometre
  kalibre değil. `konum_birlestirici.py`'nin kendi dosya-başı notundaki
  bilinen bulguyla BİREBİR aynı durum: motor akımı değişince BNO055'in
  füze edilmiş (mutlak) yaw'i 2.14° sıçramıştı — yani PID'nin mutlak/füze
  yaw'a güvenmesi TEHLİKELİ olurdu (kendi kendini besleyebilir). Bu yüzden
  SADECE GYRO (`angular_velocity.z`, tam kalibre VE manyetik girişimden
  etkilenmiyor) kullanılıyor; referans her "düz gidiş" segmentinin
  BAŞLANGICINDA sıfırlanan bir gyro entegrali (ömür boyu değil, sadece o
  segment).
  - **Aktivasyon**: ham sol/sağ PWM AYNI İŞARETTE (ikisi ileri veya ikisi
    geri) ve sıfırdan farklıysa devrede; farklı magnitude (elle Q/Z/E/C
    trimi) sorun değil, sadece işaretler aynı olmalı. Tank dönüşü/durma
    anında devre dışı kalır ve entegral sıfırlanır.
  - **Yön**: standart diferansiyel-sürüş kinematiği `ω=k*(v_sağ-v_sol)`
    ile türetildi, SOLA/SAĞA DÖN'ün zaten doğrulanmış tanımlarıyla tutarlı;
    doğrusal olduğu için İLERİ'de de GERİ'de de AYNI düzeltme (sol'a
    +trim, sağ'a -trim) doğru fiziksel etkiyi veriyor — ayrı bir yön
    çarpanı gerekmiyor. `arduino_motor_kontrol.py`'de `_pid_yon_carpani`
    tek satırlık bir anahtar — canlı testte ters çıkarsa `-1.0` yapılır.
  - **Konum**: araç tarafında, `arduino_motor_kontrol.py`'de (network
    gecikmesi olmasın diye) — `MAX_TRIM=45` PWM ile üst sınırlı (~%18).
  - **Arayüz**: sol menüde pusula ikonlu, tıklanabilir AÇ/KAPA butonu
    (`main.py` `_yon_pid_butonu_ekle`), varsayılan AÇIK, `/yon_pid_aktif`
    (Bool) yayınlıyor.
  - **Test**: 7 izole senaryo (durgun/tank dönüşü/ileri sola-sapma
    düzeltmesi/geri sola-sapma düzeltmesi/farklı-magnitude-aynı-işaret/
    kapatma/MAX_TRIM sınırı) — hepsi PASS, hem sahte hem GERÇEK araç
    Jetson'ında (gerçek Arduino'ya bağlı, gerçek koddan) çalıştırıldı.
  - **UYARI**: yön matematiği izole testle doğrulandı ama HENÜZ gerçek
    hareket halindeki araçta test edilmedi — ilk canlı test düşük hızda,
    acil durdurma elde hazır şekilde yapılmalı.

- **TABELA → ETAP/DURDURMA KARAR DÜĞÜMÜ (2026-08-31)** — yeni dosya:
  `tabela_etap_yoneticisi.py`. `tabela_node.py` YOLO ile tabela tespit edip
  `/tabela_tespit`'e (`"SinifAdi:güven"` veya `"YOK"`) yayınlıyordu ama
  KENDİ DOCSTRING'İNDE belirtildiği gibi hiçbir sürüş mantığına bağlı
  değildi (canlı kontrolde `Subscription count: 0` bulundu). Model sınıfları
  (`Tabela_ana.pt`, canlı sorgulandı): `One`..`Eleven` (etap numarası),
  `EndOfEleven` (parkur bitişi), `Stop`.
  - **Etap takibi** (risksiz, bilgi amaçlı): `One`..`Eleven` görülünce
    `/guncel_etap` (Int32) yayınlanır (sadece değişince, güven eşiği 0.6).
    `EndOfEleven` → `/parkur_durumu` (String) `"TAMAMLANDI"`.
  - **Stop tabelası → gerçek durdurma** (güvenlik-kritik): SADECE OTONOM
    modda (`/surus_modu`) ve ARDIŞIK 3 karede yeterli güvenle tespit
    edilirse (tek bir gürültülü kare yanlışlıkla durdurmasın diye), MEVCUT
    acil durdurma kilidini (`arduino_motor_kontrol.py` `_kilitli`) `/arac_komut`'a
    `"EMERGENCY_STOP_CMD"` yayınlayarak tetikler — ayrı bir mekanizma İCAT
    ETMEK yerine zaten test edilmiş kilit yeniden kullanılıyor. Kilit
    KALICI kalır (varsayılan: `STOP_OTOMATIK_DEVAM_SN=None`), operatör
    arayüzden "DEVAM ET" basmalı — yarış kuralı "N saniye dur, sonra devam
    et" gerektiriyorsa bu tek satırlık ayar kolayca değiştirilebilir.
    Operatörün NEDEN durduğunu anlaması için `/otonom_dur_nedeni` (String,
    `"STOP TABELASI ALGILANDI"`) ayrıca yayınlanır.
  - MANUEL modda Stop tabelası HİÇBİR ŞEY tetiklemez (operatörün kendi
    kontrolüne müdahale edilmiyor).
  - **ÇOKLU STOP TABELASI (kullanıcı tarafından doğrulanan gerçek senaryo,
    2026-08-31)**: parkurda BİRDEN FAZLA Stop tabelası var — dik rampa
    ÇIKIŞINDA bir tane (araç durur, düzlükte silah ateşlenir), rampa
    İNİŞİNDE bir tane daha. İlk tasarımda `_stop_tetiklendi` mandalı
    SADECE OTONOM'dan çıkılınca sıfırlanıyordu — yani OTONOM modda
    KALINARAK "DEVAM ET" ile devam edilirse (bu senaryoda tam olarak
    böyle) İKİNCİ Stop tabelası hiçbir zaman tekrar tetiklenemezdi. Bu
    bulunup düzeltildi: node artık `/arac_komut`'u da dinliyor,
    `"DEVAM_CMD"` gelince (operatör kilidi her açtığında) mandal/sayaç
    sıfırlanıyor — aynı OTONOM koşusunda kaç tane Stop tabelası olursa
    olsun her biri ayrı ayrı doğru tetikleniyor.
  - `tufan_mppi.launch.py`'ye kalıcı olarak eklendi (bir sonraki tam
    başlatmada otomatik gelir); bu oturumda ayrıca canlı olarak tek başına
    çalıştırılıp `/tabela_tespit`'e abone olduğu doğrulandı.
  - **Test**: 11 izole senaryo (etap değişimi/tekrar-yayınlamama/parkur
    tamamlandı/MANUEL modda Stop güvenlik engeli/3-kare doğrulama eşiği/
    tekrar-tetiklenmeme/ara tespitle sayaç sıfırlanması/**AYNI OTONOM
    koşusunda DEVAM_CMD sonrası ikinci Stop tabelasının da tetiklenmesi**)
    — hepsi PASS, gerçek araç Jetson'ında.
  - **HENÜZ YAPILMADI**: gerçek kamera + gerçek Stop tabelası ile canlı
    uçtan uca test (şu ana kadar hep sahte/simüle `/tabela_tespit` mesajı
    ile test edildi — YOLO modelinin gerçek tabelayı doğru sınıflandırdığı
    ayrıca doğrulanmalı).

## 8. Dashboard Gösterge Referansı

Ana ekrandaki her göstergenin **hangi veriden**, **nasıl bir mantıkla**
tetiklendiği ve **kodun neresinde** olduğu. Her satır: *sinyal kaynağı
(veri nereden geliyor) → arayüz fonksiyonu (nasıl çiziliyor)*.

> **KURAL**: Bu bölüm, gösterge/veri akışı mantığını etkileyen her
> değişiklikte (yeni gösterge, eşik değeri değişimi, kaynak değişimi vb.)
> güncellenmelidir — hem bu dosyada hem GitHub'a giden repo kopyasında.

### İMU

| | |
|---|---|
| **Ne gösterir** | `/imu/data` (bno055) heartbeat'i — IMU verisi gerçekten geliyor mu |
| **Eşik** | 1.0 saniye içinde veri gelmezse PASİF |
| **Veri kaynağı** | `telemetri_sistemi.py:205` `imu_heartbeat_cb()` → zaman damgası günceller; asıl karar `telemetri_sistemi.py:301-304` `surekli_yayin_dongusu()` içinde (`imu_canli_mi = (time.time() - son_imu_zamani) < 1.0`) → `imu_durum_sinyali` sinyali |
| **Arayüz çizimi** | `main.py:326` `imu_arayuz_guncelle(aktif_mi)` — yeşil "IMU : ACTIVE/AKTİF" veya kırmızı "PASİF" |
| **Bağlantı** | `main.py:191` `telemetri_motoru.imu_durum_sinyali.connect(self.imu_arayuz_guncelle)` |

### SİSTEM (OTONOM HAZIR / MANUEL HAZIR)

| | |
|---|---|
| **Ne gösterir** | Araç Jetson'da otonom yığın (Nav2/goal_manager) ve/veya manuel sürüş köprüsü (Arduino) çalışıyor mu |
| **Eşik** | Otonom: `/goal_manager_heartbeat` 3.0 saniye içinde gelmişse hazır. Manuel: `/arduino_baglanti_durumu` son gelen değer (seri port açık mı) |
| **Veri kaynağı** | `telemetri_sistemi.py:210` `otonom_heartbeat_cb()`, `telemetri_sistemi.py:191` `motor_durum_cb()` → `telemetri_sistemi.py:286-290` `surekli_yayin_dongusu()` içinde `mod_durum_sinyali(otonom_hazir, manuel_hazir)` |
| **Arayüz çizimi** | `main.py:381` `mod_durum_guncelle()` + `main.py:406` `_mod_metni_ciz()` — ikisi de hazırsa 2 saniyede bir "OTONOM HAZIR" (mavi `#00AAFF`) / "MANUEL HAZIR" (yeşil `#00ff00`) arasında dönüşümlü; hiçbiri değilse kırmızı "PASİF" |
| **Font** | `main.py:406-424` `_sistem_yazisini_sigdir()` — metin uzunluğuna göre 32px'ten 14px'e kadar otomatik küçülen font (kutuya taşmasın diye) |

### LİDAR

| | |
|---|---|
| **Ne gösterir** | `/scan` heartbeat'i — LiDAR verisi gerçekten geliyor mu |
| **Eşik** | 1.0 saniye |
| **Veri kaynağı** | `telemetri_sistemi.py:199` `scan_heartbeat_cb()` → `telemetri_sistemi.py:293-297` `surekli_yayin_dongusu()` → `lidar_durum_sinyali` |
| **Arayüz çizimi** | `main.py:365` `lidar_arayuz_guncelle()` — aktifken mavi "TARANIYOR...", pasifken kırmızı "PASİF" |
| **Not** | Ana menüdeki dekoratif dönen radar çemberi (`main.py` `radar_cizimi_Guncelle`) BİLEREK gerçek LiDAR verisi çizmez — gerçek nokta bulutu sadece Navigasyon → TAKTİK LİDAR sekmesinde |

### UBIQUITI / WIFI (otomatik yedekleme, 2026-08-31)

| | |
|---|---|
| **Ne gösterir** | Araçla arayüz arasındaki kablosuz linkin bağlantı kalitesi — önce Ubiquiti nokta-nokta köprü denenir, TAMAMEN kopuksa (%0) otomatik olarak WiFi'ye (araç Jetson'ın `terminal_widget.py`'nin SSH için kullandığı AYNI IP'si, `10.40.64.43`) düşülür |
| **Ölçüm yöntemi** | Her 3 saniyede bir önce `UBIQUITI_LINK_HEDEF_IP`'ye (`telemetri_sistemi.py`, 192.168.1.22) 5 hızlı ping; %0 dönerse HEMEN AYNI ŞEKİLDE WiFi IP'sine 5 ping daha denenir (radyonun kendi yönetim IP'si ICMP'yi engellediği için doğrudan araç hedef alınıyor) |
| **Veri kaynağı** | `telemetri_sistemi.py` `_wifi_kontrol_dongusu()` (ROS2'den bağımsız ayrı thread) → `_wifi_kontrol()` → `wifi_durum_sinyali(yuzde, kaynak)` — `kaynak` "UBIQUITI" veya "WIFI" |
| **Arayüz çizimi** | `main.py` `wifi_arayuz_guncelle(yuzde, kaynak)` — "{UBIQUITI\|WIFI} : %XX", >%60 yeşil, %30-60 turuncu, <%30 kırmızı |
| **Öncelik** | Ubiquiti çalışıyorsa WiFi'ye HİÇ ping atılmaz (gereksiz trafik/gecikme olmasın diye) — sadece Ubiquiti %0 dönünce WiFi denenir; ikisi de kopuksa gösterge "UBIQUITI : %0" olarak kalır (WiFi'yi asıl bağlantı gibi göstermemek için) |
| **Neden ayrı thread** | Ping bloklayıcı olabilir; ROS2 executor'ı içinde çalıştırılırsa (eskiden `nmcli` için olduğu gibi) `/palet_hizlari` dahil tüm yayın periyodik olarak birkaç saniyeliğine durur — canlı ölçümle doğrulanmış bir sorun |
| **İzole test** | 3 senaryo (ikisi de kopuk / Ubiquiti kopuk+WiFi çalışıyor / Ubiquiti çalışıyor→WiFi'ye hiç ping atılmıyor) — hepsi PASS |

### KAMERA

| | |
|---|---|
| **Ne gösterir** | Silah/turret kamerasından (UDP :5000) gerçekten kare gelip gelmediği |
| **Eşik** | 1.5 saniye |
| **Veri kaynağı** | `main.py:725-732` `video_ekrana_bas()` her karede `son_kamera_frame_zamani` günceller → `main.py:313-315` `_kamera_heartbeat_kontrol()` (500ms zamanlayıcı) |
| **Arayüz çizimi** | `main.py:358` `kamera_arayuz_guncelle()` — hazırsa yeşil "KAMERA : READY/HAZIR", değilse kırmızı "BAĞLANTI KOPTU" |

### SİLAH kamerası + hedef mesafesi

| | |
|---|---|
| **Ne gösterir** | Silah/turret kamerasının küçük önizleme kutusu, üzerinde TF02-Pro lidar'dan gelen hedef mesafesi |
| **Kanal eşlemesi** | `kamera_sistemi.py`'nin UDP :5000'den aldığı kare her zaman anahtar `1`'e denk gelir → `main.py:741` `pix1 = resmi_yuvarla(goruntu_sozlugu.get(1), ...)` |
| **Mesafe verisi** | `/turret_hedef_mesafe` (Float32) → `telemetri_sistemi.py:216` `hedef_mesafe_cb()` → `main.py:377` `hedef_mesafe_guncelle()`; 1.0 saniyeden eskiyse (`main.py:736`) sadece "SİLAH" yazısına döner, mesafe göstermez |
| **Sayfa bazlı optimizasyon** | `main.py:725-733` — Kamera sayfası (indeks 1) aktif değilken bu çizim tamamen atlanır, sadece kalp atışı zaman damgası güncellenir (performans, bkz. §Bilinen Sınırlamalar altındaki mimari notu) |

### GPS/GNSS (artık gerçek veri, 2026-08-31)

| | |
|---|---|
| **Ne gösterir** | GNSS gerçekten aktif mi — önceden `gps_durum_sinyali` tanımlıydı ama HİÇBİR YERDE `.emit()` edilmiyordu (tasarım zamanındaki sabit metinde donuk kalıyordu); artık `konum_birlestirici.py`'nin de kullandığı AYNI kaynağa (`/mavros/gpsstatus/gps1/raw`, GPSRAW) bağlı |
| **Eşik** | Heartbeat 3.0 saniye İÇİNDE veri gelmiş OLMALI **VE** `fix_type >= 3` (3D fix) olmalı — RTK şart değil (RTK'ya özel durum ayrı, NTRIP/RTK panelinde); sadece GNSS'in temelde çalıştığını göstermek için `konum_birlestirici.py`'nin kendi `gps_heading_min_fix_type` varsayılanıyla (3) TUTARLI bir eşik seçildi |
| **Veri kaynağı** | `telemetri_sistemi.py` `gps_raw_cb()` → `surekli_yayin_dongusu()` içindeki heartbeat+eşik kontrolü → `gps_durum_sinyali` |
| **Arayüz çizimi** | `main.py` `gps_arayuz_guncelle()` (değişmedi) — "GPS : ETKİN/ACTIVE" (yeşil) veya "GPS : PASİF/PASSIVE" (kırmızı) |
| **Canlı doğrulama** | Araç Jetson'ında `ros2 topic echo /mavros/gpsstatus/gps1/raw --once` ile gerçek veri kontrol edildi: `fix_type: 3`, `satellites_visible: 22`, `h_acc: 9127` (mm) — RTK aktif değilken bile GNSS'in temelde çalıştığı doğrulandı |
| **Not** | Gerçek GNSS konumu bu göstergeyle KARIŞTIRILMASIN — konum/harita verisi Navigasyon ekranındaki UYDU HARİTASI ve TAKTİK LİDAR sekmelerinde gösteriliyor; bu sadece "GNSS aktif mi" özet göstergesi (bkz. §Görsel Gösterim Mekanizmaları) |

### Hız / Batarya (araç, yer)

| | |
|---|---|
| **Hız** | `/odom` (`twist.linear.x`) → `telemetri_sistemi.py:178` `odom_cb()` → `hiz_sinyali` → `main.py:187` doğrudan `label_hizYazi.setText` |
| **Batarya** | `/arac_batarya` (gerçek karşılığı yok, bkz. §Veri Kanalları notu) → `batarya_sinyali` → `main.py:188` `arac_batarya_renklendir` |

### Canlı Sistem Logları (Ayarlar sayfası)

| | |
|---|---|
| **Ne gösterir** | Uygulamanın kendi iç olayları — mod değişimi, joystick kalibrasyonu, PWM sınırı onayı, bağlantı durumları vb. (ham ROS2 verisi DEĞİL) |
| **Eskiden** | Bu kutu (`textEdit_canliSistemLog`) tamamen kullanılmıyordu; `log_yaz()` ana ekrandaki terminale yazıyordu ama terminal artık gerçek bir SSH oturumu olduğu için o yazılar hiçbir yerde görünmüyordu (sadece konsola düşüyordu) |
| **Şimdi** | `main.py:115-127` `log_yaz()` — zaman damgalı satırı hem konsola hem `textEdit_canliSistemLog`'a yazar; kutu 500 satırı geçerse eskiler otomatik silinir (uzun oturumlarda sınırsız büyümesin diye) |

### Terminal (ana ekran, sol alt)

| | |
|---|---|
| **Ne gösterir** | Araç Jetson'a gerçek SSH oturumu; bağlanır bağlanmaz normal, bağlı bir terminal prompt'u gelir (eskiden otomatik PWM akışı başlıyordu, kullanıcı isteğiyle kaldırıldı — artık isteğe bağlı) |
| **Konum** | `terminal_widget.py` `_proje_dizinine_gec()` — sadece `cd` + `source` (ROS2) otomatik komutu |
| **Canlı PWM akışını göster** | Sol bardaki, ACİL DURDUR'un hemen üstündeki nabız/sinyal ikonlu buton (`main.py` `_pwm_izleme_butonu_ekle()`, `pushButton_pwmIzle`) — tıklanınca ana ekrana geçer ve `terminal_ekrani.pwm_akisina_don()`'u çağırır: önce Ctrl+C ile o an çalışan/yazılmakta olanı keser, sonra `ros2 topic echo /palet_hizlari`'ı başlatır. İkon, mevcut SVG ikonlarla aynı renkte (`#00E5FF`) `QPainter` ile çiziliyor (yeni bir kaynak/SVG dosyasına gerek kalmadan — `arayuz.py` otomatik üretim olduğu için elle düzenlenmiyor) |
| **Normal komut yazma** | Ctrl+C ile akışı durdurup normal komut satırına dönülebilir |
| **Komut geçmişi** | `PROMPT_COMMAND='history -a'` sayesinde ani bağlantı kopmasında bile `~/.bash_history`'e anında yazılır |

### PWM Hız Sınırı kutusu (Ayarlar sayfası, eski "Telefon" kutusu)

| | |
|---|---|
| **Ne yapar** | Kolay sürüş için üst PWM sınırını (85-255 arası, alt sınır sabit 85) canlı ayarlar; hem klavye hem joystick bu sınıra uyar |
| **Konum** | `main.py:277-283` (kurulum, her zaman kehribar renginde — diğer kutulardan bilerek farklı), `main.py:763-807` `pwm_sinirini_uygula()` (doğrulama + onay penceresi + `telemetri_motoru.pwm_ust_sinirini_ayarla()`) |
| **Mantık** | `telemetri_sistemi.py:262-280` `pwm_ust_sinirini_ayarla()` / `_pwm_olcekle()` — deadzone-telafili ölçekleme: `oran=0` → PWM 0, `abs(oran)>0` → en az 85, `abs(oran)=1` → güncel üst sınır |
| **Düzeltme** | `editingFinished` odak KAYBINDA da tetikleniyor; kutuda onaylanmış bir değer dururken (başarılı onaydan sonra kutu temizlenmiyor) başka bir yere (örn. kapatma butonuna) tıklamak odağı kaçırıp AYNI değeri tekrar tekrar onaya sokuyordu — `_pwm_son_uygulanan_metin` ile değer değişmediyse tekrar sorulmuyor artık |

### Joystick butonu (Ayarlar sayfası)

| | |
|---|---|
| **Ne yapar** | Sürüş kaynağını klavye ↔ fiziksel joystick arasında değiştirir (aynı anda sadece biri `/palet_hizlari`'a yazabilir) |
| **Konum** | `main.py:835-857` `ayar_joystick_tetikle()` — `SurusJoystickThread` başlatma/durdurma, `telemetri_motoru.surus_kaynagi` ayarı |
| **Kalibrasyon** | `surus_joystick_sistemi.py` — bağlantıda ilk 0.5sn ölçüm + sürekli oto-merkezleme (sadece mevcut merkeze yakın küçük kaymalar kabul edilir, tam itilmiş bir konum asla merkez sanılmaz — 300kg araç güvenliği için) |

## 8.1 Canlı Test Sonrası Düzeltmeler (2026-09-01)

Kullanıcının araç Jetson'u fiilen sürerken/test ederken bulduğu bir grup
sorun, tek oturumda araştırılıp düzeltildi:

- **Ubiquiti göstergesi YANLIŞ veri gösteriyordu** — eskiden sadece
  `UBIQUITI_LINK_HEDEF_IP`'ye ping atılıp başarılıysa "UBIQUITI" deniyordu.
  Canlı `ip route get` ile doğrulandı: bu IP'ye giden rota, Ubiquiti FİZİKSEL
  OLARAK BAĞLI DEĞİLKEN BİLE WiFi'nin kendi ağ geçidi üzerinden gidiyordu
  (arayüz Jetson'da Ubiquiti'ye özel ayrı bir arayüz/alt ağ yok) — yani
  Ubiquiti kablosu hiç takılı değilken bile gösterge "UBIQUITI: %40/%100"
  gösterebiliyordu (canlı bildirildi). **Düzeltme**: artık önce Ubiquiti
  radyosunun bağlı olduğu kablolu Ethernet portunun (`enP8p1s0`) FİZİKSEL
  link durumu (`/sys/class/net/enP8p1s0/carrier`) kontrol ediliyor - kablo/
  radyo yoksa ping'e hiç bakılmadan direkt WiFi'ye düşülüyor. İzole testle
  doğrulandı (fiziksel bağlantı yokken ping başarılı dönse bile artık
  "UBIQUITI" etiketlenmiyor, gereksiz ping de atılmıyor).
- **Klavye ile süremiyorum (Ayarlar sayfasından aktif edince)** —
  `pushButton_klavye`/`pushButton_joystick` Ayarlar sayfasında (index 4)
  duruyor, ama `keyPressEvent` SADECE [0,1,2] sayfalarında tuş kabul ediyor
  (kasıtlı - ayar kutularına yazarken yanlışlıkla araç sürülmesin diye).
  Kullanıcı Ayarlar sayfasındayken klavyeyi aktif edip hemen WASD'a basınca
  hiçbir şey olmuyordu. **Düzeltme**: klavye aktif olunca otomatik olarak
  ana ekrana (index 0) dönülüyor.
- **ESC ile rota çizimini iptal etme** — Taktik Radar'da fare ile hedef oku
  çizerken artık ESC'ye basınca (`TaktikRadarEkrani.hedef_cizimini_iptal_et`)
  hedef GÖNDERİLMEDEN önizleme oku temizlenip iptal ediliyor. `main.py`
  `keyPressEvent`'te en başta, klavye/mod şartlarından BAĞIMSIZ çalışır.
- **Rota tamamlanınca mavi çizgi kaybolmuyordu** — Nav2 hedefe ulaşınca
  `/plan`'a yeni mesaj yayınlamayı KESİYOR (boş Path de göndermiyor), yani
  eski rota çizgisi ekranda sonsuza kadar kalıyordu. **Düzeltme**:
  `goal_manager_node.py`'nin zaten yayınladığı `/ugv_goal_result`
  (SUCCESS/FAILURE) dinlenip, sonuç gelince (`goal_result_callback`) hem
  global hem local plan çizgisi temizleniyor (boş liste yayınlanıyor).
- **Terminal kapatılınca/arayüz kapanınca araçtaki süreçler ölüyordu** —
  eskiden çıplak `ssh -tt` oturumu açılıyordu; arayüz kapanınca yerel ssh
  istemcisine SIGTERM gidip bağlantı düşüyor, bu da UZAK shell'e SIGHUP
  göndererek o an ön planda çalışan HERHANGİ BİR ŞEYİ (ör. operatör elle
  `ros2 launch` çalıştırmışsa) öldürüyordu. **Düzeltme**: artık
  `tmux new-session -A -s <isim>` ile bağlanılıyor - oturum aracın kendi
  tmux SUNUCUSUNDA yaşıyor, SSH bağlantısı kesilse bile tmux sadece DETACH
  olur, içindeki hiçbir şey durmaz; bir dahaki bağlantıda (`-A`) aynı
  oturuma, aynı çalışan komutlarla geri dönülür. 3 ayrı SSH bağlantısı
  arasında ortam değişkeninin korunduğu canlı test edilerek doğrulandı.
- **Birden fazla terminal** — sol menüye "+" ikonlu yeni bir buton eklendi
  (`main.py` `_yeni_terminal_ac`) - her tıklamada, benzersiz isimli AYRI bir
  tmux oturumuna bağlanan yüzen bir pencere açılır; ana terminalden ve
  birbirlerinden bağımsız çalışırlar, hepsi yukarıdaki kalıcılık garantisine
  sahiptir.
- **Dil tutarsızlığı** — `imu_arayuz_guncelle`/`gps_arayuz_guncelle`/
  `kamera_arayuz_guncelle` (ve zararsız ama aynı desende `guc_arayuz_guncelle`)
  sadece PASİF/hata durumunda dile bakıyordu, AKTİF durumda her zaman
  İngilizce yazıyordu (ör. sistem Türkçeyken "IMU : PASİF" ama "IMU : ACTIVE"
  gibi) — hepsi düzeltildi, artık her iki durumda da dile bakıyor.
- **SİSTEM yazısı (OTONOM/MANUEL HAZIR)** — artık ikisi de yeşil (`#00ff00`,
  eskiden OTONOM mavi `#00AAFF` idi) ve daire sınırının biraz dışına
  taşabiliyordu (`_sistem_yazisini_sigdir`'daki kenar payı 12px'ten 50px'e
  çıkarıldı - dairenin o yükseklikteki gerçek/yuvarlak genişliği kutunun
  kendisinden bile az farkla dar olduğu hesaplanarak bulundu).
- **HENÜZ YAPILMADI / araştırma gerektiriyor**: "araç çizilen mavi rotanın
  ÜZERİNDEN tam takip etmiyor" şikayeti - bu bir Nav2 kontrolcü ayarı/canlı
  tünning konusu, kod incelemesiyle kesin bir hata bulunamadı; canlı testle
  birlikte araştırılmalı. "araca bağlantı gitti" (Wi-Fi yetersizliği
  şüphesi) de aynı şekilde canlı ağ testi gerektiriyor.

## 9. Bilinen Sınırlamalar / Gelecek İşler

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
