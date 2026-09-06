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

> **NOT (2026-09-04):** `surus_joystick_sistemi.py` / `SurusJoystickThread`
> ARTIK KULLANILMIYOR — yerini `kontrol_paneli_sistemi.py`
> `KontrolPaneliThread` aldı (yer istasyonu fiziksel kontrol paneli:
> 2 joystick + 2 buton + 2 toggle + pot, tek Arduino). Yukarıdaki kod
> hâlâ geçerli bir QThread + pyserial üslup örneğidir. Bkz. §7.5.

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

### 5.2.3 SABİT ÇERÇEVE Mimarisi (2026-09-01) — §5.2.2'yi DEĞİŞTİRİR, KRİTİK

**Kullanıcı isteği (birebir):** "arayüzden rota oluştururken çizilen mavi
renkli rota araç ilerledikçe araç prototipi o rota üzerinde ilerlesin ve
rota bittiğinde tamamen yok olsun... sabit olan aracın prototipi değil
aracın odometrisi olsun... ben rvizdeki araçta olan aynı görüntüyü çekmek
istiyorum taktik lidar ekranına."

**Eski mimari (§5.2.2, artık GEÇERSİZ) — EGO-MERKEZLİ:** araç ikonu HER
ZAMAN ekran merkezinde sabit çizilirdi, `paintEvent` TÜM SAHNEYİ
`rotate(-self.yaw)` ile döndürürdü - rota/costmap/lidar hepsi ÖNCEDEN
araç-göreceli (ileri,sol) çiftine çevrilmiş olarak geliyordu. Bu, "araç
hiç ilerlememiş gibi görünüyor" şikayetinin kök nedeniydi: ikon asla
hareket etmediği için, rota "onun etrafında dönüyormuş" gibi bir izlenim
veriyordu - RViz'deki (`nav2_view.rviz`, Orbit kamera, `Target Frame:
base_footprint`) görünümle KARŞILAŞTIRILDIĞINDA, RViz'in kamerası SADECE
aracın KONUMUNU takip ediyor, kendi Yaw/Pitch açısını döndürmüyor - yani
harita SABİT bir yönelimde durup araç o sabit harita üzerinde gerçekten
hareket ediyor/dönüyor.

**Yeni mimari — SABİT ÇERÇEVE (RViz'in canlı doğrulanmış davranışının
birebir aynısı):**
- **Kamera**: hâlâ aracın konumunu takip eder (öteleme/pan - `robot_x`,
  `robot_y`, `Ros2GcsMotoru.odom_callback` → `guncelle_konum`, ARTIK
  `self.update()` de tetikliyor - eskiden sadece hata ayıklama yazısı
  için kullanılıp repaint tetiklemiyordu, kamera IMU/LiDAR'dan bağımsız
  pürüzsüz akmıyordu).
- **Yönelim**: ARTIK SABİT - `paintEvent`'teki `rotate(-self.yaw)`
  SAHNEDEN kaldırıldı. "Yukarı" ekranda HER ZAMAN dünya (`/odom`) +X
  yönünü gösterir (aracın açılıştaki/ilk yönü - bkz. §araç başlangıç
  konumu notları).
- **Rota (`/plan`, `/local_plan`)**: artık `_dunya_to_arac` ile
  ÇEVRİLMİYOR, HAM `/odom` (x,y) koordinatı olarak taşınıyor. Geriden
  eksilme mantığı (§8.1) korundu, ama artık `_gecmis_mi()` yardımcı
  fonksiyonu SADECE FİLTRELEME için yön hesabı yapıyor, çizim için değil.
- **Costmap**: aynı şekilde artık HAM dünya koordinatı yayınlıyor
  (`costmap_callback`), yön (yaw) dönüşümüne hiç gerek kalmadı.
- **LiDAR** (`self.noktalar`, gövde-göreceli ileri/sol): SABİT yönelime
  uydurmak için `paintEvent` içinde HER ÇİZİMDE aracın güncel yaw'ı kadar
  döndürülüp (dünya eksenlerine hizalanıp) sonra aracın o anki konumundan
  (kamera merkezi) çiziliyor.
- **Araç ikonu**: artık SAHNE değil SADECE İKONUN KENDİSİ
  `rotate(-self.yaw)` ile dönüyor (ayrı bir `save()/restore()` bloğunda) -
  RViz'deki "araç modeli haritanın üzerinde gerçekten döner" davranışının
  birebir aynısı. Ekranda hâlâ merkezde (kamera onu takip ettiği için),
  ama artık GERÇEKTEN kendi ekseni etrafında dönüyor.
- **Hedef tıklama (`mouseReleaseEvent`)**: eskiden fare sürüklemesi önce
  "sabit çerçeve" ofsetine, SONRA aracın o anki yönüne göre TEKRAR
  gövdeye döndürülüp `frame_id='base_footprint'` ile yayınlanıyordu - bu
  çifte dönüşüm, "goal_pose istediğim noktaya gitmiyor" şikayetinin olası
  bir kaynağıydı. **Artık**: Nav2'nin TÜM stack'i zaten `global_frame:
  odom` kullandığı için (nav2_params.yaml'da doğrulandı), tıklama
  DOĞRUDAN mutlak `/odom` koordinatına çevrilip `hedefe_git()` içinde
  `frame_id='odom'` ile HİÇBİR ek dönüşüme uğramadan yayınlanıyor -
  tıklanan nokta = hedefin gideceği nokta, aracın anlık yönünden tamamen
  bağımsız, garantili.

**Doğrulama:** 10 senaryolu izole matematik testiyle (dünya↔ekran
dönüşümünün birbirinin tam tersi olduğu, kamera takibi, LiDAR
dönüşümünün ikon dönüşümüyle TUTARLI yönde olduğu) doğrulandı; canlı
ekran görüntüsüyle de doğrulandı - araç fiziksel olarak 113° dönmüşken
ikon da ekranda gerçekten 113° dönmüş halde görüldü (eskiden ikon HER
ZAMAN yukarı bakardı, yaw'dan bağımsız).

**v2 DÜZELTME (2026-09-01, aynı gün) — kamera takibi de kaldırıldı:**
v1'de kamera hâlâ aracın konumunu takip ediyordu (sadece sahneyi
döndürmüyordu) - canlı bildirildi: "hala rota üzerinde araç ilerlemiyor".
Kök neden: araç ekranda HER ZAMAN AYNI (merkez) noktada kaldığı için
"ilerleme" hissi hâlâ oluşmuyordu. **v2: kamera artık HİÇ ÖTELEMİYOR/
DÖNDÜRMÜYOR** - ekran merkezi HER ZAMAN dünya (`/odom`) orijinine (0,0)
denk gelir (RViz'de "Fixed Frame: odom" ile birebir aynı). Araç ikonu
artık `arac_ekran_x/y` (kendi gerçek dünya konumundan hesaplanan ekran
konumu) üzerine çizilir - GERÇEKTEN ekranda gezinir, görüş alanının
(artık 15m, eskiden 6m - RViz Orbit "Distance:15" ile paralel) dışına
bile çıkabilir (gerçek bir sabit çerçevenin doğal sonucu). Hedef tıklama
da artık aracın konumunu EKLEMİYOR - tıklanan ekran noktası DOĞRUDAN
mutlak `/odom` koordinatı. 8 senaryolu izole matematik testiyle
(özellikle: "araç ilerledikçe KENDİ ekran konumu değişir" ve "sabit bir
dünya noktası araç nerede olursa olsun AYNI ekran konumunda kalır")
yeniden doğrulandı.

**v3 DÜZELTME (2026-09-01, aynı gün) — LiDAR/ikon GÖRECELİ titremesi:**
canlı bildirildi: "araç sabit duruyor ama Taktik LiDAR ekranında sürekli
oynama oluyor, RViz'de yok". **Kök neden**: v2'de LiDAR noktaları
(gövde-göreceli) "sabit" dünya yönelimine uydurmak için AYRI bir manuel
sin/cos ile döndürülüp, araç ikonu ise AYRI bir `ressam.rotate(-self.yaw)`
transformuyla çiziliyordu - İKİSİ DE aynı `self.yaw` değerini kullanıyor
olsa da, IMU her güncellendiğinde (~60Hz throttle) bu değer küçük de olsa
gürültü içerir; İKİ FARKLI transform yolu (biri Python sin/cos, diğeri
Qt'nin kendi matrisi) arasındaki kayan-nokta/zamanlama farkı, LiDAR
bulutunun SABİT costmap ızgarasına göre DEĞİL ama ARAÇ İKONUNA göre
GÖRECELİ olarak titremesine yol açtı. **Düzeltme**: LiDAR noktaları
artık AYRI bir dünya-dönüşümü yapmıyor - doğrudan aracın KENDİ
`translate(arac_ekran_x,y); rotate(-self.yaw)` bloğunun İÇİNDE, Qt'nin
TEK BİR transform matrisiyle çiziliyor (orijinal/eski koddaki AYNI teknik,
sadece artık ikon merkezde değil kendi gerçek konumunda). Lidar ve ikon
artık HER ZAMAN birebir aynı transformu paylaştığı için aralarında
göreceli titreme matematiksel olarak imkansız.

**v4 DÜZELTME (2026-09-01, aynı gün) — LiDAR nokta boyutu:** v3'ten sonra
bile "hala Taktik LiDAR'da kaymalar oluyor" bildirildi. Araştırıldı:
`/odom` VE `/local_costmap/costmap` ikisi de araç dururken PİKSEL PİKSEL
SABİT çıktı (canlı ölçüldü, hiç sensör kaynaklı gerçek "kayma" YOK).
**Kök neden**: RViz'in kendi LaserScan ayarı (`nav2_view.rviz`) noktaları
SABİT 5cm (0.05m) dünya boyutunda çiziyor - burada ise SABİT 6 PİKSEL
kullanılıyordu; 15m görüş alanında bu ~0.4m'ye denk geliyordu (RViz'in
~8 KATI). Her LiDAR'ın normal ölçüm gürültüsü (birkaç cm, tüm sensörlerde
vardır) RViz'in küçük noktalarında görünmezken, bu kadar büyük noktalarda
orantısal olarak çok daha belirgin bir "kayma" gibi görünüyordu.
**Düzeltme**: nokta boyutu artık RViz ile AYNI (5cm dünya boyutu, zoom'a
göre ölçekleniyor, sabit piksel DEĞİL).

**HENÜZ CANLI TEST EDİLMEDİ**: v4'ün gerçekten yeterli olduğu + gerçek
bir hedef tıklanıp aracın rota üzerinde gerçekten ilerlediği/döndüğü,
rota bitince kaybolduğu uçtan uca canlı sürüşle doğrulanmalı (fare
tıklaması/sekme değişimi simülasyonu yapılamıyor, kullanıcının kendisinin
kontrol etmesi gerekiyor).

### 5.2.4 Fare ile Yakınlaştırma/Kaydırma (2026-09-01)

SABİT ÇERÇEVE mimarisi (§5.2.3) kamerayı artık araca kilitlemediği için,
araç uzak bir hedefe giderken/uzaklaşırken görünüm dışına çıkabilir -
RViz'deki gibi elle kamera kontrolü eklendi:

- **Fare tekerleği**: yakınlaştırma/uzaklaştırma (`wheelEvent`) -
  `maksimum_menzil`i `uydu_harita_widget.py`'deki AYNI his/oranla
  (`×0.9`/`×1.1`) değiştirir, 1-200m arası sınırlı.
- **SAĞ TIK sürükleme**: görünümü kaydırır (`pan_x`/`pan_y`, dünya/`/odom`
  metre cinsinden). SOL TIK zaten hedef oku çizmek için kullanıldığından
  (haritacılık yazılımlarındaki yaygın kural gereği) çakışma olmasın diye
  SAĞ TIK seçildi.
- Tüm dünya↔ekran dönüşümleri artık `_dunya_to_ekran`/`_ekran_to_dunya`
  adlı iki KARŞILIKLI TERS metotta toplandı (eskiden `paintEvent` içinde
  bir closure'du) - hem çizim hem hedef tıklama hem kaydırma AYNI
  formülü kullanıyor, `pan_x`/`pan_y` sıfır olduğunda §5.2.3'teki
  davranışla birebir aynı sonucu verir. 4 senaryolu izole matematik
  testiyle (kaydırma sonrası round-trip tutarlılığı, sürükleme yönü)
  doğrulandı.

### 5.2.5 Çoklu Nokta Rotası (2026-09-01) — Otonom Gitmesi İçin Sıralı Hedefler

Kullanıcı isteği: "otonom gitmesi için aracın bizim birkaç nokta vermemiz
gerekiyor... birkaç işaret atabilelim, sağ tarafta atılan işaretlerin
mesafesi olsun". Eskiden her sol-tık-sürükle-bırak DOĞRUDAN tek bir hedef
gönderiyordu (§5.2.3/5.2.4). Artık:

- **Nokta biriktirme**: her sol-tık-sürükle-bırak artık hemen göndermez,
  `TaktikRadarEkrani.hedef_listesi`'ne EKLER - haritada sarı, numaralı
  işaretler + aralarında ince noktalı bağlantı çizgisiyle gösterilir.
- **Sağ panel** (`_sag_panel_kur`, `QFrame`'in sağına `resizeEvent`'te
  hizalanan bir `QWidget`): her noktayı `#1  X:12.34  Y:15.67  (19.2m)`
  formatında listeler - mesafe ARAÇTAN (o anki `robot_x/y`'den) hesaplanır.
  Listede bir noktaya **çift tıklamak** onu siler (`_nokta_sil`).
- **"🚀 ROTAYI GÖNDER"** butonu: listeyi `rota_gonder_istendi` sinyaliyle
  `Ros2GcsMotoru.coklu_hedef_gonder`'e iletir. **"✕ LİSTEYİ TEMİZLE"**:
  hiç göndermeden tüm listeyi boşaltır.
- **Gönderim mekanizması (KRİTİK)**: `/goal_pose` (bt_navigator'ın
  DOĞRUDAN dinlediği, §5.2.3'teki tekli/anlık hedef girişi) DEĞİL,
  **`/ugv_goal`** kullanılıyor - `goal_manager_node.py`'nin receding-
  horizon (uzak/görünmeyen hedefler İÇİN TASARLANMIŞ, kendi içinde ara
  bacaklara bölen) girişi. Noktalar SIRAYLA gönderilir: bir sonraki nokta,
  bir öncekinin `/ugv_goal_result` (SUCCESS/FAILURE/CANCELLED) sonucu
  gelmeden **ASLA** gönderilmez - FAILURE/CANCELLED'da rota güvenli
  tarafta DURDURULUR (kalan noktalar körlemesine gönderilmez), panelde
  "🛑 Rota DURDURULDU: nokta X/Y ... - kalan Z nokta GÖNDERİLMEDİ" yazar.
  İzole testle (5 senaryo: hepsi başarılı, ortada başarısız, boş liste,
  tek nokta, ardışık iki rota + "durmuş rotaya gecikmiş sahte SUCCESS
  gelirse yeniden başlamamalı" kenar durumu) doğrulandı - ilk yazımda SON
  noktanın sonucu geldiğinde kuyruk zaten boşaldığı için "TAMAMLANDI"
  hiç tetiklenmiyordu (bulundu/düzeltildi, bkz. `_rota_aktif` bayrağı).
- **⚠️ ÖNEMLİ - PAYLAŞILAN GİRİŞ**: `/ugv_goal` bu arayüzün TEK yazıcısı
  DEĞİL - araç tarafında `tabela_etap_yoneticisi.py` (otonom görev sırası,
  ör. rampa hedefi) ve `serbest_yon_takipcisi.py` (boşluk takibi/keşif) de
  AYNI topic'e yazıyor (canlı doğrulandı: `ros2 topic info --verbose` ile
  3 yayıncı görüldü). `goal_manager_node.py` yeni gelen HANGİSİ olursa
  olsun eskisini iptal edip kabul ediyor (`_on_new_goal`) - yani araç o
  anda otonom bir görev/keşif sırasındaysa VE kullanıcı manuel bir rota
  gönderirse, ikisi ÇAKIŞIR (son gönderen kazanır). Bu özellik, aracın
  otonom mantığı aktif olarak hedef GÖNDERMEDİĞİ manuel navigasyon
  oturumları için tasarlandı.
- **Grid iyileştirmesi** (kullanıcı isteği: "gridin daha ayırt edici
  olması gerekiyor hassas noktalar atabilmek için"): mesafe halkaları
  artık HER metrede etiketli (eskiden sadece 5'te bir, görüş alanı
  >20m'yse yine 5'te bire dönüyor - kalabalık olmasın diye), dünya
  eksenlerine hizalı 5m aralıklı ince bir kare grid eklendi (sadece
  halkalarla SADECE mesafe/yarıçap okunabiliyordu - kare grid X/Y
  yönünde de sayarak hassas hedefleme sağlar). **İmleç koordinat HUD'u**:
  fare, tıklamadan/sürüklemeden BAĞIMSIZ olarak (`setMouseTracking`
  zaten açıktı) hareket ettikçe ekranın altında canlı `İmleç →
  X:.. Y:..` yazısı gösterilir - RTK'lı hassas GPS bile "göz kararı"
  tıklamaktan güvenilir değildir, tıklamadan ÖNCE tam koordinatı görmek
  görünmeyen konumlara hassas nokta atmayı mümkün kılar.
- **Kare grid, v2 - RViz'in KENDİ görüntüsü (2026-09-01, kullanıcı örnek
  görsel gönderip netleştirdi: "bu şekilde kareler değil rvizdeki gibi
  grid şeklinde olacak")**: v1 (aracı çevreleyen büyüyen KARE'ler,
  `drawRect`) YANLIŞTI - kullanıcının istediği "menzil göstergesi"
  değil, milimetrik kağıt gibi TÜM ekranı kaplayan DÜZ/homojen bir
  ızgaraydı (paylaşılan örnek görsel: küçük, eşit aralıklı kareler).
  Artık: dünya eksenlerine hizalı, GÖRÜNÜR ALANIN TAMAMINI kaplayan ince
  çizgiler - 1m aralık (RViz'in varsayılan "Cell Size"ı), her 5 hücrede
  bir biraz daha parlak "ana" çizgi (klasik mühendislik grid kağıdı
  deseni - ince çizgiler arasında sayarak "5 hücre sağda" diyebilmek
  için). Çok uzağa yakınlaştırılınca (`maksimum_menzil` büyüyünce) 1m
  hücreler alt-piksel kalıp anlamsız/CPU israfı olmasın diye hücre
  aralığı otomatik büyütülüyor (her zaman ekranda en az ~6px genişlik -
  1→5→10→20→... m). Aracı çevreleyen eski "menzil halkası/karesi" TAMAMEN
  KALDIRILDI - mesafe artık aracın sağından geçen dikey eksen üzerinde
  her 5 metrede bir okunabilir bir etiketle (`Xm`) gösteriliyor.

### 5.2.6 Göreceli Nokta Ekleme (2026-09-02)

Kullanıcı isteği: "ilk noktayı atınca sağ panelden ilk noktanın 6 metre
sağına 7 metre soluna x metre ilerisine diye noktalar ekleyebilelim" -
haritaya tıklamadan, SAYI GİREREK bilinen bir mesafe/yönde nokta eklemek
için (ekranda net görünmeyen ama koordinatı/mesafesi bilinen bir hedef
için tıklamaktan çok daha hassas):

- Sağ panelde **"İleri(+)/Geri(-)"** ve **"Sağ(+)/Sol(-)"** iki sayı
  kutusu (`QDoubleSpinBox`, metre) + **"➕ Referansa Göre Ekle"** butonu.
- Panelde her zaman görünen bir **"Referans: Nokta #N (X:.. Y:..)"**
  etiketi - referans her zaman **SON eklenen nokta**; liste boşsa
  **aracın o anki konumu**. Bu sayede noktalar ZİNCİRLEME eklenebilir
  (nokta 2 = nokta 1 + ofset, nokta 3 = nokta 2 + ofset, ...).
- **YÖN SÖZLEŞMESİ v2 (2026-09-02, DÜZELTİLDİ)**: v1 "ileri/sağ"ı
  EKRANIN sabit dünya yönüne göre tanımlıyordu (kasıtlı, öngörülebilirlik
  için) - kullanıcı bunu istemedi: "araç hafif çapraz duruyorken yolda
  ileri 3 metre at diyince aracın hizasında 3 metre atsın istiyorum, tüm
  yönler için böyle olsun". Artık "ileri/sağ" **ARACIN O ANKİ GERÇEK
  YÖNÜNE (yaw)** göre hesaplanıyor - `Ros2GcsMotoru._dunya_to_arac`/
  `_gecmis_mi`'nin (dünya→gövde, zaten navigasyon-kritik ve doğrulanmış)
  dönüş matrisinin TERSİ (gövde→dünya; dönüş matrisi olduğu için ters =
  devrik) kullanılıyor:
  `dx = ileri·cos(yaw) + sağ·sin(yaw)`, `dy = ileri·sin(yaw) - sağ·cos(yaw)`.
  Hesap SADECE "Ekle" tıklanan ANDAKİ `self.yaw` ile BİR KEZ yapılıp
  SABİT bir dünya (/odom) koordinatı üretir - üretilen nokta bir dünya
  koordinatı olduğu için araç SONRADAN dönse/hareket etse bile o nokta
  yerinde kalır (ekran zaten hiç dönmüyor, SABİT ÇERÇEVE, §5.2.3) -
  sadece "ileri" hesaplanırken o AN aracın nereye baktığı kullanılır.
  İzole matematik testiyle doğrulandı: round-trip tutarlılığı (tüm
  yaw/ofset kombinasyonlarında ileri-git-geri-hesapla aynı değeri
  veriyor), yaw=0'da v1 ile birebir aynı sonuç (geriye dönük tutarlı),
  yaw=90°'de "ileri" dünya +Y eksenine dönüyor, yaw=45° (kullanıcının
  tarif ettiği "çapraz duran araç" senaryosu) için "ileri 3m" X ve Y'ye
  eşit dağılıp toplamda tam 3m'lik bir vektör üretiyor.
- Eklenen nokta panelde varsayılan yönelimle (`q_z=0, q_w=1`) tutulur -
  ama bkz. §5.2.7, GÖNDERİLMEDEN hemen önce bu otomatik olarak
  ZİNCİRLEME yönlere çevrilir, sabit `q_z=0` olarak KALMAZ.

### 5.2.7 Rota Gönderiminde Otomatik Zincirleme Yönlendirme (2026-09-02)

Canlı bildirildi: "araç 1. referans noktasına geldiğinde ters tarafa
döndü, durdu, sonra uzun bir dönüş yaptı" - göreceli panelden eklenen
noktalar sabit/varsayılan yönelimle (`q_z=0, q_w=1`) gönderiliyordu, araç
o noktaya HANGİ açıyla varırsa varsın kabul ediliyordu - bu bazen
SIRADAKİ noktanın TAM TERSİ bir yöne bakarak durup sonra gereksiz uzun
bir dönüşle toparlanmasına yol açıyordu.

**Düzeltme**: `TaktikRadarEkrani._yonlendirilmis_rota()` - "🚀 ROTAYI
GÖNDER" tıklanınca (tüm noktaların nihai konumu artık BELLİ olduğu için
tek seferde hesaplanabiliyor), her ARA nokta otomatik olarak **SIRADAKİ
noktaya bakacak** şekilde yönlendiriliyor - varış anında araç zaten
doğru yöne dönük olur, ters/uzun dönüşe gerek kalmaz. **SON** nokta için
"sıradaki" olmadığından, bir önceki bacağın yönünü **DÜZ
SÜRDÜRÜYORMUŞ gibi** (extrapole) yönlendirilir (rastgele `0` yerine).
Açı formülü (`atan2(dy_world, dx_world)`) `mouseReleaseEvent`'teki
sürükleme jestiyle **BİREBİR AYNI** - izole matematik testiyle
doğrulandı (ekran-pikseli tabanlı sürükleme formülü ile her zaman
özdeş sonuç verdiği + L-şekilli bir rotada (kullanıcının şikayet ettiği
tam senaryo: önce doğu sonra kuzeye viraj) 1. noktanın doğuya (2.
noktaya), 2. noktanın kuzeye (3. noktaya) doğru döndüğü kanıtlandı).

**Bilinçli olarak DOKUNULMAYAN kısım**: her referans noktasında aracın
kısa bir süre durup beklemesi (canlı bildirildi, "akıcı gidebilirse öyle
olsun") - bu, kullanıcının ONAYLADIĞI ve önceden test edilmiş güvenlik
mekanizmasının (bkz. §5.2.5, her bacağın `/ugv_goal_result` SUCCESS'i
BEKLENMEDEN bir sonraki nokta ASLA gönderilmez) doğal/kaçınılmaz bir
sonucu - kullanıcının kendi isteği "hiçbir şeyi bozmadan"/"bekleme
konusu öyle kalsın şimdilik elleme" olduğu için bu güvenlik kontrolü
KALDIRILMADI, sadece bilgilendirildi.

**Yönelim OKU görselleştirmesi (2026-09-02, aynı gün, kullanıcı takibi)**:
kullanıcı "araç 1. noktaya geldiğinde önünü 2.'ye çevirmiyor, bekliyor,
SONRA rota oluşturunca çeviriyor" gözlemini bildirip "sadece noktayı
değil küçük bir ok şeklinde yönelimini de göster" istedi - yukarıdaki
zincirleme yön düzeltmesinin GERÇEKTEN doğru hesaplandığını göndermeden
ÖNCE gözle doğrulayabilmek için. `TaktikRadarEkrani.paintEvent`'te her
bekleyen nokta işaretinin üzerine artık yeşil bir YÖN OKU çiziliyor -
panelden eklenen noktaların HAM/kayıtlı yönelimi (`q_z=0`) DEĞİL,
`_yonlendirilmis_rota()`'nın (yani GÖNDERİM ANINDA gerçekten
hesaplanacak zincirleme yönün) AYNISI çizilir - kullanıcı "Gönder"e
basmadan önce her okun mantıklı yöne baktığını kontrol edebilir.
**Not (araç tarafına dokunulmadan, sadece açıklama)**: oku doğru
hesaplamak, aracın o noktaya VARIRKEN gerçekten o açıya dönmüş olacağını
GARANTİ ETMEZ - bu, `goal_manager_node.py`'nin (araç tarafı, DOKUNULMADI)
"hedefe ulaşıldı" toleransının konumu mu yoksa konum+yönelimi mi kontrol
ettiğine bağlıdır; kullanıcının gözlemlediği "önce bekliyor, SONRA
rota oluşunca çeviriyor" davranışı, aracın SADECE konuma ulaşınca
SUCCESS saydığına ve gerçek dönüşün ancak BİR SONRAKİ hedef gönderilince
gerçekleştiğine işaret ediyor olabilir - araç tarafı incelenmeden kesin
teşhis edilemez.

### 5.3 Suni Ufuk (IMU görselleştirme)
- `SuniUfukEkrani`, `/imu/data` mesajından çıkarılan roll/pitch açılarını
  klasik uçak suni ufuk göstergesi tarzında çizer (`QPainter.rotate`/`translate`).
- **Silah kamerası, Suni Ufuk'un ARKA PLANI olarak (2026-09-02, kullanıcı
  isteği - v1 AYRI bir kutuydu, "ben tam olarak bunu istemedim" ile
  düzeltildi)** - v1'de Suni Ufuk'un altına AYRI bir `QLabel` eklenmişti;
  kullanıcının GERÇEK isteği farklıydı: "ufuk kısmının parametreleri aynen
  duracak fakat o kahverengi ve mavi olan kısma görüntü gelecek,
  görüntünün üstünde yine ufuk yazıları olacak". Artık `SuniUfukEkrani.
  paintEvent`'teki gökyüzü (`#0066cc` mavi) / yer (`#8b4513` kahverengi)
  düz renk dolgusunun YERİNİ silah kamerası görüntüsü alıyor - AYNI
  `translate`/`rotate(self.roll)` bloğunun İÇİNDE olduğu için görüntü de
  roll ile döner, pitch ile AYNI şekilde kayar (eski düz renk dolgu
  neresi kaplıyorsa görüntü orayı kaplar).
  **v2 düzeltmesi (canlı bildirildi: "kare olduğu için çok yakını
  gösteriyor, zoom yapmış gibi")**: v1 görüntüyü hedefe sığdırmak için
  ORTADAN KARE KIRPIYORDU - geniş (16:9/4:3) bir kamera görüntüsünü
  kareye kırpmak genişliğin büyük kısmını (16:9'da ~%44'ünü) atıp
  "yakınlaştırılmış" bir görünüme yol açıyordu. Artık HİÇ KIRPMA YOK -
  kaynağın TAMAMI çiziliyor, hedef dikdörtgen kaynağın EN-BOY ORANINI
  KORUYARAK büyütülüyor (kısa kenar 800px sabit - eski kare dolgunun
  "her roll açısında görünür alanı kaplar" garantisi korunuyor, sadece
  uzun kenar orana göre büyüyor). İzole testle (640x480, 480x270, 4:3,
  9:16 gibi farklı oranlarda kısa kenarın hep 800 kaldığı) doğrulandı.
  Pitch
  ladder çizgileri/derece yazıları, uçak sembolü, dairesel çerçeve
  AYNEN korunuyor - hepsi görüntünün ÜSTÜNE (sonraki çizim adımlarında)
  çiziliyor. Kamera bağlı değilken (`silah_pixmap is None`) eski düz
  mavi/kahverengi dolguya OTOMATİK geri dönülüyor (bozulma yok).
  **PERFORMANS (kullanıcının kendi hatırlattığı mevcut kalıp)**:
  `main.py::video_ekrana_bas` zaten SADECE Kamera sayfasındayken (index 1)
  pahalı 4x `resmi_yuvarla`/QPainter bindirmesi yapıyordu, diğer
  sayfalarda tamamen atlıyordu (kalp atışı hariç). Bu AYNI felsefeyle
  genişletildi: Navigasyon'dayken (index 2) SADECE silah karesi (düz
  `QPixmap.fromImage`, rounded-rect işlemesi YOK - kırpma zaten
  `SuniUfukEkrani.paintEvent`'in kendi işi) `HaritaYoneticisi.
  silah_kamera_guncelle()` → `SuniUfukEkrani.silah_kare_guncelle()`'e
  iletiliyor, Kamera sayfasındayken (index 1) eskisi gibi tam bindirme
  yapılıyor - **HİÇBİR ZAMAN ikisi aynı anda çalışmaz**, sadece o an
  gerçekten GÖRÜNEN sayfanınki çalışır.

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
- Araç konumu artık `/mavros/global_position/raw/fix` (`NavSatFix`,
  `qos_profile_sensor_data`) mesajından gelir (2026-09-01 düzeltmesi, bkz.
  §6.5 - eskiden `/fix` kullanılıyordu ama o topic hiç yayınlanmıyordu),
  ekran merkezinde dönen bir ok + kat edilen yolun izi (`iz_noktalari`
  polyline) olarak çizilir.
- **RTK durumu (2026-09-01, kullanıcı isteği: "gpsin hatası ve fix mi
  float mı olduğu durumu yazsın")** — sol üstte, IMU satırının altında,
  yeni bir satır: `GPS: {ETİKET} | ±{hata}m`. Etiket `/mavros/gpsstatus/
  gps1/raw` (`GPSRAW.fix_type`, MAVLink `GPS_FIX_TYPE` enum) değerinden
  geliyor - "GPS YOK/FIX YOK" (kırmızı), "2D/3D FIX" (turuncu), "DGPS"
  (turkuaz), "RTK FLOAT" (mavi), "RTK FIXED" (yeşil); hata `GPSRAW.h_acc`
  (mm→m çevrilmiş, `konum_birlestirici.py`'nin de kullandığı AYNI alan -
  bu donanımda gerçekten dolduruluyor, canlı doğrulandı: RTK Float'ta
  `h_acc=128mm`). Veri kaynağı: `Ros2GcsMotoru.gpsraw_callback` →
  `rtk_durum_sinyali` → `UyduHaritaWidget.rtk_durum_guncelle`.
- **"Çok fazla yakınlaştıramıyorum" (2026-09-02, kullanıcı isteği, kök
  nedeni bulundu) —** kök neden İKİ KATLIYDI:
  1. `wheelEvent` HER yakınlaştırma adımında `tile_onbellek`'in TAMAMINI
     temizliyordu - bu, aşağıdaki (2)'deki yedekleme mekanizmasını da
     imkansız kılıyordu (tam da lazım olduğu anda önceki zoom'un
     karoları da SİLİNMİŞ oluyordu). **Düzeltme**: önbellek artık
     KORUNUYOR - farklı zoom seviyeleri farklı anahtarlarda (`z,x,y`)
     tutulduğu için çakışma yok, her seviyenin karoları bir kere iner.
  2. Esri World Imagery (hiçbir uydu görüntü sağlayıcısı gibi) HER
     YERDE en yüksek zoom seviyesinde (`max_zoom=20`) görüntü SUNMAZ -
     kırsal/az kullanılan bölgelerde (yarışma sahası gibi) üst zoom
     seviyelerinde 404 dönebilir. Eskiden bu durumda karo hücresi DÜZ
     GRİ (`#1a2029`) çiziliyordu - kullanıcı yakınlaştırdıkça görüntü
     BÜYÜMÜYOR, sadece gri kalıyordu, "daha fazla yakınlaşamıyorum"
     hissi tam olarak buradan geliyordu. **Düzeltme**: standart "slippy
     map" davranışı (Google Maps/Leaflet vb.) eklendi - `_en_yakin_
     atalik_tile()` hedef karo yoksa ÜST (daha düşük, zaten önbelleğe
     alınmış) bir zoom seviyesindeki karonun İLGİLİ KÖŞESİNİ kırpıp
     büyüterek gösterir (`min_zoom`'a kadar yukarı dener) - boş gri
     yerine BULANIK ama GERÇEK bir görüntü, kesintisiz/akıcı
     yakınlaştırma hissi verir; gerçek karo arka planda istenmeye devam
     eder, inince yerini alır. Kırpma/ölçekleme matematiği izole testle
     (4 senaryo: 1 kat yukarı, çeyrek konumları, 2 kat yukarı, hiç ata
     yoksa None) doğrulandı.
  3. **"map data not yet available" (2026-09-02, canlı bildirildi, kök
     nedeni bulundu) —** (2)'deki 404/hata varsayımı EKSİKTİ: Esri,
     kapsama DIŞI zoom seviyelerinde HTTP HATASI vermek yerine GEÇERLİ
     bir JPEG (200 OK) ile düz gri arka plan + "Map data not yet
     available" yazan bir YER TUTUCU görsel döndürüyor - canlı `curl` ile
     doğrulandı (yarışma konumunda z=19 gerçek/net görüntü, z=20 bu yer
     tutucu - HTTP kodu ikisinde de 200). Eskiden bu GERÇEK görüntüymüş
     gibi hem bellekte hem DİSKTE önbelleğe alınıyordu - bir kez
     görülünce o karo SONSUZA KADAR bu yazıyı göstermeye devam ederdi.
     **Düzeltme**: `_yer_tutucu_mu()` - gerçek uydu görüntüsü pratikte
     HİÇBİR ZAMAN tek-düze (tek renge yakın) değildir; 16px aralıklı bir
     örnekleme ızgarasında en baskın renge (±10 tolerans) yakın örnek
     oranı ölçülür, `%90` ve üzeri ise yer tutucu sayılıp `basarisiz`
     olarak işaretlenir (hem yeni indirilen hem eski/diskte önbelleğe
     alınmış - o zaman disk dosyası da silinir) - bu da yukarıdaki (2)
     yedekleme mekanizmasını (üst zoom'dan bulanık ama GERÇEK görüntü)
     otomatik devreye sokar. Gerçek örnek verilerle doğrulandı: bu
     konumda z=19 gerçek görüntü ~%50 baskın-renk oranı (geniş bir tarla
     olduğu için beklenen), z=20 yer tutucu ~%98 - eşik ikisini net
     ayırıyor.

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

### 5.6 Dokunmatik Ekran Klavyesi (onboard) (2026-09-03)
- Arayüz artık fiziksel klavyesiz, doğrudan dokunmatik ekranda kullanılıyor.
  Metin girişi (SSH terminali, Ayarlar sayfasındaki NTRIP kullanıcı/şifre
  alanları, hedef klasör vb.) için ekranda bir klavye gerekiyor.
- Klavye programı: **onboard** (Ubuntu deposunda hazır; `sudo apt install
  onboard`). gsettings'te önceden ayarlı: `org.onboard.window force-to-top
  true`, `docking-enabled true`, `docking-edge 'bottom'`.
- **Neden basit değil:** arayüz `showFullScreen()` ile gerçek X11
  fullscreen (`_NET_WM_STATE_FULLSCREEN`) açılıyordu. Mutter fullscreen
  pencereyi en üst dizilim katmanına koyar; onboard "force-to-top" (dock +
  `_NET_WM_STATE_ABOVE`) olsa bile klavye **her zaman** arayüzün arkasında
  kalır (canlı `_NET_CLIENT_LIST_STACKING` ile doğrulandı).
- **Çözüm** (`main.py`):
  - Pencere `Qt.FramelessWindowHint` ile çerçevesiz yapılır ama açılışta
    yine `showFullScreen()` kullanılır (normal kullanımda hiçbir fark yok).
  - `ekran_klavyesi_ac_kapat()`: klavye açılırken pencere fullscreen
    **durumundan** çıkarılır (`setWindowState(... & ~Qt.WindowFullScreen)`)
    ve elle tam ekran **boyutuna** getirilir (`setGeometry(primaryScreen)`).
    Fullscreen katmanında olmadığı için onboard'ın dock'u artık üste gelir.
    Klavye kapatılınca `showFullScreen()` ile geri dönülür.
  - onboard D-Bus ile sürülür: `org.onboard.Onboard` /
    `/org/onboard/Onboard/Keyboard` → `Show` / `Hide` (`_onboard_dbus()`,
    `dbus-send` alt süreci). onboard çalışmıyorsa `subprocess.Popen(["onboard"])`.
- **Kısayol butonu:** sağ ÜST köşede yüzen, çerçevesiz bir aç/kapat butonu
  (`pushButton_ekranKlavyesi`, `centralwidget` child'ı, `_ekran_klavyesi_
  butonu_ekle()`). Sol menüye konmadı çünkü klavye açıkken (pencere
  fullscreen değilken) Ubuntu'nun sol dock'u sol menüyü kısmen örtüyor;
  sağ üst köşe her iki durumda da erişilebilir. Buton `checked` iken mavi.
- **Yan etki:** klavye açıkken arayüz fullscreen olmadığından GNOME üst
  paneli (27 px) ve sol dock görünür olur; klavye kapatılınca kaybolurlar.
  Kabul edilebilir bir ödünç — onboard'ı fullscreen pencerenin üstüne
  çıkarmanın X11'de güvenilir başka yolu yok.
- **Ctrl+Alt+T gibi çoklu tuş (dokunmatikte aynı anda basılamıyor):**
  onboard değiştirici (modifier) tuşlarını **kilitler** — Ctrl'e dokun
  (basılı kalır), Alt'a dokun (basılı kalır), T'ye dokun → kombinasyon
  gönderilir. Çift dokunuş = kalıcı kilit. İstenirse GNOME "Yapışkan
  Tuşlar" da açılabilir: `gsettings set org.gnome.desktop.a11y.keyboard
  stickykeys-enable true`.

## 6. Veri Kanalları — Hangi Veri Nereden Geliyor

### 6.1 ROS2 Topic'leri (araç Jetson ↔ arayüz Jetson, DDS üzerinden)

| Topic | Tip | Yön | Kullanıldığı yer |
|---|---|---|---|
| `/odom` | `nav_msgs/Odometry` | araç → arayüz | Hız göstergesi (`telemetri_sistemi.py`) |
| `/scan` | `sensor_msgs/LaserScan` | araç → arayüz | Taktik radar, ana menü dekoratif radar, LİDAR heartbeat |
| `/imu/data` | `sensor_msgs/Imu` | araç → arayüz | Suni ufuk, costmap derotasyonu, IMU heartbeat/gösterge |
| `/mavros/global_position/raw/fix` | `sensor_msgs/NavSatFix` (`qos_profile_sensor_data`) | araç → arayüz | Uydu haritada araç konumu + Taktik LiDAR GNSS yazısı + NTRIP GGA/RMC kaynağı (2026-09-01: eski `/fix` hiç yayınlanmıyordu, bkz. §6.5) |
| `/local_costmap/costmap` | `nav_msgs/OccupancyGrid` | araç → arayüz | Taktik radarda engel/costmap katmanı |
| `/plan`, `/local_plan` | `nav_msgs/Path` | araç → arayüz | Nav2 global/local rota görselleştirme |
| `/goal_manager_heartbeat` | `std_msgs/Int32` | araç → arayüz | OTONOM HAZIR göstergesi |
| `/arduino_baglanti_durumu` | `std_msgs/Bool` | araç → arayüz | MANUEL HAZIR göstergesi (motor Arduino seri bağlantısı açık mı) |
| `/turret_hedef_mesafe` | `std_msgs/Float32` | araç → arayüz | SİLAH kamerası kutusunda hedef mesafesi (TF02-Pro lidar, entegrasyon sürüyor) |
| `/goal_pose` | `geometry_msgs/PoseStamped` | arayüz → araç | RViz benzeri tıkla-git hedef gönderimi |
| `/surus_modu` | `std_msgs/String` (`"MANUEL"`/`"OTONOM"`) | arayüz → araç | Aracın hangi sürüş modunda olduğunu araca bildirir (kontrol paneli sol butonu da değiştirir, bkz. §7.5) |
| `/palet_hizlari` | `std_msgs/Float32MultiArray` `[sol, sağ]` | arayüz → araç | MANUEL modda ham PWM komutu (klavye veya kontrol paneli sol joystick kaynaklı) |
| `/silah_modu` | `std_msgs/String` (`"MANUEL"`) | arayüz → araç | turret_node manuel turret + ateşi kabul etsin diye; `surekli_yayin_dongusu`'nda her turda yayınlanır (2026-09-04: kontrol paneli sağ joystick her zaman silahı sürüyor) |
| `/turret_manuel_cmd` | `geometry_msgs/Twist` (`linear.x`=pan, `linear.y`=tilt) | arayüz → araç | Kontrol paneli **sağ joystick** ham oranı (sürüş modundan bağımsız) |
| `/silah_ates_manuel` | `std_msgs/Bool` | arayüz → araç | Silah ateş — ekrandaki ateş butonu VEYA kontrol paneli **sağ toggle switch** (GND'de), 50ms tekrarlı |
| `/arac_komut` | `std_msgs/String` | arayüz ↔ araç | Ham klavye tuş kodu VE literal `"EMERGENCY_STOP_CMD"`/`"DEVAM_CMD"` kilit komutları (bkz. §7) — `tabela_etap_yoneticisi.py` de Stop tabelasında buraya yayınlıyor. **Kontrol paneli sol toggle switch**: GND'ye alınca `EMERGENCY_STOP_CMD`, bırakınca **otomatik** `DEVAM_CMD` (§7.5) |
| `/yon_pid_aktif` | `std_msgs/Bool` | arayüz → araç | Yön düzeltme PID AÇ/KAPA anahtarı (bkz. §7) |
| `/farlar` | `std_msgs/Bool` | arayüz → araç | Farlar AÇIK/KAPALI ("F" tuşu VEYA kontrol paneli **sağ buton**, bkz. §7.5) — **araç tarafında henüz bir dinleyici (subscriber) YOK** (canlı doğrulandı: `ros2 topic info /farlar` → `Subscription count: 0`), yani şu an sadece yayın yapılıyor, fiziksel far/röle sürülmüyor - araç tarafına bir "farlar_node" (veya mevcut bir motor kontrol düğümüne ek) eklenmesi gerekiyor |
| `/mavros/gpsstatus/gps1/raw` | `mavros_msgs/GPSRAW` | araç → arayüz | GPS/GNSS ETKİN göstergesi (`fix_type`, bkz. §8) + Uydu Haritası RTK durumu/hata yazısı (bkz. §5.4, `telemetri_sistemi.py`'nin bağımsız aboneliğiyle AYNI topic, ayrı QoS gereksinimi yok - varsayılan RELIABLE) |
| `/tabela_tespit` | `std_msgs/String` (`"SinifAdi:güven"` / `"YOK"`) | araç içi (`tabela_node.py`→`tabela_etap_yoneticisi.py`) | Tabela algılama → etap/durdurma kararı (bkz. §7) |
| `/guncel_etap` | `std_msgs/Int32` | araç → arayüz | Algılanan son etap numarası (1-11) → dashboard'daki ETAP göstergesi (`label_etapNo`) — Designer'da sabit "8" yazıyordu, `etap_sinyali` TANIMLIYDI ama hiç emit edilmiyordu (`gps_durum_sinyali` ile AYNI durumdaydı); artık gerçek veriyle besleniyor, canlı doğrulandı |
| `/parkur_durumu`, `/otonom_dur_nedeni` | `std_msgs/String` | araç → arayüz (henüz arayüzde gösterilmiyor) | Parkur tamamlandı / otonom durdurma sebebi |

### 6.2 UDP Kanalları (ham soket, ROS2 dışı)

| Port | İçerik | Kaynak |
|---|---|---|
| `5000` | Ön/tabela kamera JPEG kare | `tabela_node.py` |
| `5001` | Silah/turret kamerası, işlenmiş JPEG kare | `turret_node.py` (YOLO tespit kutusu çizili) |
| `5002` | Arka kamera, ham JPEG kare (model yok, sadece görüntü) | `arka_kamera_node.py` |

**2026-09-01 düzeltmesi**: port ataması TERSİNE ÇEVRİLDİ (eskiden 5000=silah,
5001=tabela idi) - araç tarafındaki `tufan_mppi.launch.py`'nin GÜNCEL hâliyle
(başka bir oturumda değiştirilmiş) eşleşsin diye `kamera_sistemi.py` ve
`main.py::video_ekrana_bas` buna göre güncellendi. Ayrıca **ARKA kamera artık
gerçekten bağlı** - eskiden `main.py` anahtar 3'e (arka kutusu) hep sabit
`None` veriyordu (donanım/port hiç yoktu); `arka_kamera_node.py` zaten port
5002'de yayın yapıyordu ama arayüz tarafında dinleyen üçüncü bir soket hiç
yoktu - artık var, gerçek görüntü akıyor.

**2026-09-01, İKİNCİ düzeltme (araç tarafı, KRİTİK) — "silahta ön kamera,
diğer ikisinde arka kamera gözüküyor"**: portlar/main.py doğruydu ama
gerçek KÖK NEDEN araç Jetson'daydı. `tabela_node.py` ve `arka_kamera_node.py`
AYNI MODEL iki C270 kamerayı isimle ayırt edemediği için (`ID_SERIAL` bile
Logitech tarafından paylaşılmış/sahte) fiziksel USB port yolunu
(`kamera_usb_port` parametresi) "parmak izi" olarak kullanıyor - ama bu iki
parametrenin varsayılan DEĞERLERİ BİRBİRİYLE TERS YAZILMIŞTI:
- `tabela_node.py` (ÖN olmalı) → `kamera_usb_port='1-2.2.4.3'` (bu aslında
  ARKA kameranın portu!) → gerçekte **arka kameranın** görüntüsünü
  yayınlıyordu (port 5000/ÖN kutusunda)
- `arka_kamera_node.py` (ARKA olmalı) → `kamera_usb_port='1-2.2.3'` (bu
  aslında ÖN kameranın portu!) → gerçekte **ön kameranın** görüntüsünü
  yayınlıyordu (port 5002/ARKA kutusunda)

`turret_node.py` (silah, C922 - tek örnek, isim çakışması yok) etkilenmedi,
zaten doğruydu. Canlı teşhis: `/proc/<pid>/fd`'de hangi düğümün GERÇEKTE
hangi `/dev/videoN`'i açtığı kontrol edilerek kesin doğrulandı (ROS
parametresi "camera_index=2" dese bile gerçek açılan cihaz video4 çıkıyordu).
**Düzeltme**: iki dosyadaki `kamera_usb_port` varsayılanları birbirleriyle
takas edildi (`tabela_node.py`→`'1-2.2.3'`, `arka_kamera_node.py`→
`'1-2.2.4.3'`), kaynak VE kurulu (`install/`) kopyalar güncellendi, iki
düğüm de düzeltilmiş kodla yeniden başlatıldı. **Canlı doğrulandı**:
yeniden başlattıktan sonra `/proc/<pid>/fd` tekrar kontrol edildi -
`tabela_node` artık gerçekten `/dev/video2`'yi, `arka_kamera_node` artık
gerçekten `/dev/video4`'ü açıyor.

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

**2026-09-01 düzeltmesi**: `~/.ssh/config`'teki ControlMaster paylaşımı
(bkz. §Git backup workflow/Ağ notları) eskiden SADECE WiFi IP'sini
(`10.40.64.43`) kapsıyordu - Ubiquiti IP'sinden (`192.168.1.22`) yapılan
ad-hoc `ssh` çağrıları (ör. canlı teşhis komutları) paylaşılan bağlantıyı
KULLANMIYORDU, her çağrı `who`da YENİ bir oturum satırı bırakıyordu (canlı
bildirildi: 14 birikmiş oturum). Aynı `Host` bloğu artık `192.168.1.22`
için de tanımlı - `ssh -O check arf203@192.168.1.22` ile "Master running"
doğrulandı.

### 6.5 mavros (Cube/ArduPilot) ve NTRIP/RTK köprüsü

Araç Jetson'a **USB ile bağlı bir Cube (ArduPilot çalıştıran uçuş
kontrolcüsü)** eklendi. Araç tarafında `mavros_node` çalışıyor
(`fcu_url:=/dev/ttyACM0:57600`).

**DÜZELTME (2026-09-01) — `/fix` remap'i ARTIK YOK, dokümantasyon
güncellendi:** Bu bölüm eskiden `/mavros/global_position/global`'in
`/fix`'e yeniden adlandırıldığını (`-r ...:=/fix`) belirtiyordu - canlı
kontrol edildi, bu remap **artık launch dosyasında yok** ve `/fix` topic'i
**HİÇBİR ZAMAN yayınlanmıyor** (`Publisher count: 0`, canlı doğrulandı).
Bu, `harita_sistemi.py`'nin GNSS yazısının hep "sinyal yok" göstermesine
VE `ntrip_rtk_sistemi.py`'nin GERÇEK konum yerine sabit bir yer
tutucuyla (Ankara) çalışmasına yol açıyordu - RTK Float yine de elde
ediliyordu (VRS toleranslı) ama gerçek konumla muhtemelen daha isabetli
olabilirdi. **Düzeltme**: her iki dosya da artık `konum_birlestirici.py`
(araç tarafı) ile AYNI çözümü kullanıyor - doğrudan mavros'un kendi
yayınladığı `/mavros/global_position/raw/fix` topic'ine, DOĞRU QoS
(`qos_profile_sensor_data` - mavros varsayılanı BEST_EFFORT, RELIABLE
DEĞİL) ile abone oluyorlar. Canlı doğrulandı: `ros2 topic info` ile üç
abone de (konum_birlestirici, tufan_ntrip_rtk_koprusu, tufan_gcs_master_node)
artık aynı yayıncıyla UYUMLU QoS'ta görünüyor.

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
  izolasyon dersi). Node ayrıca **doğrudan `/mavros/global_position/raw/fix`'e
  abone olur** (`_gps_callback`, `ARAC_GPS_TOPIC`, `qos_profile_sensor_data`
  - 2026-09-01: eskiden `/fix` idi ama o topic hiç yayınlanmıyordu, bkz.
  §6.1/§6.5 başı) — arayüz Jetson'da GPS olmadığı için GGA'daki konum,
  araç Jetson'un mavros'unun cross-machine ROS2 discovery ile yayınladığı
  gerçek konumdan okunur; ilk bağlantı anında henüz veri gelmemişse
  geçici bir yer tutucu (Ankara civarı) kullanılır.
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

### 6.6 Sistem Servisleri (systemd, arayüz Jetson'da açılışta otomatik)

- **`tufan-multicast-route.service`** (2026-09-01, kullanıcı isteği):
  eskiden Ubiquiti bağlantısı üzerinden multicast trafiği (NTRIP/DDS
  discovery gibi 224.0.0.0/4 aralığını kullanan protokoller) için her
  açılışta elle `sudo ip route add 224.0.0.0/4 dev enP8p1s0`
  çalıştırılması gerekiyordu. Artık `/etc/systemd/system/
  tufan-multicast-route.service` içinde, `network-online.target`'tan
  SONRA (ağ hazır olduktan sonra) otomatik çalışan bir `oneshot` servis
  var - `systemctl enable --now` ile etkinleştirildi, canlı doğrulandı
  (`ip route show` → `224.0.0.0/4 dev enP8p1s0 scope link`). İdempotent
  yazıldı: rota zaten varsa veya `enP8p1s0` arayüzü o an takılı/hazır
  değilse (Ubiquiti bağlı değilse) sessizce başarılı sayılır (`exit 0`),
  boot loglarında "failed" servis olarak görünüp gürültü yaratmaz. Servis
  dosyasının kaynağı: bu depoda değil, doğrudan `/etc/systemd/system/`
  altında (sistem geneli, tek makineye özgü) - kopyası talep edilirse
  `systemctl cat tufan-multicast-route.service` ile görülebilir.

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
- **"MANUEL'e basınca ana sayfaya atıyor" (2026-09-01, KESİN çözüldü,
  bkz. §8.1)**: `manuel_sec()` MANUEL moda geçince klavyeyi otomatik aktif
  ediyor (`ayar_klavye_tetikle()`). İlk düzeltme (sadece geçersiz bir
  sayfadaysak - Hakkında/Ayarlar - ana ekrana dön) kod incelemesinde doğru
  görünse de kullanıcı sorunun DEVAM ettiğini 2. kez bildirdi; kesin/
  tartışmasız çözüm için `ayar_klavye_tetikle()`'deki otomatik sayfa
  DEĞİŞTİRME tamamen kaldırıldı - artık klavye hangi sayfadan aktif
  edilirse edilsin sayfa HİÇBİR ZAMAN değişmiyor (bkz. §8.1 detaylı not).
- **Mod bazlı erişim**: klavye/joystick komutları sadece `arac_modu ==
  "MANUEL"` iken etkilidir; OTONOM modda `surus_koprusu.py` (araç tarafı)
  Nav2 çıktısını `/palet_hizlari`'a çevirir, arayüz sessiz kalır.
- **FARLAR - "F" tuşu (2026-09-01, kullanıcı isteği)**: `keyPressEvent`'te
  ESC gibi ERKEN işleniyor ve KASITLI OLARAK yukarıdaki "sadece MANUEL
  modda" şartından ÖNCE çalışıyor - farlar bir sürüş komutu değil yardımcı
  ekipman, OTONOM modda da (gece/tünel/rampa) açılabilmeli. Klavye aktif
  VE geçerli bir sayfadaysa (Ana/Kamera/Navigasyon) her F basışında
  (`event.isAutoRepeat()` ile korunmuş - tuşu basılı tutmak OS'ta tekrarlı
  keyPress ürettiği için çırpınma önleniyor) `self.farlar_acik` AÇIK/KAPALI
  arasında geçiş yapıp `telemetri_sistemi.py::farlar_ayarla()` üzerinden
  `/farlar` (`std_msgs/Bool`) topic'ine yayınlanıyor. **NOT**: araç
  tarafında bu topic'i dinleyip fiziksel farı/röleyi süren bir düğüm HENÜZ
  YOK (canlı doğrulandı, bkz. §6.1) - şu an sadece arayüz tarafı hazır.
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

## 7.5 Yer İstasyonu Fiziksel Kontrol Paneli (2026-09-04)

Arayüz monitörü (dokunmatik ekran) yer istasyonuna monte edildi ve yanına
**tek bir Arduino'ya** bağlı fiziksel bir kontrol paneli eklendi. Bu, ARTIK
KULLANILMAYAN tek-joystick sistemin (`surus_joystick_sistemi.py` +
`arduino_joystick/`, "HEDEF: ARAÇ/SİLAH" butonu) yerini alır — araç ve silah
artık **ayrı** joystick'lerde, paylaşım/hedef seçimi kalktı.

### Donanım / pin haritası

| Donanım | Pin | İşlev |
|---|---|---|
| Sol joystick | A3, A4 | Aracı sür → tank karışımı → `/palet_hizlari` |
| Sağ joystick | A0, A1 | Silah (turret) pan/tilt → `/turret_manuel_cmd` |
| Potansiyometre | A5 | PWM üst sınırı (85–255) **canlı** |
| Sol toggle switch | D4 | GND'ye alınca ACİL STOP (`EMERGENCY_STOP_CMD`); bırakınca **otomatik** `DEVAM_CMD` |
| Sağ toggle switch | D5 | GND'ye alınca silah ateş (`/silah_ates_manuel` = true, 50ms tekrarlı) |
| Sol buton | D9 | Her basışta Manuel ↔ Otonom |
| Sağ buton | D10 | Farları aç/kapat (F tuşuyla aynı `_farlar_toggle()`) |

Toggle/buton'lar `INPUT_PULLUP`: GND'de/basılı = LOW(0), serbest = HIGH(1).
Anlamlandırma (kenar tespiti, debounce, hangi eksen X, yön ters mi) Python
tarafında.

### Yazılım

- **`arduino_kontrol_paneli/arduino_kontrol_paneli.ino`** — **Arduino UNO**
  (orijinal; eski Mega sketch'i A6/A7 kullanıyordu, UNO'da o pinler YOK —
  bu sketch A0,A1,A3,A4,A5 + D4,D5,D9,D10 kullanır, hepsi UNO'da var).
  ~50 Hz, 115200 baud, satır formatı: `P,A3,A4,A0,A1,A5,D4,D5,D9,D10\n`.
  `P,` öneki eski tek-joystick formatından (`X,Y,BTN`) ayırt için. Bu
  Arduino motorlara/silaha DOĞRUDAN bağlı DEĞİL — sadece panel okur.
  Yükleme: `arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno
  arduino_kontrol_paneli`.
- **`arduino_tani_gecici/`** — geçici tanı sketch'i: 6 analog (A0–A5) +
  12 digital (D2–D13) pini etiketli basar. Kablolama doğrulama için
  (joystick/pot besleme + hangi pinde). Kullandıktan sonra asıl sketch
  geri yüklenmeli.
- **`kontrol_paneli_sistemi.py`** `KontrolPaneliThread(QThread)` — seri
  oku + kalibre et + oran/kenar üret. Sinyaller: `surus_sinyali(sol,sag)`
  (tank karışımı sonrası), `turret_sinyali(x,y)` (ham), `pot_sinyali(int)`
  (PWM 85–255, EMA + histerezis + throttle), `sol_toggle_sinyali(bool)`,
  `sag_toggle_sinyali(bool)`, `sol_buton_sinyali()`, `sag_buton_sinyali()`,
  `baglanti_sinyali(bool)`, `guvenlik_sinyali(str)`, `ham_veri_sinyali(dict)`
  (PANEL TESTİ dialogu için).
- **`main.py`** `_kontrol_paneli_baslat()` — açılışta **otomatik** başlar
  (kameralar gibi), portu sürekli tarar. Ayarlar'daki "Joystick" butonu
  artık paneli **yazılımsal** aç/kapatır (`ayar_joystick_tetikle()`).
  Arduino bağlanınca `surus_kaynagi="JOYSTICK"` + `silah_manuel_moduna_al()`;
  koparsa `surus_kaynagi="KLAVYE"`'ye geri döner (klavyeyle sürüş sürebilsin).

### Kalibrasyon / güvenlik — `surus_joystick_sistemi.py`'den BİREBİR taşındı

Tek-eksenlik `_EksenDurumu` sınıfına paketlendi, her iki joystick'in **dört
ekseni için ayrı ayrı** uygulanıyor (X ve Y bağımsız). Korunan mantık:
bağlantıda ilk ~0.5 sn merkez ölçümü, `ADC_OLU_BOLGE=60`, merkeze-göre
asimetrik normalize, bağımsız canlı oto-merkezleme (`MAKS_MERKEZ_KAYMASI=150`),
ve **kalibrasyondan tamamen bağımsız SON GÜVENLİK AĞI** (`GUVENLIK_SABIT_*`,
ilk kalibrasyon merkezine yakınlık şartı — bkz. §7 ve dosya içi notlar).
Ek katman: **D4 (acil stop) GND'deyken thread `surus_sinyali` yaymaz** —
arayüz kilidi + 0.8 sn watchdog'a ek yerel kalkan. Silah (turret + ateş)
kasıtlı olarak etkilenmez ("araçtaki motorlara giden kodlar" içindir).

### Eksen yerleşimi / yön — PANEL TESTİ ile ayarlanır

Joystick'lerin hangi pini X (dönüş/pan), yönlerinin ters olup olmadığı
FİZİKSEL MONTAJA bağlı ve **bilinmiyor**. Ayarlar → **PANEL TESTİ** butonu
(`_panel_testi_ac()`, eski "HEDEF" butonunun yerinde) 9 ham kanalı +
hesaplanan sol_oran/sağ_oran/turret x-y'yi canlı gösterir. Her ekseni tek
tek oynatıp gözleyin, sonra `kontrol_paneli_sistemi.py` başındaki
`SOL_JOY_X_PIN` / `SAG_JOY_X_PIN` ("A3"/"A4", "A0"/"A1") ve
`SOL_JOY_TERS_X/Y`, `SAG_JOY_TERS_X/Y` sabitlerini ayarlayın (toggle
switch'ler ters kabluluysa `SOL_TOGGLE_TERS` / `SAG_TOGGLE_TERS`). Doğru ayarda:
sol joystick ileri → sol_oran ve sağ_oran birlikte +; sağa çevir →
sol_oran > sağ_oran.

### Potansiyometre → PWM üst sınırı + ekran-altı gösterge

Pot çevrildikçe `telemetri_motoru.pwm_ust_sinirini_ayarla(deger, sessiz=True)`
canlı çağrılır (onay penceresi YOK; `sessiz=True` → log spam'i yok). Ayrıca:
(1) Ayarlar'daki kutu (`lineEdit_kullaniciAdi_2`) güncellenir, (2) ekranın
alt-orta kısmında ses-OSD'si gibi belirip ~1.4 sn sonra solan bir gösterge
(`_pwm_osd_goster()` / `pwmOsd`): `PWM ÜST SINIRI · <değer>` + dolu çubuk +
`min 85 — max 255`.

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
| **Ne gösterir** | **2026-09-01 düzeltmesi** (kullanıcı isteği: "kaç kameranın bağlı olduğunu göstersin") — eskiden SADECE silah kamerasının (UDP :5000, o zamanki port ataması) kare gelip gelmediğine bakıyordu, ön/arka'yı hiç yansıtmıyordu. Artık ÜÇ kameradan (ön :5000, silah :5001, arka :5002) KAÇININ gerçekten kare gönderdiği ayrı ayrı izleniyor |
| **Eşik** | Her kamera için ayrı: 1.5 saniye içinde o portdan kare gelmiş olmalı |
| **Veri kaynağı** | `video_ekrana_bas()` her karede anahtar 1/2/3'e göre `son_silah_frame_zamani`/`son_on_frame_zamani`/`son_arka_frame_zamani`'dan ilgilisini günceller → `_kamera_heartbeat_kontrol()` (500ms zamanlayıcı) üçünü sayıp `kamera_arayuz_guncelle(bagli_sayisi)`'i çağırır |
| **Arayüz çizimi** | `kamera_arayuz_guncelle()` — 0 bağlıysa kırmızı "BAĞLANTI KOPTU" (eski/tanıdık metin korundu), 1-2 bağlıysa turuncu "X/3 BAĞLI", 3/3 bağlıysa yeşil "3/3 BAĞLI" |

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
| **Hız** | `/odom` (`twist.linear.x`) → `telemetri_sistemi.py:178` `odom_cb()` → `hiz_sinyali` → `main.py:187` doğrudan `label_hizYazi.setText`. **2026-09-01 düzeltmesi**: eskiden veri kesilince (araç Jetson kapandı/bağlantı gitti) ekranda EN SON gelen değer sonsuza kadar asılı kalıyordu - kullanıcı isteğiyle diğer heartbeat'lerle (lidar/imu/gps) AYNI desen eklendi: `surekli_yayin_dongusu` (0.1sn) `son_hiz_zamani`'nın 1sn'den eski olup olmadığını kontrol eder, bayatladığı AN (bir kez) `hiz_sinyali`'ni zorla `"0.0 m/s"` ile yayınlar - canlı doğrulandı (araç Jetson şu an tamamen erişilemez durumdayken test edildi) |
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

### Joystick butonu (Ayarlar sayfası) — 2026-09-04'ten sonra: fiziksel kontrol paneli aç/kapat

| | |
|---|---|
| **Ne yapar** | Yer istasyonu fiziksel kontrol panelini (`KontrolPaneliThread`) **yazılımsal** aç/kapatır. Panel donanımı açılışta zaten otomatik başlar (`_kontrol_paneli_baslat()`); bu buton kapatınca thread durur ve `surus_kaynagi` KLAVYE'ye döner (klavye otomatik açılır) |
| **Konum** | `main.py` `ayar_joystick_tetikle()`, `_kontrol_paneli_baslat()`, `_panel_baglanti_degisti()` |
| **Kalibrasyon** | `kontrol_paneli_sistemi.py` `_EksenDurumu` — `surus_joystick_sistemi.py`'den birebir taşındı, dört eksen (2 joystick) için ayrı ayrı. Bağlantıda ilk 0.5sn ölçüm + bağımsız canlı oto-merkezleme + kalibrasyondan bağımsız son güvenlik ağı. Bkz. §7.5 |
| **Eksen/yön ayarı** | Ayarlar → **PANEL TESTİ** dialogu ile ham kanallar canlı izlenir, `kontrol_paneli_sistemi.py` başındaki `SOL_JOY_X_PIN` / `*_TERS_*` sabitleri ayarlanır |

### Ekran klavyesi butonu (sağ üst köşe, yüzen)

| | |
|---|---|
| **Ne yapar** | onboard ekran klavyesini açar/kapatır; klavye açılırken arayüzü geçici olarak fullscreen'den çıkarır ki klavye üstte kalabilsin |
| **Konum** | `main.py` `_ekran_klavyesi_butonu_ekle()` / `ekran_klavyesi_ac_kapat()` / `_onboard_dbus()` |
| **Detay** | bkz. §5.6 — çoklu tuş (Ctrl+Alt+T) için onboard modifier kilitleme |

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
- **TEK OTURUM GARANTİSİ (2026-09-01, KRİTİK, canlı bulundu) —** yukarıdaki
  `tmux new-session -A` yaklaşımının bir açığı vardı: arayüz sert kapanınca
  (SIGKILL, çökme, ya da bu terminal widget'ının SIGTERM'i normal Qt
  closeEvent akışı DIŞINDA alması - `os.setsid` ile ayrı process group'ta
  olduğu için ana pencereye giden sinyali hiç almaması) yerel `ssh -tt`
  süreci YETİM kalıp tmux'a "attached client" olarak SONSUZA KADAR takılı
  kalabiliyordu - `new-session -A` sadece "yoksa oluştur, varsa bağlan"
  yapar, ESKİ istemcileri DETACH ETMEZ. Canlı `who`/`tmux list-clients` ile
  **11 birikmiş istemci** bulundu (kullanıcı bildirdi: "araca sadece tek
  kullanıcı ile gir"). **Düzeltme**: `terminal_widget.py`'deki bağlantı
  komutu artık `tmux has-session` ile kontrol edip, oturum zaten varsa
  `tmux attach-session -d` (`-d`: KENDİSİNDEN ÖNCEKİ TÜM istemcileri
  detach eder) kullanıyor - yoksa `tmux new-session` ile taze oluşturuyor.
  Artık kaç tane yetim süreç birikirse birikşin, HER ZAMAN sadece EN SON
  bağlanan istemci aktif kalıyor. **NOT**: ilk denemede `\;` (tmux'un
  KENDİ argüman-zincirleme sözdizimi) ile bash `&&`/`||` mantığı
  karıştırılınca ssh süreci anında sessizce çöktü (canlı bulundu) - düz
  `;` (bash komut ayracı) ile iki AYRI komut olarak düzeltildi. Canlı
  doğrulandı: art arda 2 kez yeniden başlatmada da tam olarak TEK istemci
  kaldı (`tmux list-clients` her seferinde 1 satır).
- **Birden fazla terminal (düzeltildi, 2026-09-01)** — ilk tasarımda sol
  menüye ayrı bir "+" butonu ve terminaller için AYRI bir yüzen pencere
  vardı; kullanıcı isteğiyle değiştirildi: artık "+" butonu SOL MENÜDE
  DEĞİL, doğrudan terminalin KENDİ küçük araç çubuğunda (sağ üst köşe,
  "+ Yeni Terminal") — ve her tıklama YENİ bir pencere AÇMAK yerine Ana
  Ekran'daki terminal alanına, mevcut `QSplitter`'a YAN YANA yeni bir pane
  ekliyor (termux/tmux benzeri, hepsi aynı anda tek ekranda görülüyor,
  sürüklenerek yeniden boyutlandırılabiliyor). Her pane yine benzersiz
  isimli AYRI bir tmux oturumuna bağlanır, ana terminalden bağımsız çalışır.
- **tmux renk kaybı (düzeltildi, 2026-09-01)** — tmux'a geçince (bkz.
  yukarıdaki kalıcılık maddesi) terminal renkleri (ls, git, prompt vb.)
  kayboldu (canlı bildirildi) - kök neden: tmux, İÇİNDEKİ kabuğa kendi
  "default-terminal" ayarını (varsayılan "screen" - sınırlı renk) verir,
  dışarıdan miras gelen `TERM=xterm-256color`'ı yok sayar. Düzeltme:
  bağlanırken `tmux set-option -g default-terminal tmux-256color` da
  ayarlanıyor (araç Jetson'da bu terminfo mevcut, `infocmp` ile doğrulandı;
  canlı test: `tput colors` artık 256 dönüyor).
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
- **"Ubiquiti bağlayınca arayüz açılmıyor" (KRİTİK, kök nedeni bulundu,
  v2'de düzeltildi) —** canlı teşhis: bu makinede WiFi (`wlP1p1s0`,
  10.40.64.44) VE Ubiquiti (`enP8p1s0`, 192.168.1.20) + docker0/l4tbr0/
  usb0/usb1/can0 gibi ilgisiz sanal arabirimler AYNI ANDA aktifken, ROS2'nin
  varsayılan DDS'i (Fast-DDS) discovery/gönderim için TÜM arabirimleri
  kullanmaya çalışıyor - bu ANA GUI THREAD'İNİ KERNEL SEVİYESİNDE bloke
  ediyordu (`/proc/PID/wchan`: `sock_alloc_send_pskb`'de kilitli kalıyordu -
  pencere hiç açılmıyordu/donuyordu).
  - **v1 düzeltme (YANLIŞ, geri alındı)**: SADECE WiFi arabirimini beyaz
    listeye alan `fastdds_wifi_only.xml` - pencereyi açtı AMA yeni bir
    soruna yol açtı: araç SADECE Ubiquiti'den erişilebilir olduğunda
    (WiFi yolu "Destination Host Unreachable" - canlı gözlendi) ROS2
    verisi TAMAMEN kesildi ("diğer jetson açık ama veri gelmiyor", canlı
    bildirildi) - çalışan tek gerçek yolu yanlışlıkla dışarıda bırakmıştık.
  - **v2 düzeltme (doğru, canlı doğrulandı)**: `fastdds_gercek_arabirimler.xml`
    artık HER İKİ gerçek arabirimi de (WiFi + Ubiquiti + localhost) beyaz
    listede tutuyor, SADECE ilgisiz sanal arabirimleri (docker0 vb.)
    dışarıda bırakıyor - hangisi çalışıyorsa ROS2 onu kullanabiliyor.
    `main.py` bunu `rclpy` import edilmeden ÖNCE (dosyanın en tepesinde)
    `FASTRTPS_DEFAULT_PROFILES_FILE` ortam değişkeniyle otomatik devreye
    sokuyor - elle bir şey yapmaya gerek yok. Canlı doğrulandı: Ubiquiti
    TEK erişilebilir yolken hem pencere anında açıldı HEM DE gerçek veri
    aktı (`ros2 node list`'te araç VE arayüz düğümleri birlikte görüldü,
    Taktik Radar'da canlı LiDAR nokta bulutu + IMU suni ufuk gerçek
    değerlerle güncellendi).
  - **NOT**: WiFi/Ubiquiti IP'leri değişirse (DHCP) XML'deki adresler de
    güncellenmeli (DHCP rezervasyonu/statik IP bu sorunu kalıcı çözer).
- **"ssh ile karşının portu/IP'si yanlış" (Ubiquiti bağlıyken)** — yukarıdaki
  sorunla bağlantılı canlı bulgu: Ubiquiti bağlıyken araç bazen SADECE
  Ubiquiti ağından (192.168.1.22) erişilebilir oluyor, WiFi IP'sinden
  (10.40.64.43) değil ("No route to host" canlı gözlendi) - ama
  `terminal_widget.py` her zaman sabit WiFi IP'sini kullanıyordu.
  **Düzeltme**: `terminal_widget.py`'ye `_hedef_host_belirle()` eklendi -
  `telemetri_sistemi.py`'nin WIFI/UBIQUITI göstergesindeki AYNI mantık
  (önce Ubiquiti'yi dener, o an ulaşılamıyorsa WiFi'ye düşer), HER
  (yeniden) bağlantıda TAZE karar verir. Canlı doğrulandı: WiFi'den
  ulaşılamazken Ubiquiti IP'sine otomatik geçip bağlandı.
- **HENÜZ YAPILMADI / araştırma gerektiriyor**: "araç çizilen mavi rotanın
  ÜZERİNDEN tam takip etmiyor" şikayeti - bu bir Nav2 kontrolcü ayarı/canlı
  tünning konusu, kod incelemesiyle kesin bir hata bulunamadı; canlı testle
  birlikte araştırılmalı.
- **"RTK verisi FIXED'e geçmiyor" (kök nedeni bulundu, düzeltildi, canlı
  doğrulandı) —** kullanıcı şikayeti üzerine canlı teşhis: NTRIP/RTK
  köprüsünü çalıştıran `NtripRtkThread` (`ntrip_rtk_sistemi.py`), ana
  program (`main.py`) 19+ dakikadır sorunsuz çalışırken, HİÇBİR hata/log
  BASMADAN sessizce ölmüştü - `ros2 node list`'te `tufan_ntrip_rtk_koprusu`
  düğümü kaybolmuştu, RTCM düzeltme yayını tamamen durmuştu
  (`ros2 topic hz /mavros/gps_rtk/send_rtcm` → "does not appear to be
  published yet"). Bu, GPS'in RTK Float'tan (`fix_type:5`) DGPS'e
  (`fix_type:4`) gerilemesiyle birebir örtüşüyordu - yani "fixe girmiyor"
  şikayetinin gerçek nedeni hız/throughput değil, düzeltme akışının SESSİZCE
  KESİLMİŞ olmasıydı. Kök neden: `run()` metodunun İÇ try/except'i sadece
  tek bir bağlantı denemesini sarıyordu; `_ros_baglanti_kur()` ya da
  `rclpy.spin_once()` içinde oluşan bir istisna bu iç bloğun DIŞINA taşıp
  thread'i sessizce sonlandırıyordu (QThread'de yakalanmayan istisna hiçbir
  yere loglanmaz). **Düzeltme (2 parça)**:
  1. `ntrip_rtk_sistemi.py`: eski `run()` gövdesi `_run_ic()`'e taşındı; yeni
     `run()` TÜM gövdeyi tek bir try/except ile sarıyor, herhangi bir
     beklenmeyen istisnada `log_sinyali` üzerinden
     `"🛑 NTRIP thread'i BEKLENMEYEN bir hatayla durdu: {e!r}"` basıyor (artık
     sessiz ölüm imkansız - en azından loglanıyor).
  2. `main.py`: `_ntrip_rtk_baslat()`'e 30 saniyede bir kontrol eden bir
     `QTimer` bekçisi (`_ntrip_bekci_zamanlayici` → `_ntrip_bekci_kontrol()`)
     eklendi - `goal_manager_watchdog.py`'deki aynı desen - thread
     `isRunning()` false dönerse otomatik yeni bir `NtripRtkThread`
     oluşturup başlatıyor ve log'a
     `"⚠️ NTRIP/RTK thread'i durmuş bulundu - otomatik olarak yeniden
     başlatılıyor."` yazıyor.
  **Canlı doğrulama**: düzeltmeyle yeniden başlatılan süreçte
  `tufan_ntrip_rtk_koprusu` düğümü tekrar `ros2 node list`'te göründü, RTCM
  yayını `ros2 topic hz` ile ~1-2 Hz akarken görüldü, ve GPS fix_type kısa
  sürede **6 (RTK FIXED)**'e ulaştı, `h_acc: 20mm` (canlı `ros2 topic echo
  /mavros/gpsstatus/gps1/raw` ile doğrulandı). Not: RTK Fixed'e geçiş
  sadece düzeltme akışının SAĞLIKLI/KESİNTİSİZ olmasına değil, ayrıca
  belirsizlik çözümü (ambiguity resolution) için sürdürülen bir SÜREYE de
  ihtiyaç duyar - uydu geometrisi/multipath gibi çevresel etkenler de rol
  oynar; bu tamamen yazılımla kontrol edilebilir bir şey değildir.
- **"Manuele basınca / klavyeyi aktif edince hâlâ atıyor" (2. kez canlı
  bildirildi, kesin çözüldü) —** `ayar_klavye_tetikle()`'deki sayfa-korumalı
  otomatik yönlendirme (`currentIndex() not in (0,1,2)` şartı) kod
  incelemesinde doğru görünüyordu (Taktik Radar `page_navigasyon`=index 2,
  korunan kümenin içinde) ama kullanıcı sorunun DEVAM ettiğini bildirdi.
  Kesin/garantili çözüm için tartışmaya yer bırakmayacak şekilde otomatik
  sayfa DEĞİŞTİRME tamamen kaldırıldı (`main.py::ayar_klavye_tetikle`) -
  artık klavye HANGİ sayfadan aktif edilirse edilsin sayfa asla
  değişmiyor. Bedel: Ayarlar sayfasındayken (index 4) klavyeyi aktif edip
  hemen WASD'a basmak hâlâ tepki vermez (`keyPressEvent` kasıtlı olarak
  sadece [0,1,2]'de tuş kabul ediyor) - kullanıcı Ana/Kamera/Navigasyon
  sayfalarından birine kendisi geçmeli.
- **Taktik Radar'da hâlâ "kayma" (v5 düzeltme, kök nedeni bulundu) —**
  canlı teşhis: araç ikonu + canlı LiDAR bulutunun döndüğü açı HAM
  `/imu/data` kuaterniyonundan geliyordu (`HaritaYoneticisi.imu_guncelle`);
  `/odom` (konum kaynağı) canlı `ros2 topic echo` ile bit-bit SABİT
  ölçüldüğü halde `/imu/data`'nın kendisi (aynı anda, aynı yöntemle) o an
  yine sabit çıktı - yani sorunun kaynağı HAM IMU okumasının ara sıra
  taşıdığı küçük gürültü (MEMS jiroskop/manyetometre, titreşim/EMI
  kaynaklı) - RViz'in araç modeli muhtemelen `/odom`'un füzyonlu/filtrelenmiş
  TF zincirinden beslendiği için bu gürültüyü göstermiyor. 15m menzilde
  0.3°'lik ham bir sıçrama bile en uzak LiDAR noktalarında ~8cm'lik görünür
  bir titremeye dönüşüyordu. **Düzeltme**: `TaktikRadarEkrani` içine SADECE
  ÇİZİM için ayrı bir `self.yaw_gorsel` eklendi - `guncelle_veri()`'de açı
  sarmasına (179°↔-179°) karşı korumalı üstel hareketli ortalama (EMA,
  α=0.3) ile yumuşatılıyor, `paintEvent`'teki `ressam.rotate()` artık ham
  `self.yaw` yerine bunu kullanıyor. Navigasyon matematiği (costmap dönüşü,
  hedef tıklama) buna DOKUNMADI - onlar zaten ayrı/ham
  `Ros2GcsMotoru.latest_yaw_rad`'ı kullanıyor, sadece görsel titreme
  gideriliyor. İzole matematik testiyle doğrulandı: küçük gürültü
  bastırılıyor, açı sarması doğru (kısa yoldan) işleniyor, gerçek 90°'lik
  bir dönüş ~150ms içinde (%97) yakalanıyor - fark edilir bir gecikme
  yaratmıyor. Canlı görsel doğrulama (gerçek sürüş sırasında titremenin
  gerçekten kaybolduğu) kullanıcıdan bekleniyor.
- **"Arayüz açılmıyor" (2026-09-01, kök nedeni bulundu, düzeltildi) —**
  kullanıcı "araç Jetson kapalı olduğu için mi" diye sordu; canlı teşhis
  bunun sebep OLMADIĞINI kanıtladı (araç Jetson'a hem WiFi hem Ubiquiti
  üzerinden %100 paket kaybı varken bile arayüz eksiksiz açıldı - harita,
  kamera, ROS2 motoru, NTRIP hepsi başladı). Gerçek kök neden: kullanıcı
  `python3 main.py`'yi VS Code'un entegre terminalinden DOĞRUDAN
  çalıştırdığında ROS2 ortamı (`setup.bash`) source EDİLMEMİŞ oluyordu -
  `telemetri_sistemi.py`'nin başındaki `try/except ImportError` bunu zaten
  öngörüp yakalıyordu (`rclpy=None` vb.) ama `TelemetriThread.__init__`'in
  ROS2-YOK erken dönüş bloğu, ROS'a hiç ihtiyacı olmayan SADE Python durum
  değişkenlerini (`pwm_ust_sinir`, `arac_modu` vb.) ve publisher
  attribute'larını (`yon_pid_pub`, `silah_ates_pub` vb. - `None` bile
  DEĞİL, hiç TANIMLI değildi) hiç oluşturmuyordu; `main.py`'nin
  `_buton_baglantilarini_kur()`/`_yon_pid_butonu_ekle()` gibi başlangıç
  kodu bunları KOŞULSUZ okuyunca `AttributeError` ile TÜM uygulama
  çöküyordu (canlı, kullanıcının kendi terminal çıktısıyla doğrulandı:
  `AttributeError: 'TelemetriThread' object has no attribute
  'pwm_ust_sinir'`, sonra `'yon_pid_pub'`). **Düzeltme (2 parça)**:
  1. `telemetri_sistemi.py`: ROS2-YOK erken dönüş bloğuna, main.py'nin
     ROS2 durumuna bakmadan okuduğu TÜM sade durum değişkenleri (`arac_modu`,
     `surus_kaynagi`, PWM sınırları/kademeleri, `manuel_hazir`) VE tüm
     publisher attribute'ları (`silah_modu_pub`, `turret_cmd_pub`,
     `silah_ates_pub`, `yon_pid_pub`, `farlar_pub` - hepsi `None`) eklendi.
     Artık ROS2 hiç bulunamasa bile arayüz TAM AÇILIYOR, sadece
     telemetri/sürüş/harita verisi devre dışı kalıyor (çökme YOK) - canlı
     `env -i` ile gerçek "ROS2 yok" senaryosu simüle edilerek doğrulandı.
  2. `calistir.sh` (yeni): doğru ortamı garanti eden basit bir başlatıcı
     eklendi (`source /opt/ros/humble/setup.bash` + `~/ros2_humble` +
     `python3 main.py`) - arayüzü başlatmak için artık bunun kullanılması
     öneriliyor, ROS2'nin gerçekten aktif olduğu (telemetri/sürüş/RTK
     çalışır) bir açılış garantiler.
  - **NOT (henüz düzeltilmedi, ayrı/tekrarlanan bir bulgu)**: güç
    butonuyla kapatma sırasında iki ayrı canlı testte de
    `terminate called without an active exception` ile bir çökme
    gözlemlendi (uygulama düzgün açılıp çalıştıktan SONRA, kapanışta) -
    kök nedeni AYRI bir araştırma gerektiriyor, bu oturumun konusu
    olan "açılmıyor" sorunuyla karışmasın diye burada sadece not
    düşülüyor.
- **NTRIP/RTK "zombi thread" kör noktası (2026-09-01, KRİTİK, canlı
  bulundu) —** kullanıcı tekrar sordu: "RTK verisini geç mi
  gönderiyorsun yoksa gönderen node mu çöküyor". Canlı doğrulandı: evet,
  düğüm gerçekten çöküyor - `tufan_ntrip_rtk_koprusu` `ros2 node list`'ten
  kayboluyordu AMA bu sefer bir öncekinden (bkz. yukarıdaki "RTK Fixed'e
  geçmiyor" maddesi) FARKLI bir şekilde: `run()`'daki try/except HİÇBİR
  hata loglamadı VE mevcut bekçi (`_ntrip_bekci_kontrol`) de HİÇ yeniden
  başlatma denemesi yapmadı - yani thread'in kendisi hâlâ `isRunning()=True`
  dönüyordu (kod incelemesinde açık bir `destroy_node()`/`rclpy.shutdown()`
  çağrısı da bulunamadı). Bu, önceki düzeltmenin gözden kaçırdığı bir KÖR
  NOKTA: `isRunning()` sadece Python/Qt thread NESNESİNİN hayatta olup
  olmadığını gösterir - içindeki ROS düğümü "zombi" hâle gelip (DDS
  grafiğinden kaybolup) thread teknik olarak "çalışıyor" görünmeye devam
  edebilir. **Düzeltme**: `ntrip_rtk_sistemi.py`'ye diğer heartbeat'lerle
  (lidar/imu/gps/hız - bkz. §7, §Veri Kanalları) AYNI desende bir NABIZ
  eklendi (`son_nabiz_zamani`, hem dış bağlanma döngüsünde hem iç
  spin/veri döngüsünde HER turda güncellenir) - `_ntrip_bekci_kontrol`
  artık SADECE `isRunning()` değil, bu nabzın 90 saniyeden fazla
  bayatlayıp bayatlamadığını da kontrol ediyor; ikisinden biri olumsuzsa
  (thread bitmiş VEYA nabız kesilmiş/"zombi") yeniden başlatma tetikleniyor
  - zombi durumda eski thread'in `_calisiyor` bayrağı GUI'yi
  DONDURMADAN (`wait()` ÇAĞIRMADAN) kapatmaya işaretlenip yeni bir thread
  hemen devralıyor. Canlı doğrulandı: taze başlatmada hem NTRIP düğümü
  hem harita'nın GPSRAW aboneliği (aşağıdaki madde) sorunsuz kuruldu, RTK
  Float ±0.23m ile akıyordu.
- **Uydu Haritası'nda "GPS: veri yok" (aralıklı/geçici, canlı bulundu) —**
  kullanıcı bildirdi: "gnss araçta çalışıyor olmasına rağmen veri yok
  yazıyor". Kök neden NavSatFix (harita/uydu görüntüsü konumlama) DEĞİL -
  o hep çalışıyordu (harita her zaman doğru gösteriliyordu). Sorun SADECE
  ayrı GPSRAW (fix tipi/hata) aboneliğindeydi (`tufan_gcs_master_node`'un
  `gpsraw_sub`'ı) - bazı canlı kontrollerde bu abonelik DDS grafiğinde HİÇ
  yoktu (`ros2 node info`'da görünmüyordu) BAŞKA kontrollerde (taze
  başlatmalarda) TAMAMEN doğru kuruluyordu - kod incelemesi ve izole
  içe aktarma testleri (`import harita_sistemi; print(harita_sistemi.
  GPSRAW)`) HER ZAMAN başarılı döndü, yani `mavros_msgs` içe aktarma
  hatası DEĞİL. En olası açıklama: bu oturumdaki ÇOK SIK art arda
  yeniden başlatmaların (aynı düğüm adıyla saniyeler içinde defalarca
  başlat/durdur) Fast-DDS keşif önbelleğinde geçici bir tutarsızlık
  yaratması - kesin/tekil bir kod hatası BULUNAMADI, taze/normal aralıklı
  başlatmalarda sorun hiç tekrarlanmadı. **İzlenecek**: eğer bu sorun
  NORMAL kullanımda (sık art arda yeniden başlatma OLMADAN) tekrar
  görülürse, bu geçici-DDS-önbelleği açıklaması yanlış demektir ve daha
  derin bir araştırma gerekir.

## 8.2 Canlı Test Sonrası Düzeltmeler (2026-09-04/05)

- **KRİTİK: `ROS_DOMAIN_ID` ayarlanmamıştı ("arayüz sürekli çöküyor"/
  açılmıyor şikayetlerinin asıl kök nedeni) —** izole testle KANITLANDI:
  paylaşılan ağda (bu Jetson'ın bağlı olduğu WiFi/Ubiquiti) varsayılan
  domain (0) çok kalabalık DDS keşif trafiğine denk geliyor - saf bir
  `rclpy.init(); Node('test')` bile **53 saniye** sürdü; izole bir domainde
  (77) aynı işlem **0.98 saniye**. `TelemetriThread.__init__`'te bu
  senkron (GUI thread'inde) çağrıldığı için arayüz dakikalarca hiç
  açılmıyor/"çökmüş" gibi görünüyordu. Düzeltme: hem arayüz (`calistir.sh`
  VE `main.py`'nin en başında kod içine gömülü, script'e bağımlı
  kalmasın diye) hem araç Jetson'daki `tufan_mppi.launch.py` **AYNI
  domain'de (77)** çalıştırılıyor - biri diğerinden farklı domainde
  olursa BİRBİRLERİNİ HİÇ GÖREMEZLER (yavaş değil, TAMAMEN kopuk).
- **"Sen çalıştırınca çalışıyor, ben çalıştırınca çalışmıyor" (kullanıcı
  bildirdi) —** kullanıcı `main.py`'yi `calistir.sh` YERİNE doğrudan
  (kendi terminal alışkanlığı) başlatınca ROS2 hiç source edilmemiş bir
  shell'de `import rclpy` tamamen BAŞARISIZ oluyordu (canlı doğrulandı),
  `telemetri_sistemi.py` bunu sessizce devre dışı bırakıyordu
  (`ros2_bagli=False`) - IMU/LIDAR/GPS hep "pasif" kalıyordu. Artık
  `main.py`, rclpy import edilebilir mi diye BAŞKA HİÇBİR ŞEYDEN ÖNCE
  kontrol ediyor; edilemiyorsa doğru ortamı (calistir.sh ile AYNI) source
  edip KENDİSİNİ otomatik yeniden başlatıyor (`TUFAN_ENV_HAZIR` bayrağı
  sonsuz döngüyü önlüyor). Artık başlatma yöntemi (script/IDE/doğrudan
  terminal) fark etmiyor.
- **IMU/LIDAR göstergesi yanlış "PASİF" gösteriyordu —** veri ağda HER
  ZAMAN sağlıklı akıyordu (ayrı ayrı `ros2 topic hz` ile defalarca
  doğrulandı), sorun `telemetri_sistemi.py`'deki tazelik eşiğiydi: sadece
  **1.0 saniye** (GPS'in kullandığı 3.0 saniyeye kıyasla çok sıkı) - bu
  thread'in executor'ı diğer thread'lerle (kamera/terminal) GIL
  paylaşırken ara sıra 1sn'yi hafif aşan kısa duraksamalar yaşayabiliyor,
  veri KESİLMEDEN gösterge yanlışlıkla yanıp sönüyordu. Her ikisi de
  3.0sn'ye çekildi.
- **SSH terminal panelinin GUI thread'i tıkaması (en büyük tekil kasma/
  "çökme" kaynağıydı, GUI takılma izleyicisiyle canlı ölçüldü: bazen
  540ms-40sn arası) —** `pyte` (saf Python VT100 emülatörü) eskiden
  `QSocketNotifier` ile GUI thread'inde, SSH'tan her veri gelişinde
  SENKRON çalışıyordu - araç Jetson'daki `ros2 launch --output=screen`
  sürekli log bastığı için bu panel SÜREKLİ, gerçek bir yük kaynağı.
  Düzeltmeler (`terminal_widget.py`): (1) okuma+ayrıştırma (`feed()`)
  ayrı bir arka plan thread'ine taşındı, GUI thread'e sadece Qt sinyaliyle
  "render'a ihtiyacın var" haberi veriliyor; (2) bu sinyalin bir log
  patlamasında saniyede binlerce kez tetiklenip Qt olay kuyruğunu
  ŞİŞİRMESİ (kuyruğu boşaltmanın kendisi tek başına ~9-40sn sürüyordu)
  önlendi - artık kuyrukta her zaman EN FAZLA BİR bekleyen istek var
  (coalescing bayrağı); (3) render debounce'u 16ms'den (~60Hz)
  100ms'ye (~10Hz) çıkarıldı - terminal metni video değil, kameranın
  önüne daha az sıklıkta geçiyor. Ayrıca **"🔻 Terminali Gizle"** butonu
  eklendi - panel gizliyken render TAMAMEN atlanıyor (SSH oturumu
  KESİLMEZ), kamera/sürüş kritik anlarında kullanıcı bu maliyeti
  isteyerek sıfırlayabiliyor.
- **LiDAR nokta bulutu ve costmap çizimi yavaştı —** `harita_sistemi.py`
  paintEvent'i her LiDAR taramasındaki (~360-720 nokta, hiç örnekleme
  yapılmadan) TÜM noktaları tek tek antialiaslı `drawEllipse` ile
  çiziyordu - GUI takılma izleyicisiyle 1.7-2.7 saniyeye kadar tıkandığı
  ölçüldü. Düzeltme: `scan_callback`'te veri kaynağında 1/3'e seyreltme
  (costmap'teki AYNI "adim" deseni) + hem LiDAR noktaları hem costmap
  dörtgenleri için antialiasing kapatıldı (küçük şekillerde gözle fark
  edilmiyor, yazılımsal çizimde maliyetli) - nav2'nin kendi engelden
  kaçınması bundan ETKİLENMEDİ (bu sadece görsel gösterge).
- **IMU'nun aşırı sık tetiklediği tam ekran yeniden çizim —** `imu_sinyal`
  eskiden ~62Hz'e kadar (IMU'nun kendi hızı ~85-95Hz) Suni Ufuk VE Taktik
  Radar'ın TAM yeniden çizimini tetikliyordu - kamera akışı için
  uygulanan throttle bunu hiç kapsamıyordu. 20Hz'e düşürüldü (insan gözü
  için zaten akıcı); `SuniUfukEkrani.guncelle_veri`'ye de (eskiden hiç
  yoktu) `TaktikRadarEkrani`'ndeki AYNI görünürlük koruması eklendi.
- **Taktik Radar'da her fare/dokunma hareketinde tam ekran yeniden çizim —**
  `mouseMoveEvent`'in salt-hover (sürükleme/pan OLMAYAN) dalı, SADECE
  imleç koordinat yazısını tazelemek için HER hareket olayında
  `self.update()` çağırıyordu - dokunmatik ekranda parmak sürekli hareket
  halinde olduğundan saniyede yüzlerce gereksiz tam-ekran çizim demekti.
  ~30Hz'e throttle edildi.
- **Kamera akışı: UDP alım birikintisi + sınırsız çizim hızı + gereksiz
  dönüşümler —** (1) `kamera_sistemi.py`'de soket her turda sadece BİR
  paket okuyordu, kod kameranın hızına yetişemezse eski kareler birikip
  gecikme SÜREKLİ ARTIYORDU - artık soket her turda TAMAMEN boşaltılıp
  sadece en yeni kare kullanılıyor. (2) `main.py::video_ekrana_bas`
  gelen HER kareyi (kameranın kendi FPS'i kadar) işliyordu - ~15Hz'e
  throttle edildi, küçük önizleme kutuları ise ~1Hz'e (sadece
  büyütülen/seçili kamera tam hızda akıyor). (3) `resmi_yuvarla`:
  `QPixmap.fromImage()` + `drawPixmap` yerine `QPainter.drawImage()`
  DOĞRUDAN kullanılıyor (ara dönüşüm adımı atlanıyor, Qt forum
  kaynaklarında belgeli bir performans sorunu); eskiden eklenen manuel
  ön-küçültme adımı KALDIRILDI (kullanıcı isteği + gereksizdi: tüm kamera
  `QLabel`'ları zaten `setScaledContents(True)` ile kuruluydu, Qt kendi
  native ölçeklemesini zaten yapıyordu).
- **Çökme sertleştirme —** (1) global `sys.excepthook`: yakalanmamış HER
  Python hatası artık `crash_log.txt`'ye yazılıp log kutusuna basılıyor
  ama uygulama KAPANMIYOR (eskiden PyQt5'in varsayılan davranışı
  sessizce çökmekti) - bu şekilde gerçek bir hata (`KameraThread`'de
  eksik `kayit_durumu_degistir` metodu - VİDEO KAYDI özelliği HİÇ
  implemente edilmemiş, ayrı bir eksiklik) yakalanıp raporlandı. (2)
  `main.py`'ye düzgün bir `closeEvent` eklendi - pencere kapanmadan önce
  TÜM worker thread'ler (kamera/telemetri/ntrip/kontrol paneli) BLOKE
  EDEREK (gerçekten bitene kadar bekleyerek) durduruluyor - eskiden
  hiçbiri düzgün durdurulmuyordu, `KameraThread.stop()` sadece bir bayrak
  set edip `wait()` çağırmıyordu - pencere kapanırken bu thread'ler hâlâ
  çalışırken Qt nesneleri yok ediliyordu, bu da "QThread: Destroyed while
  thread is still running" FATAL hatasıyla sessizce çökmeye yol
  açıyordu (canlı log'da birebir bu iz bulundu).
- **GPS'e özel bekçi eklendi —** telemetri bekçisi (zombi thread testi)
  SADECE executor döngüsünün kendisinin dönüp dönmediğini yakalıyordu -
  GPS aboneliğinin DDS eşleşmesi tek başına bozulup diğer her şey normal
  çalışmaya devam edebiliyordu (zombi testinden GEÇER ama GPS yine de
  "veri yok" gösterirdi). Artık GPS özelinde de ayrı bir tazelik kontrolü
  var - 60sn'den uzun süredir GPS verisi gelmemişse (diğer her şey
  sağlıklıysa) bağlantı kendiliğinden, sessizce yeniden kuruluyor.
- **Stres testi (izole, otomatik) —** 3 kamera portuna eş zamanlı
  ~45fps/port (doğal hızın ~3 katı) sahte JPEG akışı 90 saniye boyunca 2
  kez gönderildi - uygulama HİÇBİR ZAMAN çökmedi, sadece (yukarıdaki
  terminal sorunuyla birleşince) geçici yavaşlamalar yaşadı.
- **Lazer (silah ateşleme) "açılıp kapanmıyor" şikayeti —** kod incelemesi
  sonucu bir hata BULUNAMADI (`_lazer_gonder`/`_lazer_yazici_thread`
  tasarımı doğru, 0.5sn güvenlik zaman aşımı var); asıl sebep araç
  Jetson'un kapatılıp açılmasının ardından ROS2 yazılım yığınının
  (turret_node.py dahil) otomatik başlamamış olmasıydı - yeniden
  başlatılınca topic seviyesinde test edilip kabul edildiği doğrulandı.
- **Dokunmatik ekran desteği (Taktik Radar) —** Qt'nin `WA_AcceptTouchEvents`
  özelliği yalnızca kendisi ayarlayan widget'ta çalışıyor, çocuk
  widget'lar (sağ panel butonları/liste) dokunmayı hiç almıyordu - önce
  panel alanı üzerinde başlayan dokunmalar hiç ele alınmayıp Qt'nin
  normal sentez-fare mekanizmasına bırakılarak düzeltildi. Ayrıca: tek
  parmak dokunuş = nokta ekle, sürükleme = kaydırma (eski sağ-tık
  formülüyle AYNI), iki parmak pinch = yakınlaştırma (eski fare
  tekerleği formülüyle AYNI, ikinci parmak Qt'de ayrı bir `TouchBegin`
  DEĞİL mevcut `TouchUpdate` içinde geldiği için referans mesafe orada
  da başlatılıyor).
- **Yer istasyonu fiziksel kontrol paneli kalibrasyon arayüzü —** eskiden
  sabit kod sabitleri olan eksen pin ataması/ters çevirme ayarları artık
  JSON dosyasında kalıcı (`kontrol_paneli_ayarlari.json`) ve PANEL TESTİ
  diyalogundan canlı veriye bakarak değiştirilip anında (yeniden
  başlatmaya gerek olmadan, arka planda sessizce yeniden bağlanarak)
  uygulanabiliyor - bkz. `kontrol_paneli_sistemi.py::ayarlari_yukle/
  ayarlari_kaydet` ve `KontrolPaneliThread.ayarlari_yeniden_yukle`.
- **Etap göstergesi artık Stop/Bitiş tabelalarını da metne çeviriyor —**
  `/guncel_etap` eskiden sadece 1-11 sayıları için yayınlanıyordu; araç
  tarafı artık Stop doğrulanınca -1, EndOfEleven görülünce -2 gönderiyor
  (Int32 tipi bozulmadı), arayüz bunları "STOP"/"BİTİŞ" metnine çeviriyor.

## 8.3 Gerçek SIGSEGV Kökeni ve Diğer Düzeltmeler (2026-09-05)

- **GERÇEK ÇÖKME (SIGSEGV, çıkış kodu 139) bulundu ve düzeltildi —**
  `crash_rapor_son.txt` mekanizması (bkz. aşağıda) ilk kez GERÇEK bir
  native çökme yakaladı (önceki tüm "çöküyor" raporları ya Python
  istisnası, ya elle force-quit, ya da ROS_DOMAIN_ID kaynaklı donma
  idi). Log'da çökmeden hemen önce art arda
  `QBackingStore::endPaint() called with active painter; did you forget
  to destroy it or call QPainter::end() on it?` uyarısı görüldü.
  `faulthandler` (aşağıda açıklanıyor) tam thread dökümü aldı; ana/GUI
  thread'in çökme anındaki tek çerçevesi `app.exec_()` içindeydi (yani
  çökme Qt'nin C++ olay döngüsü içinde gerçekleşti - Python tarafında
  istisna YOKTU). Tüm `QPainter(` çağrıları grep'lenerek kök neden
  bulundu: `uydu_harita_widget.py::paintEvent` içinde HİÇBİR
  `ressam.end()` çağrısı yoktu - GNSS henüz yokken erken `return`
  vardı VE uzun taş döşeme (tile)/disk G/Ç/renk analizi gövdesi
  istisna atabilirdi; her iki durumda da `QPainter` "aktif" kalıp
  Qt'nin arka saklama alanını (backing store) bozuyor, bu da er ya da
  geç segfault'a yol açıyordu. Düzeltme: `paintEvent` artık ince bir
  sarmalayıcı - `ressam = QPainter(self); try: ... (erken dönüş veya)
  self._paintEvent_govde(ressam) ... finally: ressam.end()` - orijinal
  gövde `_paintEvent_govde` metoduna taşındı. AYNI risk sınıfı önlem
  amaçlı `harita_sistemi.py`'deki `SuniUfukEkrani.paintEvent` ve
  `TaktikRadarEkrani.paintEvent` için de uygulandı (ikisi de artık
  `_paintEvent_govde` deseniyle try/finally içinde) - özellikle
  `TaktikRadarEkrani` harici LIDAR/costmap/waypoint verisi üzerinde
  döngü kurduğu için riskliydi. Düzeltme sonrası canlı izlemede
  `QBackingStore` uyarısı BİR DAHA GÖRÜLMEDİ.
- **`crash_rapor_son.txt` otomatik çökme raporu (`calistir.sh`) —**
  kullanıcının her seferinde "neden çöktü" diye sormasını önlemek için:
  `calistir.sh` artık `exec` KULLANMIYOR (python3 bittiğinde onu
  gözlemleyen bash süreci hayatta kalıyor), gerçek çıkış kodunu
  `${PIPESTATUS[0]}` ile yakalıyor, kodu yorumluyor (128'den büyükse
  sinyal = kod-128; 137=KILL, 139=SEGV), log'da istekli kapatma
  ("Sistem kapatılıyor") izi olup olmadığına bakıyor, ve
  `/home/tufan-yer/Desktop/tufan/crash_rapor_son.txt` dosyasına
  başlangıç/bitiş zamanı + yorumlanmış sonuç + son 60 log satırını
  yazıyor. Elle `kill -9` ile test edilip doğrulandı; yukarıdaki GERÇEK
  SIGSEGV de bu mekanizma sayesinde ilk denemede doğru teşhis edildi.
  Ayrıca `ulimit -c unlimited` eklendi (gerekirse core dump analizi
  için).
- **`showFullScreen()` sıralaması —** pencere, Qt olay döngüsü
  (`app.exec_()`) başlamadan ÖNCE tam ekran gösterilirse pencere
  yöneticisinin ping/expose mesajlarına cevap veremez; eğer bu sırada
  yavaş senkron bir işlem (örn. eski ROS_DOMAIN_ID=0 sorunu) sürerse
  masaüstü ortamı süreci "yanıt vermiyor" diyip öldürebilir (SIGKILL,
  Python istisnası yok). Önlem olarak `self.showFullScreen()` çağrısı
  `TufanGCS.__init__` içinde artık TEK senkron rclpy-bloklayan adımdan
  (`_telemetri_baglantilari_kur()`) SONRA yapılıyor; diğer worker
  thread'ler (`NtripRtkThread`, `Ros2GcsMotoru`, `KontrolPaneliThread`)
  zaten Node'larını `run()` içinde oluşturduğu için ana thread'i
  bloklamıyor.
- **GPS'in konum tahminine katkısı artık görünür —** araç tarafı
  `konum_birlestirici.py` her `_tick()`'te yeni bir `/gps_katki_durumu`
  (std_msgs/String) mesajı yayınlıyor (yön hizalaması bekleniyor / çapa
  bekleniyor / hedef yok / veri bayat / zaten hizalı / aktif düzeltme +
  hız/kalite/kalan-hata gibi Türkçe açıklayıcı durumlar). Arayüz bu
  konuya abone olup (`Ros2GcsMotoru.gps_katki_sinyali`) Taktik Radar
  ekranında GNSS satırının hemen altına yazıyor (yeşil: aktif katkı
  yapıyor, sarı: bekliyor/pasif).
- **RTK Fixed'te daha hızlı cm-hassasiyeti yakınsaması —**
  `konum_birlestirici.py`'de `gps_heading_samples_needed` 30'dan 15'e
  düşürüldü (yön hizalaması ~2x daha hızlı kilitleniyor); yeni
  `gps_fixed_fix_type` (varsayılan 6 = RTK Fixed) ve
  `gps_correction_max_speed_mps_fixed` (varsayılan 0.3 m/s) parametreleri
  eklendi - RTK Fixed alındığında düzeltme hızı otomatik olarak eski
  temkinli `gps_correction_max_speed_mps` (0.05 m/s, RTK Float için hâlâ
  geçerli) yerine bu çok daha hızlı değere geçiyor.
- **Araç tarafı değişiklikleri `colcon build --packages-select
  tufan_v2_ws` ile derlenip `tufan_mppi.launch.py` yeniden başlatılarak
  devreye alındı** (kullanıcının açık isteği üzerine) - saf Python
  ament_python paketi olduğu için derleme ~0.25sn sürüyor (sadece dosya
  kaydı/kopyalama), `diff` ile src/install ağaçlarının eşleştiği
  doğrulandı.
- **PANEL TESTİ: "🎯 Joystick Merkezini Sıfırla" butonu eklendi
  (2026-09-05, canlı bildirildi: "joystickleri ellemiyorum ama sağa
  dönüyor") —** güç-açılışı kalibrasyonundaki merkez gerçek fiziksel
  ortadan (>±90 ADC birimi) uzaksa, canlı oto-merkezleme
  (`MAKS_MERKEZ_KAYMASI` sınırı) bunu kendi başına düzeltemiyordu. Yeni
  buton (main.py `_panel_testi_ac`) `KontrolPaneliThread.merkezi_sifirla()`
  çağırır - "KAYDET VE UYGULA" ile AYNI güvenli reconnect+yeniden-kalibrasyon
  yolunu (Arduino hiç koparılmadan) tetikler, joystickler BIRAKILMIŞ
  varsayımıyla o anki ham ADC değerlerini yeni merkez (sıfır) yapar.
- **KRİTİK: turret_cmd_pub RELIABLE QoS → GUI thread donması (2026-09-05,
  canlı bildirildi: "arayüz dondu, joystick bile çok geç gidiyor") —**
  `crash_rapor_son.txt` + `[STALL]` izleyicisi kanıtladı: `/turret_manuel_cmd`
  yayıncısı (telemetri_sistemi.py) varsayılan RELIABLE QoS ile GUI
  thread'inde SENKRON `publish()` çağrılıyordu (`main.py::_panel_turret_geldi`
  → `joystick_turret_gonder`). Bu ağın RELIABLE ack/retransmit mekanizması
  `/palet_hizlari`'nda daha önce tespit edilenle AYNI şekilde tıkanıyor -
  bir örnekte `publish()` **15.87 saniye BLOKE** oldu. Qt tek thread'li
  olduğu için bu TEK slot'un bloke olması, kuyruktaki TÜM diğer sinyalleri
  (joystick SÜRÜŞ güncellemeleri dahil - `_panel_surus_geldi` de aynı GUI
  thread kuyruğunda bekliyordu) aynı süre bekletti - kullanıcının "joystick
  bile çok geç gidiyor" şikayetinin gerçek nedeni buydu. Düzeltme:
  `turret_cmd_pub` `/palet_hizlari` ile AYNI desende BEST_EFFORT/depth=1
  QoS'a alındı; UYUMLULUK İÇİN araç tarafındaki `turret_node.py`'nin
  `turret_manuel_cmd` aboneliği de AYNI ANDA BEST_EFFORT'a alındı (yayıncı
  BEST_EFFORT iken abone RELIABLE beklerse DDS SESSİZCE eşleşmez - mesaj
  hiç ulaşmaz). `ros2 topic info --verbose` ile her iki uçta da
  `Reliability: BEST_EFFORT` olduğu doğrulandı. NOT: `/arac_komut`
  (EMERGENCY_STOP_CMD/DEVAM_CMD dahil), `/silah_ates_manuel`,
  `/yon_pid_aktif`, `/farlar` KASITLI OLARAK RELIABLE bırakıldı - bunlar
  tek-seferlik/nadir güvenlik komutları (sürekli yeniden yayınlanmıyor),
  BEST_EFFORT'a çevirmek garantili teslimi kaybettirebilir; `/surus_modu`
  ve `/silah_modu` da (3 ayrı abonesi olduğu için, hepsi RELIABLE bekliyor)
  bilinçli olarak DOKUNULMADI - ileride benzer bir donma görülürse önce
  bunlar şüpheli listeye alınmalı ama vehicle tarafında 3 dosyanın (
  `kaba_harita_olusturucu.py`, `surus_koprusu.py`, `tabela_etap_yoneticisi.py`,
  `turret_node.py`'nin silah_modu aboneliği) HEPSİNİN QoS'unun BİRLİKTE
  güncellenmesi gerekir.
- **KRİTİK: TaktikRadarEkrani /odom'da 59Hz throttle'sız update() → "RTK
  geç geliyor" yanılsaması (2026-09-05, canlı bildirildi: "RTK verisi
  arayüz kasmalarından dolayı geç gidiyor") —** `guncelle_konum` (her
  `/odom` mesajında, ~59Hz!), `guncelle_veri` (LIDAR+IMU), `guncelle_gnss`,
  `guncelle_costmap`, `guncelle_global_plan`, `guncelle_local_plan` HEPSİ
  koşulsuz (sadece görünürlük kontrolüyle) `self.update()` çağırıyordu -
  `/odom` TEK BAŞINA saniyede 59 kez `paintEvent`'i tetikleyebiliyordu.
  Canlı ölçüldü: `/mavros/gpsstatus/gps1/raw` ağ seviyesinde SAĞLIKLI
  ~10Hz geliyordu - "RTK geç geliyor" izlenimi aslında GUI thread'in bu
  aşırı sık tetiklenme altında EKRANA YANSITMASININ gecikmesiydi
  (turret_cmd_pub'daki QoS sorunuyla AYNI sınıf: tek meşgul GUI thread,
  kuyruktaki TÜM güncellemeleri - GPS/joystick dahil - eşit geciktiriyordu).
  Düzeltme: `terminal_widget.py`'deki AYNI coalescing deseni
  (`_render_istegi_beklemede` bayrağı + debounce timer) `TaktikRadarEkrani`
  için de eklendi (`_render_iste()`/`_render_zamanlayici_calisti()`,
  bkz. `__init__`) - veri her zaman anında güncellenir, ama pahalı
  `self.update()` çağrısı en fazla ~30fps'e (33ms) sınırlandı. Canlı
  ölçüldü: ortalama render aralığı ~50-90ms'den ~30ms'ye düştü, en kötü
  STALL süresi (sessiz/dokunulmadan ölçümde) 1050-1260ms'den 150-570ms'ye
  düştü. NOT: kalan STALL'ların bir kısmı (özellikle 90sn'lik "hiç SSH
  komutu göndermeden izleme" testinde bile devam eden 150-570ms'lik
  duraksamalar) muhtemelen paylaşılan masaüstünün (VS Code + Claude Code
  CLI + genel X11/pencere sistemi yükü) kalıcı bir katkısı - session
  boyunca defalarca kanıtlanan bir örüntü, tamamen sıfırlanamıyor ama
  kod tarafında yapılabilecek iyileştirme büyük ölçüde tüketildi.
- **UyduHaritaWidget'a da render coalescing eklendi (2026-09-05) —**
  `konum_guncelle` (GNSS), `rtk_durum_guncelle`, `imu_guncelle`,
  `lidar_guncelle` de TaktikRadarEkrani'ndeki AYNI koşulsuz
  `self.update()` desenini kullanıyordu (sadece `isVisible()` kontrolü,
  throttle YOK) - Navigasyon sayfasına geçildiğinde, o ana kadar
  BİRİKMİŞ sinyallerin art arda tetiklediği paintEvent zincirinin GUI
  thread'i kilitleme riski buraya da uygulandı. Aynı `_render_iste()`/
  `_render_zamanlayici_calisti()` deseni (en fazla ~30fps) eklendi.
- **KRİTİK: `calistir.sh` artık OTOMATİK KURTARMA (süpervizör döngüsü)
  içeriyor (2026-09-05, kullanıcı: "ara ara force quit yapmak
  gerekiyor... otomatize et her seferinde uğraşmayalım") —** eskiden
  `python3 main.py` TEK SEFER çalışıp script bitiyordu; arayüz donup
  force-quit edildiğinde (ya da gerçek bir SIGSEGV/kill olduğunda)
  kullanıcının/operatörün ELLE `kill`+yeniden-başlatma yapması
  gerekiyordu. Artık `python3 -u main.py` bir `while true` döngüsü
  içinde: çıkış TEMİZ (kod 0 - X butonu/"Sistemi Kapat") DEĞİLSE 2sn
  sonra OTOMATİK yeniden başlatılır; TEMİZ çıkışta döngü DURUR
  (kullanıcı bilerek kapatmışsa saygı duyulur, istenmeden sürekli
  yeniden açılmaz). CRASH-LOOP KORUMASI: art arda 5 kez ÇOK HIZLI
  (10sn altı) çökme olursa döngü durur (kalıcı bir hatada sonsuz
  döngüye girip kaynak tüketmesin, elle müdahale gereksin) - izole bir
  testle (sahte hızlı çökmeler) doğrulandı. `crash_rapor_son.txt` artık
  her denemenin numarasını ve çalışma süresini de gösteriyor.
- **`python3 main.py` → `python3 -u main.py` (unbuffered stdout,
  2026-09-05) —** canlı test sırasında keşfedildi: stdout bir pipe'a
  (`tee`) yönlendirildiğinde Python'un varsayılan TAM TAMPONLU modu
  yüzünden `print()`/`log_yaz()` çıktısı GERÇEK ZAMANLI görünmüyordu -
  bir testte "Başlatıldı" satırı buffer'da 100+ saniye beklemiş
  görünüyordu (program aslında birkaç saniyede başlamıştı - `-u`
  eklenince AYNI başlangıç ilk denemede/saniyeler içinde görüldü).
  Daha önemlisi: bir çökme/kill anında henüz FLUSH EDİLMEMİŞ satırlar
  `crash_rapor_son.txt`'nin "son 60 satır"ından EKSİK kalabiliyordu -
  `-u` ile her satır ANINDA yazılır, teşhis güvenilirliği artar.

## 9. Bilinen Sınırlamalar / Gelecek İşler

- **"Sayfa değiştirince donma" (2026-09-05, kullanıcı: "zaten arayüze
  sayfa değiştirince donduruyorduk threadler vardı onu hallet") —**
  bulunan/düzeltilen katkı: TaktikRadarEkrani ve UyduHaritaWidget'ta
  render coalescing (yukarıda). Kamera thread'i incelendi - zaten sayfa
  kontrolü + throttle var, ek bir sorun bulunamadı. ANCAK bu düzeltme
  araç Jetson KAPALIYKEN yapıldı (LIDAR/costmap/kamera verisi akmıyordu) -
  kullanıcının "herşey bağlıyken" tarif ettiği TAM YÜK senaryosu henüz
  canlı doğrulanamadı. Araç tekrar açılıp normal kullanımda sayfa
  geçişleri test edildiğinde, hâlâ donma varsa STALL izleyicisi
  (crash_rapor_son.txt) hangi widget/satırın sorumlu olduğunu kesin
  gösterecek - bir sonraki adım bu kanıtla kök nedeni netleştirmek.
- **YENİ ÖZELLİK: Yer istasyonu bataryası şarj göstergesi (2026-09-05,
  kullanıcı: "araç Jetsonunu yer istasyonundaki batarya besliyor, onun
  şarjını yerin şarj göstergesine çekelim, batarya 22.2V LiPo buck
  converter ile sabitliyoruz") —** eskiden `label_yerSarj` ("Yer Şarjı"
  göstergesi) altyapısı VARDI (`yer_batarya_sinyali`, `yer_batarya_
  renklendir` - main.py) ama HİÇBİR YERDE `.emit()` edilmiyordu, tamamen
  boştu. Artık kontrol paneli Arduino'sunun (arduino_kontrol_paneli/
  arduino_kontrol_paneli.ino) BOŞ analog pini A2, GERİLİM BÖLÜCÜ
  üzerinden 6S (22.2V nominal) LiPo'nun gerilimini ölçer:
  - **DONANIM (kullanıcı tarafından fiziksel olarak kurulmalı):**
    `Batarya(+) --[R1=100kΩ]-- A2 --[R2=22kΩ]-- GND`, batarya(-) Arduino
    GND'sine ortak. Bölme oranı 22/122=0.1803 → 6S dolu gerilimi
    (25.2V) bile A2'de sadece ~4.54V görünür (Arduino'nun 5V analog
    sınırının altında, güvenlik payı var). **Bölücü kurulmadan hiçbir
    şey A2'ye bağlanmamalı** - aksi halde Arduino zarar görebilir.
  - **FIRMWARE:** protokol `"P,A3,A4,A0,A1,A5,D4,D5,D9,D10"`'dan
    `"P,A3,A4,A0,A1,A5,A2,D4,D5,D9,D10"`'a genişletildi (A2, A5'ten
    hemen sonra eklendi). `arduino-cli` ile derlenip
    (`arduino:avr:uno`) UZAKTAN yüklendi ve doğrulandı (canlı seri
    okumayla 11 alanlı yeni format teyit edildi).
  - **PYTHON (kontrol_paneli_sistemi.py):** `_satir_ayristir` HEM eski
    (10 alan) HEM yeni (11 alan) formatı kabul eder - firmware henüz
    güncellenmemiş bir Arduino'da bile joystick/toggle/buton ÇALIŞMAYA
    DEVAM EDER, sadece A2 `None` sayılır. `_yer_bat_yuzde_hesapla()`
    ham ADC'yi AYNI R1/R2 değerleriyle gerçek voltaja, sonra 6S LiPo
    hücre aralığına (3.3V=%0, 4.2V=%100, yaygın pratik) göre yüzdeye
    çevirir. EMA yumuşatma (`YER_BAT_EMA_ALFA=0.1`, pot'tan daha yavaş
    - şarj hızlı değişmez) + throttle (`YER_BAT_MIN_ARALIK_S=2.0`sn)
    ile `yer_batarya_sinyali` (`"%XX"` formatı) yayınlanır.
  - **BAĞLANTI:** main.py'de `label_yerSarj` artık
    `kontrol_paneli_motoru.yer_batarya_sinyali`'ne bağlı (ESKİDEN yanlışlıkla
    `telemetri_motoru`'daki - hiç emit edilmeyen - AYNI İSİMLİ sinyale
    bağlıydı; gerçek veri kaynağı ROS2/ağ değil, kontrol paneli Arduino'su
    olduğu için doğru thread'e taşındı).
  - **DOĞRULAMA:** PANEL TESTİ dialogu artık "Yer bataryası: A2=xxx →
    X.XXV (%XX)" satırını da gösteriyor - gerilim bölücü kurulurken
    canlı doğrulama için kullanılabilir (A2 hâlâ boştaysa "YOK (eski
    firmware - A2 gönderilmiyor)" değil, düşük/gürültülü bir ADC değeri
    gösterir - firmware zaten güncel, sadece devre henüz yok).
  - **GÜNCEL DURUM (2026-09-05, kullanıcı kararı):** kullanıcı gerilim
    bölücü lehimlemek İSTEMEDİ; alternatif olarak sunulan hazır I2C güç
    izleme modülü (INA219/INA226 - lehimleme gerektirmez, Jetson'un
    genel amaçlı ADC'si OLMADIĞI ve buck converter çıkışı zaten SABİT
    olduğu için "Jetson üzerinden doğrudan ölçüm" seçeneği YOK, bunlar
    açıklandı) de ŞİMDİLİK istenmedi - "böyle kalsın" denildi. Gösterge
    bilinçli olarak PASİF/%0 bırakıldı - kod GÜVENLİ ve HAZIR bekliyor
    (A2 boşta olduğu için düşük/gürültülü bir ADC okuyup %0'a clamp
    ediyor, ZARARSIZ). İleride bir ölçüm donanımı (gerilim bölücü VEYA
    INA219/INA226) eklenirse ek bir kod değişikliği GEREKMEDEN (INA226
    seçilirse ayrı bir entegrasyon gerekir) otomatik devreye girer.
- **`~/.bashrc`'ye ROS2 ortamı otomatik yükleme eklendi (2026-09-05,
  kullanıcı: "arayüz açılmıyor... vs code açıldığında terminalde sanal
  ortam çalışsın belki o şekilde kasma olmaz") —** eskiden ROS2 SADECE
  `calistir.sh` içinde source ediliyordu; VS Code'un entegre terminalinden
  (ya da başka bir terminalden) doğrudan `python3 main.py` çalıştırılırsa
  ROS2 source edilmemiş oluyordu. main.py'nin kendi kendini düzelten
  (`TUFAN_ENV_HAZIR`, bkz. dosya başı) mekanizması bunu YAKALAYIP kendini
  yeniden başlatıyor ama YAVAŞ, VE manuel `ros2 topic ...` gibi komutlar
  hâlâ çalışmıyordu (ROS2 hiç kurulu değilmiş gibi davranıyordu). Artık
  `~/.bashrc` sonuna `source /opt/ros/humble/setup.bash`,
  `source ~/ros2_humble/install/setup.bash`, `export ROS_DOMAIN_ID=77`
  eklendi - yeni bir interaktif bash oturumunda (`bash -i -c`) hem
  `ROS_DOMAIN_ID=77` hem `import rclpy` başarısı CANLI DOĞRULANDI. HER
  YENİ terminal (VS Code dahil) artık baştan doğru ortamda açılıyor -
  hem "açılmıyor" ihtimalini hem de manuel komutların yanlış/kalabalık
  domain'de (0) çalışıp yavaşlaması riskini ortadan kaldırır.
- Uydu harita disk önbelleği (`~/.cache/tufan_harita_tiles/`) süresiz
  saklanıyor, otomatik yenilenmiyor — Esri görüntüyü güncellese bile eski
  tile diskte kalırsa gösterilmeye devam eder (manuel temizlik gerekir).
- `/turret_hedef_mesafe` şu an araç tarafında gerçek bir yayıncısı olmayan,
  arayüz tarafında hazır bekleyen bir sözleşme — TF02-Pro lidar entegrasyonu
  tamamlanınca otomatik çalışacak.
- **TF02-Pro lidar (silah/turret) canlı teşhis edildi ve DÜZELDİ (2026-09-05,
  kullanıcı: "araçtaki silah lidarından veri çekebiliyor muyum kontrol et") —**
  `turret_node.py` içinde `_arduino_satir_isle`, TF02-Pro'nun "D:<cm>\n"
  raporunu okuyup paralaks düzeltmesinde kullanıyor ama DIŞARI (ROS2 topic
  olarak) HİÇ YAYINLAMIYOR - `/turret_hedef_mesafe` bu yüzden hâlâ yok (yukarıdaki
  madde hâlâ geçerli, bu ayrı bir konu). Teşhis: `turret_node`'u kısa süreliğine
  durdurup, hazır bir debug sketch'i (`~/Desktop/silah_ws/lidar_test_standalone/
  lidar_test_standalone.ino`, ham bayt/checksum/sinyal gücünü gösteriyor)
  `arduino-cli` (arayüz Jetson'da derlenip) + `avrdude` (araç Jetson'da yükleme,
  FQBN `arduino:avr:mega`, `-cwiring -b115200`) ile GEÇİCİ olarak yüklendi,
  sonuç gözlemlendi, ORİJİNAL firmware (`sketch_jul28b.ino`) HER SEFERİNDE geri
  yüklendi - kaynak kodun kendisi hiç değişmedi. İlk denemelerde sensör hep
  geçersiz (`D:-1`) veriyordu (checksum tutarlıydı, Arduino↔Jetson USB bağlantısı
  sağlıklıydı - sorun TF02-Pro'nun kendisindeydi). Kullanıcı fiziksel kablolamayı
  (Serial3: TX→pin15/RX3, RX→pin14/TX3, ortak GND) düzelttikten sonra debug
  sketch'te sinyal gücü 14'ten binlere çıktı, mesafe tutarlı hale geldi - AMA
  orijinal firmware ile YAPILAN İLK testte HÂLÂ `-1` görüldü (muhtemelen konektör
  henüz tam oturmamıştı); HEMEN ARDINDAN tekrar denendiğinde orijinal firmware de
  100/100 geçerli okuma verdi. **Sonuç: TF02-Pro artık production'da sağlıklı
  çalışıyor** - `sonMesafeCm` doğru güncelleniyor, sadece dışarı yayın (yukarıdaki
  `/turret_hedef_mesafe` maddesi) hâlâ eksik, istenirse ayrı bir iş olarak eklenebilir.
- **Silah artık aynı fiziksel Manuel↔Otonom düğmesiyle (D9, sol buton)
  değişiyor (2026-09-05, kullanıcı: "araçtaki manuel otonom geçiş fiziksel
  tuşu ... silah manuel otonom geçiş tuşu yap") —** `turret_node.py` zaten
  `OTONOM` modunu (YOLO ile otomatik hedef takibi) destekliyordu ve kendi
  içinde güvenlik önlemi alıyordu (OTONOM'dan çıkışta lazer anında kapanır,
  PID/hedef takibi sıfırlanır, geçersiz mod değerleri yok sayılır - bkz.
  `_mod_cb`) AMA arayüz `telemetri_sistemi.py::surekli_yayin_dongusu` içinde
  KOŞULSUZ olarak sürekli `"MANUEL"` yayınladığı için bu özellik hiç
  kullanılamıyordu. Tek satır değişti: artık `self.arac_modu` (aracın kendi
  Manuel/Otonom durumu, `mod_degistir()` ile TEK merkezde tutulan) doğrudan
  `/silah_modu`'na yayınlanıyor - `main.py::_panel_mod_toggle` (D9 fiziksel
  butonu) zaten `otonom_sec()`/`manuel_sec()` üzerinden `mod_degistir("OTONOM"/
  "MANUEL")` çağırdığı için EK bir kod değişikliği gerekmedi, mimari zaten
  merkezi durum kullanıyordu. Kontrol paneli Arduino'su bağlanınca hâlâ
  güvenli varsayılan olarak bir kerelik `"MANUEL"`'e zorlanıyor
  (`silah_manuel_moduna_al`, DOKUNULMADI) - ama hemen ardından
  `surekli_yayin_dongusu` (0.1sn'de bir) gerçek `arac_modu`'yu yayınlamaya
  devam ediyor, yani bu sadece bağlantı anındaki geçici bir güvenlik adımı.
- **Sol menüdeki "ATEŞ" (lazer) butonu kaldırıldı, yerine "🚀 ARACI BAŞLAT"
  butonu eklendi (2026-09-05, kullanıcı: "arayüzde lazer açma kapama tuşu
  olan sol tarafta onu kaldır yerine araca bağlanıp kodları açmaya yarayan
  bir buton ekle") —** `main.py::_silah_ates_butonu_ekle` ikiye ayrıldı:
  `_ates_zamanlayici_kur()` (zamanlayıcı kurulumu, HER ZAMAN çağrılır - bkz.
  `__init__`) ve `_arac_baslat_butonu_ekle()` (yeni buton, AYNI konumda -
  `sol_menu_frame`, `pushButton_pwmIzle`'nin üstünde). ÖNEMLİ: fiziksel
  kontrol panelindeki SAĞ TOGGLE SWITCH (`_panel_ates`, D5, gerçek "ateş"
  anahtarı) `_ates_baslat`/`_ates_durdur`/`_ates_zamanlayici`'yi AYNEN
  kullanmaya devam ediyor - ekran butonu kaldırıldı ama FİZİKSEL anahtarla
  ateş etme yeteneği BOZULMADI, sadece zamanlayıcı kurulumu butondan
  bağımsız hale getirildi. Yeni buton (`_arac_baslat_onayla` →
  `_arac_baslat_tetikle`) tıklanınca önce bir onay diyaloğu gösterir
  (yanlışlıkla dokunmaya karşı - eskiden AYNI KONUMDA ateş butonu vardı),
  "Evet" denirse `subprocess.Popen(["ssh", ...])` ile (liste formatında,
  `shell=True` DEĞİL - komut sabit olduğu için injection riski yok, GUI
  thread'i bloke etmemek için `.wait()` çağrılmıyor) araç Jetson'a bağlanıp
  `tufan_ana_terminal` tmux oturumunu (yoksa oluşturarak) kullanarak
  `tufan_mppi.launch.py`'yi başlatır - session boyunca elle yapılan "SSH
  bağlan + tmux + launch komutu gönder" işlemini tek tıkla yapar. Canlı
  test edildi: arayüz hatasız başladı, buton doğru konumda göründü.
- **KRİTİK: Joystick/turret sinyalleri artık Qt.DirectConnection ile ayrı
  thread'de işleniyor (2026-09-05, kullanıcı: "joystickte çok gecikme
  oluyor... ayrı bir kanal üzerinden gönderelim... ayrı bir thread olarak
  çalıştır") —** STALL izleyicisi kanıtladı: gecikmenin kaynağı ROS2/network
  DEĞİLDİ - GUI thread, `TaktikRadarEkrani.paintEvent`'te (harita render'ı,
  native Qt çağrısı) SIK SIK 420-870ms takılı kalıyordu. Varsayılan
  `Qt.AutoConnection` ile `KontrolPaneliThread`'den emit edilen
  `surus_sinyali`/`turret_sinyali` GUI thread'in event kuyruğuna
  KUYRUKLANIYORDU - GUI thread paintEvent'te meşgulken joystick/turret
  komutları da AYNI ORANDA gecikiyordu (turret_cmd_pub QoS sorununda
  bulduğumuzla AYNI mekanizma: veri hızlı geliyor ama tek meşgul GUI thread
  kuyruktaki HER ŞEYİ eşit geciktiriyor). Çözüm: `main.py::_panel_surus_geldi`
  ve `_panel_turret_geldi` HİÇBİR GUI elemanına dokunmuyor (sadece
  `telemetri_motoru`'nun thread-safe metodlarını çağırıyor - biri BEST_EFFORT
  ROS2 publish, diğeri basit float atama), bu yüzden `connect(..., 
  Qt.DirectConnection)` GÜVENLİ - artık bu sinyaller GUI thread'e HİÇ
  UĞRAMADAN, `KontrolPaneliThread`'in KENDİ thread'inde SENKRON olarak
  işleniyor. GUI thread ne kadar meşgul olursa olsun (harita render'ı
  tıkansa bile) joystick/turret komutları artık ANINDA gönderiliyor.
- **TaktikRadarEkrani LIDAR/costmap render'ı toplu çizime çevrildi
  (2026-09-06, kullanıcı: "lidar ekranında çok fazla komut verisi
  gecikiyor, düzenle ve test et") —** eskiden HER LIDAR/costmap noktası
  için AYRI bir `drawEllipse()`/`drawRect()` çağrısı vardı (300'e kadar
  her biri) - STALL izleyicisi bu satırları defalarca 400-900ms+ takılı
  yakalamıştı (her ayrı native Qt çağrısı PyQt/SIP binding geçişi
  taşıyor, yazılımsal render'da yüzlerce noktada toplamda büyük maliyete
  dönüşüyordu). Artık tüm noktalar TEK bir `QPolygonF`'e toplanıp TEK bir
  `drawPoints()` çağrısıyla çiziliyor - 300 ayrı native çağrı yerine 1
  çağrı. Canlı doğrulandı: STALL izleyicisinde bu satırlar artık HİÇ
  görünmüyor (darboğaz başarıyla kaldırıldı).
- **`log_yaz` optimize edildi + KRİTİK: stdout "-u" (unbuffered) bayrağı
  GERİ ALINDI (2026-09-06) —** `log_yaz`'daki manuel "500 satırı aşınca
  100 sil" mantığı Qt'nin native `document().setMaximumBlockCount(500)`
  mekanizmasıyla değiştirildi (Python seviyesinde imleç/seçim/silme işlemi
  yerine C++ tarafında otomatik yönetim). ÇOK DAHA ÖNEMLİSİ: canlı testte
  `log_yaz` içindeki `print()` çağrısının KENDİSİ GUI thread'de **5.91
  SANİYE** takılı yakalandı (bir örnekte kapanış sırasında `KameraThread.
  stop()`'taki `self.wait()` de **19.65 SANİYE** sürdü) - kök neden:
  önceki bir oturumda eklenen "-u" bayrağı her `print()`'i tamponsuz/anlık
  yazmaya zorluyordu, bu Jetson paylaşılan bir masaüstü olduğu için (VS
  Code + Claude Code CLI + ROS2 + GUI aynı anda) CPU açlığı anlarında
  `tee` process'i pipe'ı yeterince sık boşaltamıyor, pipe buffer dolunca
  YAZICI TARAF (GUI thread'deki print() çağrısı) OS seviyesinde bloke
  oluyordu. `calistir.sh`'taki "-u" kaldırılıp normal (tamponlu, ~8KB
  bloklar halinde flush edilen) moda dönüldü - bu, gerçek zamanlı araç/
  silah kontrolünü saniyelerce dondurabilen bu riski ortadan kaldırır.
  Bedel: crash/force-quit anında flush edilmemiş son birkaç satır
  kaybolabilir - kabul edilebilir, çünkü gerçek SIGSEGV/SIGABRT'ta zaten
  faulthandler AYRI bir dosyaya (crash_log.txt) tam stack trace yazıyor
  (stdout buffer'ından bağımsız). Güvence olarak `closeEvent`'e (temiz
  kapanışta tamponu boşaltmak için) `sys.stdout.flush()` eklendi.
- **KRİTİK PERFORMANS: Taktik Radar zemin (arka plan + grid) önbelleğe
  alındı, paintEvent %63 hızlandı (2026-09-06, kullanıcı: "ana ekran ve
  kamera ekranı düşük gecikmeli fakat lidar ekranında çok gecikme var...
  bazen arayüz iyi çalışıyor ardından kötüleşebiliyor") —** ÖNCE ÖLÇÜLDÜ
  (tahmin edilmedi): paintEvent'e geçici bölüm bazlı profilleme eklendi
  (`_profil_kaydet`, `[RADAR-PROFIL]` çıktısı) ve şu kırılım bulundu:
  toplam ~26-30ms'nin **grid ~8.4ms + arka plan (QRadialGradient ile TÜM
  ekranı doldurma) ~5.6ms + costmap ~5.3ms**'si. Ayrıca `[KUYRUK-GECİKME]`
  ölçümü, sinyallerin GUI'ye ulaşmasının 57-64ms geciktiğini ama
  KATLANARAK ARTMADIĞINI gösterdi (yani kuyruk taşması YOK) ve veri
  boyutları sabitti (lidar=255, costmap=286) - yani "zamanla kötüleşme"
  bir BİRİKME değil, her karede ödenen sabit ama ÇOK YÜKSEK bir maliyetti
  (render 33ms'de bir isteniyor, her biri ~30ms sürüyor → GUI thread'in
  neredeyse tamamı paintEvent'te geçiyor, komutlara zaman kalmıyordu).
  **Çözüm:** arka plan gradyanı + grid + merkez eksenleri, `_zemin_pixmap_al()`
  ile TEK bir `QPixmap`'e önbelleklendi - bunlar sadece pencere boyutu,
  pan (sürükleme) veya zoom değişince yeniden üretilir (önbellek anahtarı:
  `(g, y, pan_x, pan_y, maksimum_menzil)`), her karede sadece TEK bir
  `drawPixmap` yapılır. Ek olarak: grid döngüsünde her iterasyonda YENİ
  QPen yaratma kaldırıldı (2 kalem döngü dışında), costmap döngüsündeki
  ~300 `_dunya_to_ekran()` çağrısı (her biri `_gecerli_olcek()`'i yeniden
  hesaplıyordu) satır içi matematiğe çevrildi, mesafe etiketlerindeki
  döngü içi `setPen` dışarı alındı, merkez eksenlerinin duplike çizimi
  kaldırıldı. **Canlı ölçülen sonuç:** paintEvent ort **29.7ms → 10.3-11.5ms**,
  en kötü **57.6ms → 21-30ms**, costmap **14.2ms → 2.2-3.5ms**, sinyal
  kuyruk gecikmesi **57-64ms → 31-35ms**. NOT: `[RADAR-PROFIL]` ve
  `[KUYRUK-GECİKME]` çıktıları KASITLI olarak bırakıldı - ileride benzer
  bir yavaşlama olursa kök neden tahmin edilmeden doğrudan ölçülebilsin.
- **KÖK ÇÖZÜM: Kontrol paneli AYRI BİR PROCESS'e taşındı — joystick
  gecikmesi bitti (2026-09-06, kullanıcı: "arduinodan çektiğimiz verileri
  arka tarafta bir panelden gönderelim, arayüz üzerinde kasma oluyor lidar
  ekranında") —** Kullanıcının teşhisi doğruydu. GERÇEK KÖK NEDEN: 
  `KontrolPaneliThread` main.py'nin İÇİNDE bir QThread'di, yani arayüzle
  **AYNI GIL**'i paylaşıyordu. Taktik Radar açıkken GUI thread her karede
  CPU-yoğun çizim yaparken GIL'i tutuyor, joystick thread'i veriyi okusa
  bile yayınlamak için GIL'i bekliyordu. Daha önce denenen üç adım da
  (turret BEST_EFFORT QoS, `Qt.DirectConnection`, paintEvent'in
  29.7ms→11ms optimizasyonu) marjı genişletti ama **GIL rekabetini
  ortadan kaldıramazdı** - çünkü sorun Qt kuyruğu değil, yorumlayıcı
  kilidiydi.
  **Çözüm:** yeni `kontrol_paneli_node.py` — Arduino'yu AYRI PROCESS'te
  (ayrı GIL) okuyup `/palet_hizlari`, `/turret_manuel_cmd`,
  `/silah_ates_manuel`, `/arac_komut` (ACİL STOP) topic'lerini DOĞRUDAN
  yayınlar. Kod tekrarı YOK: mevcut `KontrolPaneliThread` sınıfı (tüm
  kalibrasyon/ölü bölge/oto-merkezleme/güvenlik ağı mantığıyla) olduğu
  gibi kullanılır, sadece Qt sinyalleri ROS2 yayıncılarına bağlanır
  (GUI'siz `QCoreApplication` ile). Arayüz artık paneli HİÇ okumaz;
  sadece `/panel_durumu` (JSON) topic'inden GÖSTERGELERİ günceller.
  **GÜVENLİK İYİLEŞTİRMESİ:** ACİL STOP artık arayüzün sağlığına bağlı
  değil - arayüz donsa/çökse bile fiziksel anahtar araca ulaşır.
  **Ölçülen sonuç:** `/palet_hizlari` yayın aralığı **ort 100.0ms,
  en kötü 106.5ms, en iyi 92.9ms** (hedef 100ms) - yani komutlar artık
  GUI yükünden tamamen bağımsız, saat gibi düzenli. Öncesinde aynı
  ölçüm en kötü **1392ms** boşluk gösteriyordu.
  **Yol boyunca yakalanan ve düzeltilen 4 tuzak (tekrar düşmemek için):**
  1. *İki yayıncı çakışması:* main.py de `/palet_hizlari` yayınlamaya
     devam ediyordu (joystick verisi artık ona gelmediği için 0
     gönderiyordu) - ölçümde 100ms yerine 49.8ms aralık olarak yakalandı.
     `telemetri_sistemi.py::surekli_yayin_dongusu` artık
     `surus_kaynagi == "JOYSTICK"` iken SUSUYOR (klavye sürüşünde
     eskisi gibi yayınlamaya devam eder).
  2. *Yarış durumu:* bağlantı olayı SADECE değişimde yayınlanıyordu;
     node arayüzden önce başlarsa arayüz `surus_kaynagi`'nı hiç
     "JOYSTICK" yapmıyor ve çakışma geri geliyordu. Node artık bağlantı
     durumunu ~2sn'de bir TEKRAR yayınlıyor.
  3. *Orphan/çift node:* arayüz çöküp süpervizörce yeniden başlatılınca
     eski node sahipsiz kalıyor, yenisi de eklenince 2-3 node AYNI seri
     porttan okuyup veriyi bozuyordu (logda aynı ACİL STOP olayının
     saniyede bir tekrarı bunun belirtisiydi). Node'a `fcntl.flock`
     tabanlı **tekil çalışma kilidi** eklendi - ikinci örnek sessizce
     çıkar, kilit process ölünce çekirdekçe otomatik bırakılır.
  4. *Arayüzü dondurma (İKİ KEZ):* orphan temizliği için
     `subprocess.run(timeout=5)+sleep` denendi → `__init__` GUI
     thread'inde olduğu için arayüz AÇILIŞTA dondu ("force quit"
     ekranı); sonra `Popen` denendi → bu kez pkill yeni başlatılan
     node'u da öldürme yarışı yarattı. Sonuç: temizlik tamamen
     kaldırıldı, tekil kilit yeterli. **Ders: `_kontrol_paneli_baslat`
     GUI thread'inde çalışır, oraya ASLA bloke eden çağrı konmaz.**
- **PANEL TESTİ ayrı process mimarisine entegre edildi (2026-09-06,
  kullanıcı: "arayüzdeki panel testini de entegre edelim, bazen
  joystick'i sıfırlamak gerekiyor") —** Panel ayrı process'e taşınınca
  PANEL TESTİ dialogu kopmuştu (arayüz artık `KontrolPaneliThread`
  metotlarını doğrudan çağıramıyor). Yeni `/panel_komut` (String) topic'i
  bu köprüyü kuruyor - arayüz → node yönünde:
  - `merkezi_sifirla` → joystick merkezini yeniden kalibre eder
    (kullanıcının en çok ihtiyaç duyduğu işlev)
  - `ayarlari_yeniden_yukle` → "KAYDET VE UYGULA" sonrası diske yazılan
    kalibrasyon ayarlarını node'a uygulatır
  - `ham_veri_ac` / `ham_veri_kapat` → 50Hz ham panel verisi akışı
    SADECE dialog açıkken üretilir (kapalıyken hiç trafik yok)
  Ham veri artık `/panel_durumu` üzerinden `{'tip':'ham','veri':{...}}`
  olarak geliyor ve dialogdaki mevcut `guncelle()` fonksiyonuna
  yönlendiriliyor - dialogun görünümü/mantığı DEĞİŞMEDİ.
  **Uçtan uca canlı doğrulandı:** merkez sıfırlama komutu node loguna
  düştü (`PANEL: joystick merkezi sıfırlama istendi`), ham veri akışı
  gerçek ADC değerleriyle geldi (`{"tip":"ham","veri":{"A3":498,...}}`),
  ve bu sırada sürüş yayını düzenliliğini korudu (ort 97ms).
- **LAZER 1 SANİYE SONRA SÖNÜYORDU - ateş heartbeat'i kayıptı
  (2026-09-06, kullanıcı: "lazeri açıyorum fakat 1sn sonra sönüyor,
  switch açık olmasına rağmen") —** Araç tarafındaki `turret_node.py`
  manuel ateşi `_ATES_MANUEL_TIMEOUT_S = 0.5` ile koruyor: bu bilinçli
  bir GÜVENLİK önlemi (arayüz çökerse / bağlantı giderse lazer açık
  takılı kalmasın), yani `/silah_ates_manuel` **sürekli tazelenmek
  zorunda**. Panel ayrı process'e taşınmadan önce bu tekrarı
  `main.py`'deki 50ms'lik `_ates_zamanlayici` yapıyordu; taşıma
  sırasında sağ toggle sinyali artık main.py'ye HİÇ gelmediği için o
  zamanlayıcı ölü kod haline geldi ve ateş, node'da yalnızca
  **kenar-tetiklemeli tek mesaja** düştü -> araç 0.5sn sonra lazeri
  kapatıyordu.
  **Çözüm:** heartbeat node'a taşındı - `ates_degisti()` artık sadece
  `ates_manuel_aktif` bayrağını kurup anında yayınlıyor, `tick()` de
  (100ms, zaman aşımına 5 kat pay) anahtar basılıyken TEKRAR yayınlıyor.
  Heartbeat mod kontrolünden ÖNCE, yani taşımadan önceki main.py
  davranışıyla birebir aynı (ateş kararı araç tarafına ait). Acil stop
  silahı KASITLI OLARAK etkilemiyor (bkz. kontrol_paneli_sistemi.py).
  main.py'deki ölü ateş yolu (`_ates_zamanlayici_kur`, `_ates_baslat`,
  `_ates_durdur`, `_panel_ates`, `_telemetri_ates_gonder`) ve onu hâlâ
  "çalışıyor" diye anlatan YANILTICI yorum KALDIRILDI.
  **Ders: karşı taraf heartbeat bekliyorsa, o yayını taşırken
  zamanlayıcısını da taşı - kenar-tetiklemeli tek mesaj yetmez.**
  **Canlı ölçüm (switch 60sn açık tutuldu):** `/silah_ates_manuel`
  üzerinde 609 mesaj, ort **101.0ms**, anahtar basılı geçen HER saniyede
  tam 10 mesaj - yani 0.5sn'lik araç zaman aşımı bir kez bile
  tetiklenmedi. Ölçümdeki 979ms'lik tek boşluk, anahtarın KAPATILDIĞI
  ana ait (kapanınca heartbeat'in durması beklenen davranış).
- ARKA kamera için henüz fiziksel donanım/UDP kaynağı yok, kutu şimdilik boş.
- `turret_cmd_listener.py` ve eski `/turret_cmd_vel` topic'i artık
  kullanılmıyor (silah kontrolü arayüzden kaldırıldı, ileride ayrı bir
  joystick ile yeniden yapılacak).

## OTONOM: ÖNE OTOMATİK NOKTA ATMA (araç: on_bosluk_nokta_atici.py, 2026-09-06)

Kullanıcı isteği: *"araç odometrisi kaydığında attığım noktalarda kayıyor;
bunun yerine araç önüne noktalar atarak giderse konum tahmini kaymasından
dolayı otonom sürüş etkilenmez ve engelleri ortalayarak gider. Önüne nokta
atarken lidarın ön tarafını kullansın, boş bulduğu yerlere atsın."*

**Kayma sorununu nasıl çözüyor:** elle atılan hedef `odom` çerçevesinde
SABİT durur; odometri kayınca araçla hedef arasındaki gerçek geometri
bozulur. Bu düğüm hedefi HER TICK'TE (4 Hz) o anki LIDAR taramasından
YENİDEN üretir; hedefin ömrü saniyenin altındadır, o sürede birikecek
kayma santimetre mertebesindedir. Kayma DÜZELTİLMİYOR, ÖNEMSİZLEŞİYOR.

**2D LIDAR tuzakları ve çözümleri** (kullanıcının saydığı parkur öğeleri):
- (A) *İnişte zemini duvar sanma* ve (B) *yan eğimde bir yanın zemine,
  diğerinin gökyüzüne bakması* — FİZİKSEL çözüm: her ışının zemine hangi
  mesafede çarpacağı, gövdenin gerçek yönelimi (odom quaternion) ve LIDAR
  yüksekliği (0.575 m, URDF) ile HESAPLANIR; o mesafeye ulaşan okumalar
  zemindir, elenir. Hesap Euler açısı çıkarmadan doğrudan dönme matrisinin
  3. satırıyla yapılır — roll/pitch İŞARET konvansiyonu (bu projede
  defalarca sorun çıkardı) denklemin dışında kalır.
- (C) *Tırmanırken her yeri boş görme* — YAZILIMSAL ÇÖZÜMÜ YOK, sensör o
  bilgiyi hiç toplamıyor. Tek doğru davranış yanlara güvenmemek: fan
  daraltılır, kısa adımlarla düz gidilir.
- (D) *Yokuşu/dik engeli duvar sanma* — ölçüm doğru, ÇIKARIM yanlış. Ön
  sektörü kapatan şey gövdeye DİK, DÜZ ve KESİNTİSİZ bir yüzeyse ve fan
  bunun ötesine geçemiyorsa, bu parkurda çıkmaz olmadığı için tırmanılacak
  rampadır -> yüzeye dik, kısa hedefle yaklaşılır (`rampa_modu_acik`).

**Hedef seçimi:** açı taraması (fan ±85°); her aday için araç genişliğinde
koridor süpürülür (gidilebilen mesafe) ve YOL BOYUNCA en dar yanal açıklık
ölçülür. Puan = açıklık + katedilen mesafe - düz-gitme tercihi - histerezis.
Bu TEK puanlama koridor ortalamayı, slalomu ve dönüşleri birlikte üretir.

**Ayarlar (kullanıcı kararı, 2026-09-06):** hedef atma fanı **±60°
(toplam 120°)**, hedef mesafesi **2 m**. Nokta TOPLAMA açısı bundan
BAĞIMSIZ (±105°) — araç dar bir koniye hedef atsa da yanlarını görmeye
devam etmeli, yoksa yol açıklığını yanlış ölçer.

Bu iki değişiklik üç iç mantığı bozdu, üçü de düzeltildi:
1. *Rampa geçiş testi kırpılmış hedef mesafesine bakıyordu* — hedef 2 m'ye
   inince `serbest` mesafe 2.6 m'de tavanlanıyor, rampa eşiği (yüzey+1 m)
   hiç aşılamıyor ve HER köşe "rampa" sayılıyordu. Test artık kırpılmamış
   serbest mesafeyi kullanıyor (`tavan_ust`).
2. *Rampa kararı dar hedef fanına soruluyordu* — ±60'a inince köşedeki
   gerçek geçit fan dışında kalıyor, düğüm köşeyi çıkmaz sanıp aracı
   duvara sürüyordu. Karar artık GENİŞ algı açısına soruluyor: araç o
   geçide hedef atmasa bile onu GÖRÜYOR, görmek "burası çıkmaz değil"
   demek için yeterli. Geçerli aday yoksa `KAPALI` — durur, duvara sürmez.
3. *Nokta toplama açısı fana bağlıydı* — ayrıldı (yukarıda).

**Beklenmedik iyileşme:** 2 m'lik kısa ufuk, dar koridorda ortadaki
dubalarda BELİRGİN iyileşme sağladı (3.6 m koridor senaryosu artık
tamamen geçiyor, 3.2 m senaryosu 5.8 m yerine 11 m ilerliyor) — kısa ufuk
planlayıcıyı yakın geometriye göre karar vermeye zorluyor. Kapalı döngü
testi 5/7'den **6/7**'ye çıktı.

**±60'ın bilinen bedeli:** 90°'lik keskin köşede yan geçit fan dışında
kalırsa araç dönemez, `KAPALI` moduna geçip DURUR (artık duvara sürmüyor).
Parkurda keskin dönüş varsa `fan_yari_acisi_derece` yeniden derlemeden
büyütülebilir.

### OTONOM'A GEÇİNCE OTOMATİK BAŞLAMA (2026-09-06)
Kullanıcı isteği: *"otonoma geçtiğimde nokta atma işlemini otomatik olarak
başlatacak kodu yaz."* Düğüm artık `/surus_modu`'nu dinliyor: arayüzden
**OTONOM'a geçilince kendiliğinden başlar, MANUEL'e dönülünce durur.**

Aktiflik üç bağımsız kaynaktan gelebilir (`_aktif_mi`):
1. **Elle** `/oto_nokta_aktif` — moddan bağımsız, tezgah/saha testi için
2. **Sürüş modu = OTONOM** — varsayılan yol (`surus_modu_ile_aktiflesir=True`)
3. Görev sekansı `/otonom_surus_aktif` — doğrulanana kadar KAPALI

`/surus_modu`, `telemetri_sistemi.py::surekli_yayin_dongusu` tarafından
**her turda** tekrar yayınlandığı için düğüm geç başlasa bile modu öğrenir
(sadece değişimde yayınlansaydı bu kaçırılabilirdi — doğrulandı).

**GÜVENLİK — mod bayatlığı:** `/surus_modu` 5 saniye boyunca hiç gelmezse
(arayüz kapandı/çöktü) mod bilgisi bayat sayılır ve otomatik aktiflik
DÜŞER — arayüzsüz kalan araç kendi başına hedef üretmeye devam etmesin.
Elle açma bundan etkilenmez. Aktiflik değişimi log'a BİR KEZ yazılır
(her tick değil).

**Offline test (8/8 geçti):** mod bilinmiyor→pasif, MANUEL→pasif,
OTONOM→**aktif**, MANUEL→pasif, OTONOM+bayat→pasif, bayatken elle→aktif,
parametre kapalı→pasif, geçiş logları tekrarlamıyor.

**Canlı doğrulama:** `/surus_modu` abone sayısı 3→4 (düğüm katıldı), mod
`MANUEL` iken `/oto_nokta_durum` sessiz — yani düğüm doğru şekilde pasif.

**Topic'ler:** `/oto_nokta_aktif` (Bool, elle açma) · `/otonom_surus_aktif`
(görev sekansı — `otonom_surus_ile_aktiflesir` DOĞRULANANA KADAR False) ·
çıkış `/ugv_goal` (PoseStamped, odom) · teşhis `/oto_nokta_durum` (JSON) ·
`/rampa_yaklasimi` (Bool). Tüm eşikler ROS2 parametresi — sahada
`ros2 param set` ile YENİDEN DERLEMEDEN ayarlanır.

### Simülasyonda bulunan ve düzeltilen KRİTİK hatalar
Düğüm araca kurulmadan ÖNCE 15 sentetik senaryo + 7 kapalı-döngü
simülasyonuyla test edildi; şunlar canlı testte değil, masada yakalandı:
1. **Tek atışlık planlayıcı**: "hedef az değişti" filtresi, araç sabit bir
   hedefe yaklaşırken hedefi HİÇ tazelemiyordu (11 tick üst üste yayın
   atlandı) — araç tam ortadaki dubaya 1.8 m kala tepki verdi. Receding
   horizon zorunluluğu eklendi (`maks_hedef_yasi_s`, `yenileme_mesafesi_m`).
2. **Açıklığın yanlış yerde ölçülmesi**: açıklık sadece HEDEF NOKTASINDA
   ölçülüyordu; hedef engelin hemen kısasına atıldığı için "önde duba" ile
   "yandan geçen yol" neredeyse aynı puanı alıyordu. Yol BOYUNCA ölçüme
   geçildi — araç tarafını metrelerce önceden seçiyor.
3. **Parkuru ters yönde sürme**: ±100° fan ile araç sıkışınca dönüp geri
   gitti ve duvara çarptı. Fan ±85° (ileri bileşen hep pozitif) + referans
   yön koruması (`geri_donus_esigi_derece=110`) eklendi.
4. **Öndeki engelin görmezden gelinmesi**: "yanımdan geçen noktaları yok
   say" kuralı TAM ÖNDEKİ dubayı da eliyordu -> son yarım metrede çarpma.
   Kural "kısa mesafe VE gövde genişliğinin DIŞINDA" olarak düzeltildi.
5. **Rampa yanlış pozitifi**: planarlık testi standart sapmaya bakıyordu,
   koridor+duba dizilimi bunu geçip aracı duvara sürüyordu. Açısal kutu
   bazlı KESİNTİSİZLİK + tam aralık testine geçildi, sektör 35°->25°,
   ayrıca rampa sadece araç parkur yönündeyken (±45°) etkinleşir.

**48 kombinasyonluk ağırlık taraması** yapıldı; `w_duz` 0.5->0.1 seçildi.

### BİLİNEN SINIR (dürüst kayıt)
Dar koridorda (3.2-3.6 m) TAM ORTADA duran dubada araç geçemiyor: 1.2 m
genişliğindeki araç için geçiş penceresi 0.25-0.45 m kalıyor. Tarama
hiçbir ağırlık kombinasyonunda bunu çözmedi — AYAR sorunu değil, düz-ışın
ileri bakışının YAPISAL sınırı. Bu durumda düğüm `KAPALI` moduna geçip
HEDEF GÖNDERMİYOR (araç duruyor) — tehlikeli bir manevra yapmıyor.
4 m koridorda ve geniş alanda aynı dizilim SORUNSUZ geçiliyor.

### Canlı doğrulama (2026-09-06, araç Jetson)
Hedefler zararsız bir topic'e yönlendirilerek (hareket riski sıfır) gerçek
LIDAR ile çalıştırıldı: 1850-1870 geçerli nokta, `egim_on=0.34°`,
`egim_yan=0.4°` (araç düz — doğru), mod NORMAL'de kararlı, mod titremesi
ve çökme yok. Düğüm launch'a eklendi ama **VARSAYILAN PASİF**.

### AYRICA BULUNAN VE GİDERİLEN ARIZA
Test sırasında `/scan_raw` 10 Hz akarken `/scan`'in HİÇ yayınlamadığı
görüldü: `scan_front_filter` süreci %0.7 CPU ve tüm thread'leri
`futex_wait` durumunda KİLİTLENMİŞTİ (DDS eşleşmesi görünüyordu, veri
akmıyordu). Bu, Nav2 costmap'lerinin LIDAR'ı HİÇ görmemesi demekti.
Launch yeniden başlatıldı, `/scan` 10 Hz'e döndü. **Neden bulunamadı —
tekrarlarsa `/scan` hz kontrolü ilk bakılacak yerdir.**

## FREN SİSTEMİ (L298N, 2026-09-06)

Kullanıcı isteği: *"arduino unoya araçtakine l298n motor sürücü bağladım,
11 ve 10. pinlere bağlı. Yer istasyonundaki manuel/otonom geçişi butonu
fiziksel olan onu fren olarak değiştir; ona bastığımızda freni açsın,
tekrar bastığımızda kapatsın, 1 kere basınca 3 sn motoru çalıştırsın."*

### Araçtaki sürüş Arduino'su
Çalışan sketch **`~/Arduino/sketch_jul26a/sketch_jul26a.ino`** (araç
Jetson'da; git'te DEĞİL). Bu dosya artık arayüz tarafında da
`arac_arduino_surus/arac_arduino_surus.ino` olarak tutuluyor — kaynağın
tek kopyası araçta durmasın diye. Karışmaması gereken benzer dosyalar:
`~/Downloads/tufan_surus.ino` ve `~/Arduino/sketch_jul01a` BTS7960
sürümüdür (protokol `-255..255`), `sketch_jul09a`/`sketch_jul18a` eski
ESC denemeleridir. Çalışan sürüm PPM (1000-2000 µs) + far alanlıdır.

**PIN HARİTASI (çakışma yok — doğrulandı):**

| Pin | Kullanım |
|---|---|
| 5, 6 | Sol / Sağ ESC (Servo, PPM) |
| 2, 3 | Far röleleri (aktif-düşük) |
| **11, 10** | **FREN L298N — IN1 / IN2 (YENİ)** |

**KABLOLAMA VARSAYIMI:** 11→IN1, 10→IN2, ENA jumper ile 5V (hız sabit,
sadece yön kontrolü). Fren ters yöne çalışıyorsa sketch'teki
`FREN_AC_IN1` / `FREN_AC_IN2` değerlerini birbiriyle değiştirmek yeterli.

### Protokol
`"SOL_PPM,SAG_PPM,FAR,FREN\n"` — 3. ve 4. alan opsiyonel, eski formatlar
çalışmaya devam eder. FREN: 0=kapalı, 1=açık (durum, komut değil).

### Kritik tasarım kararları
- **3 saniyeyi ARDUINO sayar, Jetson değil.** Süre Jetson'da sayılsaydı,
  bağlantı o 3 saniye içinde koparsa fren YARIM kalırdı. Arduino'da
  sayılınca hareket her hâlükârda tamamlanır.
- **`delay(3000)` KULLANILMIYOR — bloklamayan `millis()` yapısı.** Bloklayan
  bir bekleme, o 3 saniye boyunca seri okumayı ve SÜRÜŞ WATCHDOG'UNU
  durdururdu; araç son hızıyla 3 saniye kontrolsüz devam ederdi.
- **Kenar tetikleme.** Motor sadece istenen durum DEĞİŞTİĞİNDE çalışır;
  Jetson saniyede 10 kez aynı değeri göndermesi tetiklemez.
- **Açılışta hareket YOK.** Arduino resetlenirse iki durum da 0 başlar,
  fark oluşmadığı için motor dönmez — fiziksel fren yerinde kalır.
- **Acil durdurma kilidi freni ENGELLEMEZ.** Kilit sadece palet
  komutlarını kesiyor; araç durdurulduktan sonra fren çekilebilmeli.
- **Komut arayüzden GEÇMEZ.** `kontrol_paneli_node` (ayrı process)
  `/fren_komut`'u doğrudan araca yayınlar — acil stop ile aynı desen,
  arayüz donsa/kapansa bile fren kumanda edilebilir.

### Değişen dosyalar
- `arac_arduino_surus/arac_arduino_surus.ino` (yeni; araçta
  `~/Arduino/sketch_jul26a/`) — fren pinleri + bloklamayan 3 sn mantığı
- araç `arduino_motor_kontrol.py` — `/fren_komut` aboneliği, `_fren_cb`,
  seri paket tek yerden üretilsin diye yeni `_seri_komut()`
- `kontrol_paneli_node.py` — `/fren_komut` yayıncısı, `fren_toggle()`;
  sol buton (D9) artık `node.fren_toggle`'a bağlı
- `main.py` — `tip == 'fren'` durumu loglanıyor; `_panel_mod_toggle`
  artık fiziksel butona BAĞLI DEĞİL (metot duruyor). **Sürüş modu
  değiştirme kaybolmadı** — ekrandaki MANUEL/OTONOM butonları çalışıyor.

### CANLI TESTTE BULUNAN HATA: tek basış → 6 tetikleme (2026-09-06)
Kullanıcı "fren çalışmıyor, belki ters gidiyor" dedi. Panel node logu asıl
nedeni gösterdi — **tek basış 6 kez tetiklenmişti**:
```
393.335 FREN -> KAPANIYOR      (aralıklar: 159, 160, 262,
393.494 FREN -> ACILIYOR        259, 220 ms)
393.654 FREN -> KAPANIYOR
393.916 FREN -> ACILIYOR   ...
```
Butonun kontak sıçraması, `kontrol_paneli_sistemi.py`'deki ~40 ms'lik
debounce penceresini (`DEBOUNCE_ORNEK=2`, 50 Hz) aşıyor. Her tetikleme
durumu ters çevirdiği için **lineer aktüatör sürekli yön değiştirip
hiçbir yere varamıyordu** — dışarıdan "çalışmıyor" gibi görünüyor.

**Çözüm debounce'u uzatmak DEĞİL** (farların/diğer butonların davranışını
da değiştirirdi ve 260 ms'lik sıçramayı yine yakalayamazdı). Doğru yer
KOMUT seviyesi — lineer motor 3 saniye zaten meşgul, o süre bitmeden gelen
basış fiziksel olarak uygulanamaz:
- **Yer istasyonu (asıl koruma):** `kontrol_paneli_node.FREN_KILIT_S = 3.5`
  — kilit süresi dolmadan gelen basış yok sayılır ve loglanır.
- **Arduino (yedek savunma):** hareket bitince `frenIstenen = frenUygulanan`
  — hareket sırasında gelen değişiklikler BİRİKMEZ. Yoksa 3 sn biter bitmez
  motor hemen ters yöne kalkıyordu. Komut başka bir kaynaktan gelse bile
  bu koruma geçerli.

### Doğrulama (2026-09-06)
- Sketch derlendi: 5594 bayt (%17 flash), 253 bayt RAM (%12)
- avrdude ile `/dev/ttyACM3`'e yüklendi ve **doğrulandı** (5594 bayt verified)
- Araç yığını yeniden derlenip başlatıldı; `/fren_komut` aboneliği canlı
  (Subscription count: 1)
- 4 alanlı seri paket, fren motorunu DÖNDÜRMEDEN doğrulandı: `/farlar`
  komutu artık aynı `_seri_komut()`'tan geçiyor ve `/rosout`'ta
  `💡 Farlar: AÇIK` görüldü
- **Fiziksel fren hareketi HENÜZ TEST EDİLMEDİ** — arayüz kapalı olduğu
  için buton basılamadı. Arayüz açılınca panel node yeni kodla başlar.

## SOL BAR: "ARACI BAŞLAT" İKONU (2026-09-06)

Kullanıcı isteği: *"sol tarafta aracı başlat butonunu daha güzel bir ikon
ile değiştir."*

Buton, eklendiğinde emoji + metin (`"🚀 ARACI\nBAŞLAT"`) kullanıyordu ve
sol bardaki diğer butonlardan görsel olarak ayrışıyordu — hepsi
**metinsiz, QPainter ile çizilmiş 64×64 QIcon** taşıyor (bkz.
`_pwm_izleme_ikonu_olustur`, `_yon_pid_ikonu_olustur`,
`_ates_ikonu_olustur`). Dosya bazlı ikon eklenmiyor çünkü `arayuz.py`
Qt Designer tarafından ÜRETİLİYOR ve elle düzenlenmiyor.

Yeni `_arac_baslat_ikonu_olustur()`: üstünde boşluk olan halka + dikey
çubuk (evrensel güç/başlatma simgesi). Halkadaki boşluğun tam tepede
olması için `drawArc(120°, 300°)` — geriye 60°..120° arası (tepe 90°)
boşluk kalır. İki geçişli çizim: önce yarı saydam kalın dış parıltı
(`alpha=70`, kalınlık 11), sonra asıl gövde (kalınlık 5).

**Renk `#00FF7B`** — projenin "olumlu/devam" rengi. Yanındaki bilgi
butonlarının camgöbeğinden (`#00E5FF`) ve ACİL DURDUR'un kırmızısından
KASITLI olarak ayrı, çünkü bu buton bir EYLEM başlatıyor.

Metin kalktığı için tooltip genişletildi (butonun ne yaptığı artık
sadece oradan okunuyor).

**Doğrulama:** ikon offscreen (`QT_QPA_PLATFORM=offscreen`) render edilip
sol bar zeminine benzer arka planda mevcut PWM butonunun ikonuyla yan yana
karşılaştırıldı — aynı görsel ağırlık, tutarlı stil.

## ACİL STOP → MANUEL (2026-09-06)

Kullanıcı bildirimi: *"araç otonomdayken yer istasyonundaki acil stop
çalışmıyor; acil stop switch'i manuele geçirsin ve aracı durdursun."*

### Teşhis: kilit ASLINDA çalışıyordu
Canlı ölçüm (`/arac_komut` yayınlayıp `/rosout` dinlenerek):
```
/arac_komut abone sayısı = 1
🛑 ACİL DURDURMA KİLİTLENDİ
kilitliyken araca giden PPM mesajı sayısı: 0     <-- kilit ÇALIŞIYOR
✅ Acil durdurma kilidi AÇILDI
```
Eksik olan şey: kilit **sadece motorlara giden komutu kesiyor**, otonomi
altta çalışmaya devam ediyordu — Nav2/goal_manager hedef üretmeyi
sürdürüyor ve switch bırakılır bırakılmaz araç kaldığı yerden devam
ediyordu. Operatör için bu "acil stop çalışmıyor" demek.

*(Not: ilk teşhis denemesi `ros2 topic pub --once` ile yapıldı ve mesaj
hiç ulaşmadı — `--once` keşif tamamlanmadan yayınlayıp çıkabiliyor.
Kalıcı yayıncı + `get_subscription_count()` beklemesiyle tekrarlandı.)*

### Çözüm
`kontrol_paneli_node.acil_stop_degisti` artık acil stopta ayrıca
**`/surus_modu` = MANUEL** yayınlıyor:
- **Araca DOĞRUDAN**: `surus_koprusu.py` otonom `cmd_vel`'i iletmeyi,
  `on_bosluk_nokta_atici.py` hedef üretmeyi anında keser. Arayüz donmuş
  ya da kapanmış olsa bile çalışır.
- **Arayüze bildirim** (`/panel_durumu` → `{'tip':'acil_stop_manuel'}`):
  `main.py` kendi modunu da MANUEL yapar. **Bu şart** — arayüz
  `/surus_modu`'nu HER TURDA tekrar yayınlıyor
  (`telemetri_sistemi.py::surekli_yayin_dongusu`), kendi durumunu
  değiştirmezse bir sonraki turda OTONOM'u geri yazar ve mod değişimi
  anında geri alınırdı.
- **Acil stop aktif kaldığı sürece her tick MANUEL tekrarlanır** —
  arayüz cevap vermiyorsa bile son sözü acil stop söyler. Bu tekrar,
  `tick()`'teki `if self.arac_modu != "MANUEL": return` erken dönüşünden
  ÖNCE yapılıyor; sonra konsaydı tam da gerektiği durumda (OTONOM'dayken)
  hiç çalışmazdı.
- **Bırakınca OTONOM'a KENDİLİĞİNDEN dönülmez.** Otonomiye devam etmek
  operatörün bilinçli kararı olmalı — arayüzden OTONOM'a basması gerekir.

### Ayrıca giderilen güvenlik açığı (kod incelemesinde bulundu)
`_fren_cb` kasıtlı olarak acil durdurma kilidinden muaf (araç
durdurulduktan sonra fren çekilebilmeli). Ama yazdığı paketin ilk iki
alanı `_son_sol_ppm`/`_son_sag_ppm` idi — yani **fren düğmesine basmak,
kilitli aracı acil stop anındaki SON HIZIYLA yeniden hareket
ettirebilirdi.** `_seri_komut()` artık kilitliyken PPM alanlarını merkeze
(dur) zorluyor; far/fren alanları çalışmaya devam ediyor.

### Doğrulama (offline, 3/3)
1. OTONOM'da acil stop → `EMERGENCY_STOP_CMD` + `/surus_modu=MANUEL` +
   arayüz bildirimi
2. OTONOM'dayken 3 tick → 3 kez MANUEL tekrarı (erken dönüşe takılmıyor)
3. Bırakınca → sadece `DEVAM_CMD`, `/surus_modu` yayını YOK (otonoma
   kendiliğinden dönmüyor)

## GPS KONUM DÜZELTMESİ KAPATILDI (2026-09-06)

Kullanıcı kararı: *"gpsi iptal edelim çok kayma yapıyor."*

### Ölçümle doğrulanan sebep
Araç **fiziksel olarak dururken** (PPM 1500/1500 nötr, `cmd_vel=0`)
`/gps_katki_durumu` şunu diyordu:
```
GPS katkısı: RTK FLOAT ~5cm/sn ile düzeltiliyor (kalan fark 1034cm)
```
Yani düzeltici, **10.34 metre** uzaktaki bir GPS konumuna doğru pozu
saniyede 5 cm çekiyordu — 30 saniyede ~1.5 m hayalet hareket. RTK **FIXED**
değil **FLOAT** çözüm alındığı için GPS konumunun kendisi gezingen;
düzeltici o gezinmeyi odometriye taşıyordu.

Bu, daha önce teşhis edilen zincirin kök nedeniydi: odometri sıçraması →
LIDAR taraması costmap'e her karede başka yere damgalanıyor → costmap
%40 dolu → **Nav2 "GridBased: failed to create plan"** → `cmd_vel=0` →
araç hiç hareket etmiyor.

### Yapılan
`tufan_mppi.launch.py`'de `konum_birlestirici`'ye parametre override:

| Parametre | Eski | Yeni | Etki |
|---|---|---|---|
| `gps_correct_min_fix_type` | 5 | **99** | Konum düzeltmesi KAPALI |
| `gps_heading_min_fix_type` | 3 | **99** | GPS yön (heading) KAPALI |
| `gps_anchor_min_fix_type` | 5 | **5** | Anchor AÇIK (kasıtlı) |

Eşikler `fix_type` ile karşılaştırılıyor (`konum_birlestirici.py` satır
405/433/460: `if msg.fix_type < esik: return`), bu yüzden 99 = ulaşılamaz
= ilgili işlev hiç çalışmaz. Temiz kapatma, yan etkisi yok.

**Anchor kasıtlı olarak açık bırakıldı:** GPS başlangıç noktası kurulmaya
devam ediyor, böylece `gps_hedef_donusturucu.py`'nin GPS hedefi → yerel
hedef dönüşümü ÇALIŞMAYA DEVAM EDİYOR. Kapatılan tek şey, GPS'in
odometriyi SÜRÜKLEMESİ.

### Ölçüm: önce / sonra (araç dururken, 30 saniye)

| | ÖNCE | SONRA |
|---|---|---|
| `/odom` kat edilen yol | **4.08 m** | **0.00 m** |
| en büyük tek adım | 0.036 m | 0.000 m |
| 10 cm üstü sıçrama | 0 | 0 |
| `/odom_rf2o` | 3.68 m | 0.14 m |
| GPS durumu | RTK FLOAT ile düzeltiliyor | **GPS katkısı YOK** |

### Bilinen ödünç
GPS düzeltmesi olmadan odometri UZUN sürüşlerde birikimli hata yapar
(kapalı çevrim düzeltme yok). Bu bilinçli bir tercih: sabit bir kayma,
saniyede 5 cm'lik rastgele çekişten çok daha iyi. Ayrıca
`on_bosluk_nokta_atici.py` zaten kaymaya BAĞIŞIK tasarlandı — hedefi her
tick canlı LIDAR'dan yeniden üretiyor, hedefin ömrü saniyenin altında.

**Geri açmak için:** launch'taki iki 99 değerini 5 ve 3 yapın. Yeniden
derlemeden denemek için:
`ros2 param set /konum_birlestirici gps_correct_min_fix_type 5`

## KAYAR ENGEL AŞAMASI (6. tabela, 2026-09-06)

Kullanıcı isteği: *"parkura kayar engel ekleyeceğim; ön kamerada 6.
tabelayı tespit ettiğinde aracın kayar engel aşamasına geldiğini anlaması
gerekiyor, araç engelden kaçıp geri dönmemeli, kayar engel açıldığında
devam etmeli."*

### Akış
`tabela_etap_yoneticisi.py` 'Six' tabelasını görünce
**`/kayar_engel_etabi` (Bool) = True** yayınlar; etap 7+ görülünce False.
Durum 1 Hz ile PERİYODİK tekrarlanır — `on_bosluk_nokta_atici.py`
sonradan başlarsa tek seferlik bir yayını kaçırır ve engelden kaçmaya
çalışırdı (bu projede aynı ders `/surus_modu` ve panel bağlantı
durumunda da yaşandı).

### Davranış (`on_bosluk_nokta_atici.py` → `KAYAR_ENGEL` modu)
Bayrak açıkken fan taraması **hiç yapılmaz** — yanlarda boşluk olsa bile
oraya hedef atılmaz ("engelden kaçıp geri dönmemeli"). Sadece dar bir ön
koniye (±25°) bakılır:
- **Kapalı** (< 3.0 m): `KAYAR_ENGEL_BEKLIYOR` — aracın **mevcut konumu
  hedef olarak yayınlanır**, goal_manager bunu "ulaşıldı" sayıp yumuşakça
  durur. *Hiçbir hedef yayınlamamak YETMEZ* — Nav2 o durumda eski
  hedefine gitmeye devam eder, yani aracı kapıya sürerdi. Konum
  3 saniyede bir tekrar sabitlenir (Nav2 kurtarma davranışlarına karşı).
- **Açık** (≥ 3.0 m): düz ileri kısa hedef, geçiş.

**RAMPA modu bu aşamada çalıştırılmaz.** Kapalı bir kapı da "gövdeye dik
düz yüzey" testini geçerdi ve araç kapıya SÜRÜLÜRDÜ — offline testte
doğrulandı: aynı yüzey normalde `RAMPA_YAKLASMA` verirken bayrak
açıkken `KAYAR_ENGEL_BEKLIYOR` veriyor.

### Offline testte bulunan hata
Serbest mesafe normalde hedef ufkunda (2.0+0.6 = **2.6 m**) kırpılıyor;
açılma eşiği **3.0 m** olduğu için ölçülen değer eşiği HİÇBİR ZAMAN
aşamazdı — kapı tamamen açık olsa bile araç sonsuza kadar beklerdi.
Bu ölçüm için kırpma eşiğin üstüne taşındı (`tavan_ust`). Rampa geçiş
testinde birebir aynı hata yaşanmıştı.

### CANLI ÖLÇÜMDE BULUNAN SAHA SORUNU: sahte tabela tespitleri
Kamera parkurda değilken bile model, `GUVEN_ESIGI`'ni (0.6) aşan sahte
tespitler üretti — tek bir test sırasında **Etap 8 (0.77 / 0.80 / 0.71),
Etap 2 (0.76), Etap 1 (0.60)**. Kayar engel kararını tek kareye bağlamak
tehlikeliydi: sahte bir "7+" tespiti aşamayı anında iptal ediyordu (ilk
testte tam olarak bu oldu, bayrak True'ya hiç oturmadı).

Çözüm: kayar engel kararı için **ardışık doğrulama**
(`KAYAR_ENGEL_DOGRULAMA_ADEDI = 3`, Stop tabelasındaki desenin aynısı).
`/guncel_etap` yayını DEĞİŞMEDİ — o zaten "risksiz, bilgi amaçlı".

**Doğrulama sonrası:** `/kayar_engel_etabi` test boyunca **6/6 mesajda
True** kaldı, sahte tespitler aşamayı iptal edemedi.

> **NOT (sahada dikkat):** sahte tespitler modelin kendi sorunu; kayar
> engel kararı artık korunuyor ama `/guncel_etap` ve Stop davranışı hâlâ
> aynı modele dayanıyor. Gerekirse `GUVEN_ESIGI` yükseltilmeli.

### SAHADA BULUNAN TASARIM HATASI: "6'yı okuyor ama hareket etmiyor"
Kullanıcı bildirimi. Canlı ölçüm (araç gerçek kayar engelin önünde):
```
mod = KAYAR_ENGEL_BEKLIYOR   on_serbest = 2.05m   acik_esik = 3.0m
LIDAR:  ±10° koni -> en yakın 4.75m, ortanca 9.11m   <-- TAM ÖN AÇIK
        ±25° koni -> en yakın 1.85m                  <-- kenarda bir şey var
```
Geçidin geometrisi ayrıca ölçüldü — **araç zaten geçebiliyordu**:
```
DÜZ giderken süpürülen koridor (yarı genişlik 0.85m) -> ilk engel 3.64m
En iyi açı: -7 derece -> 5.25m
```
İlk sürüm koni içindeki **en kısa** mesafeye bakıyordu ("koninin tamamı
boş mu?"). Kayar engel için bu YANLIŞ test: kapı çerçevesi/direği ya da
yana kaymış panel koninin kenarına girdiği anda, önündeki geçit
metrelerce açık olsa bile "kapalı" deniyordu.

**Doğru test "koni boş mu" değil, "ARAÇ BU GENİŞLİKTE GEÇEBİLİYOR MU":**
her aday açıda araç genişliğinde bir koridor süpürülür ve **en iyisi**
(en uzağa giden) alınır. "Etrafından dolaşma" koruması bozulmaz — açılar
±25° ile sınırlı, yani araç ancak geçidi HİZALAMAK için küçük bir
düzeltme yapabilir, engeli dolaşamaz.

Ek olarak **düz gitmek varsayılan** yapıldı (`kayar_engel_hizalama_payi_m
= 0.3`): bir aday ancak belirgin ölçüde daha iyiyse seçilir. Yoksa her
şey eşit olduğunda (önü tamamen boş sahne) döngüye ilk giren açı (-25°)
kazanıp aracı gereksiz yere yana kırdırıyordu — offline testte görüldü.

**Doğrulama:** düzeltme sonrası, aracı hareket ettirmeden çalıştırılan
gölge bir düğüm örneği AYNI gerçek sahnede `mod = KAYAR_ENGEL` (geçiş)
raporladı — artık `KAYAR_ENGEL_BEKLIYOR` değil.

### Offline test (6/6 geçti)
kapı kapalı→bekle · **yanda geniş geçit varken bile bekle (kaçmıyor)** ·
etap kapalıyken normal davranış · kapı açık→düz geç · rampa
tetiklenmiyor · etap kapanınca aynı yüzey yine rampa (regresyon).

---
*Bu doküman, `~/Desktop/tufan` altındaki kodun mevcut haline göre otomatik
olarak (kod incelemesiyle) hazırlanmıştır.*
