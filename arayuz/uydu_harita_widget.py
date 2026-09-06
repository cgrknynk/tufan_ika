"""UyduHaritaWidget: ArduPilot/Mission Planner tarzi uydu goruntulu harita.

Esri World Imagery (ArcGIS REST tile servisi, API key gerektirmez) kullanarak
gercek uydu goruntusu ceker, arac konumunu (GNSS /fix -> enlem/boylam) ve
yonunu bu goruntunun uzerinde ok olarak gosterir. Standart "slippy map"
(Web Mercator, OSM/Google ile ayni projeksiyon) tile matematigi kullanir.

Tile'lar hem bellekte hem diskte (~/.cache/tufan_harita_tiles/) onbelleklenir;
agdan sadece eksik/hic gorulmemis tile'lar cekilir.

Kullanim:
    widget = UyduHaritaWidget()
    widget.konum_guncelle(enlem, boylam, yon_derece)   # GNSS /fix geldikce cagir
"""
import math
import os

from PyQt5.QtCore import Qt, QPointF, QRectF, QUrl, QTimer
from PyQt5.QtGui import QPainter, QPixmap, QColor, QPen, QPolygonF, QFont
from PyQt5.QtWidgets import QWidget
from PyQt5.QtNetwork import QNetworkAccessManager, QNetworkRequest

TILE_BOYUTU = 256
TILE_URL_SABLON = "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}"
ONBELLEK_DIZINI = os.path.expanduser("~/.cache/tufan_harita_tiles")


def enlem_boylam_to_tile(lat, lon, zoom):
    """Web Mercator (OSM/Google ile ayni) -- kesirli tile koordinati dondurur."""
    lat_rad = math.radians(lat)
    n = 2.0 ** zoom
    x = (lon + 180.0) / 360.0 * n
    y = (1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n
    return x, y


class UyduHaritaWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(300, 300)

        self.zoom = 18
        self.min_zoom = 3
        self.max_zoom = 20

        self.enlem = None
        self.boylam = None
        self.yon_derece = 0.0
        self.iz_noktalari = []   # [(lat, lon), ...] kat edilen yol

        # RTK DURUMU (2026-09-01, kullanıcı isteği: "gpsin hatası ve fix mi
        # float mı olduğu durumu yazsın") - GPSRAW'dan (fix_type, h_acc).
        self.rtk_fix_type = None
        self.rtk_h_acc_m = None

        # IMU ve LiDAR - kullanicinin istegi uzerine, taktik lidar ekranindaki
        # gibi bu ekranda da (uydu goruntusu uzerinde, gercek konuma gore
        # bindirilmis olarak) gorunsun diye eklendi.
        self.roll = 0.0
        self.pitch = 0.0
        self.lidar_noktalari = []   # [(ileri_m, sol_m), ...] arac govde cercevesinde

        self.tile_onbellek = {}          # (z,x,y) -> QPixmap
        self.beklemede = set()           # (z,x,y) agdan istendi, cevap bekleniyor
        self.basarisiz = set()           # (z,x,y) 404/hata -- tekrar denemeyi engelle

        self.net_yoneticisi = QNetworkAccessManager(self)
        self.net_yoneticisi.finished.connect(self._tile_indi)

        os.makedirs(ONBELLEK_DIZINI, exist_ok=True)

        # RENDER İSTEĞİ COALESCING (2026-09-05) - harita_sistemi.py'deki
        # TaktikRadarEkrani'nde AYNI kalıpla /odom'un 59Hz throttle'sız
        # update()'inin GUI thread'i kilitlediği kanıtlandı (bkz. o
        # dosyadaki 2026-09-05 notu). Bu widget da konum_guncelle (GNSS),
        # rtk_durum_guncelle, imu_guncelle, lidar_guncelle sinyallerinde
        # AYNI koşulsuz self.update() desenini kullanıyordu - önlem olarak
        # (özellikle Navigasyon sayfasına GEÇİLDİĞİNDE, o ana kadar
        # BİRİKMİŞ tüm bekleyen sinyallerin art arda tetiklediği paintEvent
        # zincirinin GUI thread'i kilitlemesini önlemek için) buraya da
        # uygulandı - veri her zaman anında güncellenir, self.update()
        # çağrısı en fazla ~30fps'e (33ms) sınırlanır.
        self._render_timer = QTimer(self)
        self._render_timer.setSingleShot(True)
        self._render_timer.timeout.connect(self._render_zamanlayici_calisti)
        self._render_istegi_beklemede = False

    def _render_iste(self):
        if not self.isVisible():
            return
        if not self._render_istegi_beklemede:
            self._render_istegi_beklemede = True
            self._render_timer.start(33)

    def _render_zamanlayici_calisti(self):
        self._render_istegi_beklemede = False
        self.update()

    # ------------------------------------------------------------------ #
    # Disaridan konum guncellemesi (GNSS /fix geldikce cagrilir)
    # ------------------------------------------------------------------ #
    def konum_guncelle(self, enlem, boylam, yon_derece=None):
        self.enlem = enlem
        self.boylam = boylam
        if yon_derece is not None:
            self.yon_derece = yon_derece
        if not self.iz_noktalari or self._mesafe_m(self.iz_noktalari[-1], (enlem, boylam)) > 1.0:
            self.iz_noktalari.append((enlem, boylam))
            if len(self.iz_noktalari) > 2000:
                self.iz_noktalari.pop(0)
        # PERFORMANS: veri her zaman guncel (sekme degisince hemen dogru
        # gorunur) ama pahali cizim/tile-yukleme SADECE bu widget gercekten
        # ekranda gorunurken tetiklenir - gorunmuyorken surekli ag istegi +
        # QPainter cizimi bos yere CPU/ag harciyordu. DÜZELTME (2026-09-05):
        # self.update() DOĞRUDAN değil, _render_iste() (coalescing) üzerinden.
        self._render_iste()

    # GPS_FIX_TYPE (MAVLink enum, GPSRAW.fix_type) -> okunabilir etiket.
    _RTK_ETIKETLERI = {
        0: ("GPS YOK", "#ff4444"),
        1: ("FIX YOK", "#ff4444"),
        2: ("2D FIX", "#ffb454"),
        3: ("3D FIX", "#ffb454"),
        4: ("DGPS", "#5eead4"),
        5: ("RTK FLOAT", "#00d4ff"),
        6: ("RTK FIXED", "#00ff88"),
    }

    def rtk_durum_guncelle(self, fix_type, h_acc_m):
        self.rtk_fix_type = fix_type
        self.rtk_h_acc_m = h_acc_m
        self._render_iste()

    def imu_guncelle(self, roll, pitch):
        self.roll = roll
        self.pitch = pitch
        self._render_iste()

    def lidar_guncelle(self, noktalar):
        self.lidar_noktalari = noktalar
        self._render_iste()

    @staticmethod
    def _mesafe_m(p1, p2):
        # Kaba enlem/boylam-fark tabanli mesafe (iz noktasi seyreltme icin yeterli hassasiyette)
        dlat = (p2[0] - p1[0]) * 111320.0
        dlon = (p2[1] - p1[1]) * 111320.0 * math.cos(math.radians(p1[0]))
        return math.hypot(dlat, dlon)

    # ------------------------------------------------------------------ #
    # Fare tekerlegi ile yakinlastir/uzaklastir (ArduPilot GCS'lerdeki gibi)
    # ------------------------------------------------------------------ #
    def wheelEvent(self, event):
        delta = 1 if event.angleDelta().y() > 0 else -1
        self.zoom = max(self.min_zoom, min(self.max_zoom, self.zoom + delta))
        # DÜZELTME (2026-09-02, kullanıcı isteği: "çok fazla yakınlaştıramıyorum") -
        # eskiden HER yakınlaştırma adımında TÜM önbellek temizleniyordu -
        # bu, aşağıdaki "üst zoom seviyesinden kırpılmış geçici görüntü"
        # (bkz. paintEvent/_en_yakin_atalik_tile) mekanizmasını da imkansız
        # kılıyordu, çünkü tam da lazım olduğu anda (yeni zoom'un GERÇEK
        # karoları henüz inmemişken) bir önceki zoom'un karoları da SİLİNMİŞ
        # oluyordu - ekran görüntüsüz/gri kalıyor, "daha fazla yakınlaşamıyorum"
        # hissi buradan geliyordu (Esri'nin o bölge için üst zoom'da görüntüsü
        # olmasa bile karo TALEBİ engellenmiyordu, sadece GÖSTERİLECEK hiçbir
        # şey yoktu). Artık önbellek KORUNUYOR (farklı zoom'lar farklı
        # anahtarlarda (z,x,y) tutulduğu için çakışma yok) - her zoom
        # seviyesinin karoları bir kere iner, bir daha ağdan istenmez.
        self.update()

    # ------------------------------------------------------------------ #
    # Tile getirme (onbellek: bellek -> disk -> ag)
    # ------------------------------------------------------------------ #
    def _tile_iste(self, z, x, y):
        anahtar = (z, x, y)
        if anahtar in self.tile_onbellek or anahtar in self.beklemede or anahtar in self.basarisiz:
            return
        n = 2 ** z
        if x < 0 or x >= n or y < 0 or y >= n:
            return

        disk_yolu = os.path.join(ONBELLEK_DIZINI, str(z), str(x), f"{y}.jpg")
        if os.path.exists(disk_yolu):
            pix = QPixmap(disk_yolu)
            if not pix.isNull():
                if self._yer_tutucu_mu(pix):
                    # Eskiden (bu düzeltmeden ÖNCE) diske YANLIŞLIKLA
                    # kaydedilmiş bir yer tutucu olabilir - temizle ki bir
                    # daha hiç yüklenmesin (bkz. _yer_tutucu_mu notu).
                    try:
                        os.remove(disk_yolu)
                    except OSError:
                        pass
                    self.basarisiz.add(anahtar)
                    return
                self.tile_onbellek[anahtar] = pix
                return

        self.beklemede.add(anahtar)
        url = TILE_URL_SABLON.format(z=z, x=x, y=y)
        istek = QNetworkRequest(QUrl(url))
        istek.setAttribute(QNetworkRequest.User, anahtar)
        self.net_yoneticisi.get(istek)

    def _en_yakin_atalik_tile(self, z, x, y):
        """DÜZELTME (2026-09-02, kullanıcı isteği: "çok fazla
        yakınlaştıramıyorum") - hedef (z,x,y) karosu henüz inmemiş/hiç
        yoksa (Esri'nin o bölgede o zoom seviyesinde görüntüsü olmayabilir -
        uydu görüntü sağlayıcılarının HİÇBİRİ her yerde en yüksek zoom'da
        veri sunmaz), standart "slippy map" davranışı (Google Maps/Leaflet
        vb.) gibi bir ÜST (daha düşük zoom, daha önce zaten önbelleğe
        alınmış) karonun İLGİLİ KÖŞESİNİ kırpıp büyüterek gösterir - boş
        gri kare yerine BULANIK ama GERÇEK bir görüntü, kesintisiz/akıcı
        yakınlaştırma hissi verir. min_zoom'a kadar yukarı doğru dener.
        Döndürür: (pixmap, kaynak_QRectF) ya da hiçbiri onbellekte yoksa None.
        """
        for k in range(1, z - self.min_zoom + 1):
            ust_z = z - k
            olcek_2k = 2 ** k
            ust_x, ust_y = x // olcek_2k, y // olcek_2k
            ust_pix = self.tile_onbellek.get((ust_z, ust_x, ust_y))
            if ust_pix is None:
                continue
            alt_boyut = TILE_BOYUTU / olcek_2k
            kaynak_x = (x % olcek_2k) * alt_boyut
            kaynak_y = (y % olcek_2k) * alt_boyut
            return ust_pix, QRectF(kaynak_x, kaynak_y, alt_boyut, alt_boyut)
        return None

    def _tile_indi(self, reply):
        anahtar = reply.request().attribute(QNetworkRequest.User)
        self.beklemede.discard(anahtar)
        if reply.error() != reply.NoError:
            self.basarisiz.add(anahtar)
            reply.deleteLater()
            return

        veri = reply.readAll()
        pix = QPixmap()
        if pix.loadFromData(veri) and not pix.isNull():
            if self._yer_tutucu_mu(pix):
                # DÜZELTME (2026-09-02, kullanıcı isteği: "yakınlaştırma
                # yaptıkça map data not yet available diyor") - canlı
                # doğrulandı: Esri, kapsama dışı zoom seviyelerinde HTTP
                # HATASI DEĞİL, GEÇERLİ bir JPEG (200 OK) ile düz gri +
                # "Map data not yet available" yazan bir YER TUTUCU
                # döndürüyor - eskiden bu GERÇEK görüntüymüş gibi
                # önbelleğe alınıp DİSKE de yazılıyordu (bir kez görülünce
                # SONSUZA KADAR gösterilmeye devam ederdi). Gerçek uydu
                # görüntüsü pratikte HİÇBİR ZAMAN böyle tekdüze değildir -
                # örnek piksellerin çoğu (>%90) TEK bir renge yakınsa yer
                # tutucu sayılır, `basarisiz` olarak işaretlenir - bu da
                # `_en_yakin_atalik_tile` (üst zoom'dan bulanık ama GERÇEK
                # görüntü) yedeklemesini otomatik devreye sokar. Gerçek
                # örnek verilerle (bu konumda z=19 gerçek görüntü ~%50,
                # z=20 yer tutucu ~%98 baskın-renk oranı) doğrulandı.
                self.basarisiz.add(anahtar)
                reply.deleteLater()
                return
            self.tile_onbellek[anahtar] = pix
            z, x, y = anahtar
            dizin = os.path.join(ONBELLEK_DIZINI, str(z), str(x))
            os.makedirs(dizin, exist_ok=True)
            with open(os.path.join(dizin, f"{y}.jpg"), "wb") as f:
                f.write(bytes(veri))
            self.update()
        else:
            self.basarisiz.add(anahtar)
        reply.deleteLater()

    def _yer_tutucu_mu(self, pix):
        """bkz. _tile_indi'deki not - Esri'nin 200 OK ile döndürdüğü sahte
        'Map data not yet available' karosunu, GERÇEK bir uydu görüntüsü
        pratikte hiçbir zaman olmayacak kadar TEKDÜZE (tek renge yakın)
        olmasından tanır. 16px aralıklı bir örnekleme ızgarası alınır, en
        baskın renge (±10 tolerans) YAKIN örneklerin oranı hesaplanır."""
        img = pix.toImage()
        genislik, yukseklik = img.width(), img.height()
        if genislik < 16 or yukseklik < 16:
            return False
        adim = 16
        ornekler = [
            img.pixelColor(xx, yy).getRgb()[:3]
            for yy in range(0, yukseklik, adim)
            for xx in range(0, genislik, adim)
        ]
        if len(ornekler) < 4:
            return False

        def yakinlik_sayisi(baz, tol=10):
            return sum(
                1 for r, g, b in ornekler
                if abs(r - baz[0]) <= tol and abs(g - baz[1]) <= tol and abs(b - baz[2]) <= tol
            )

        en_baskin_oran = max(yakinlik_sayisi(o) for o in ornekler) / len(ornekler)
        return en_baskin_oran >= 0.90

    # ------------------------------------------------------------------ #
    # Cizim
    # ------------------------------------------------------------------ #
    def paintEvent(self, event):
        # DÜZELTME (2026-09-05, kullanıcı: "uygulama dondu"/segfault -
        # faulthandler ile CANLI YAKALANDI: bu fonksiyonda HİÇ `ressam.
        # end()` YOKTU - erken `return` (GNSS henüz yokken) VEYA aşağıdaki
        # tile/renk analizi kodlarından biri hata fırlatırsa (ör. bozuk
        # bir JPEG dosyası) QPainter'ın kendisi HİÇ KAPATILMADAN
        # fonksiyondan çıkılıyordu - "QBackingStore::endPaint() called
        # with active painter" uyarısı defalarca CANLI görüldü, bunun
        # hemen ardından GERÇEK bir segfault oluştu (crash_log.txt'de tam
        # yığın izi var). Artık try/finally ile HANGİ YOLDAN çıkılırsa
        # çıkılsın (erken return, normal bitiş, ya da bir istisna) ressam.
        # end() GARANTİ ediliyor.
        ressam = QPainter(self)
        try:
            ressam.setRenderHint(QPainter.Antialiasing)
            ressam.fillRect(self.rect(), QColor("#0d1117"))

            if self.enlem is None or self.boylam is None:
                ressam.setPen(QColor("#8b98a5"))
                ressam.setFont(QFont("Arial", 13, QFont.Bold))
                ressam.drawText(self.rect(), Qt.AlignCenter,
                                 "GNSS konumu bekleniyor...\n(Here4 baglanip /fix yayinlamaya baslayinca harita gorunecek)")
                return
            self._paintEvent_govde(ressam)
        finally:
            ressam.end()

    def _paintEvent_govde(self, ressam):

        genislik, yukseklik = self.width(), self.height()
        merkez_x_px, merkez_y_px = genislik / 2.0, yukseklik / 2.0

        merkez_tile_x, merkez_tile_y = enlem_boylam_to_tile(self.enlem, self.boylam, self.zoom)

        ilk_tile_x = int(math.floor(merkez_tile_x - (merkez_x_px / TILE_BOYUTU)))
        ilk_tile_y = int(math.floor(merkez_tile_y - (merkez_y_px / TILE_BOYUTU)))
        son_tile_x = int(math.floor(merkez_tile_x + ((genislik - merkez_x_px) / TILE_BOYUTU)))
        son_tile_y = int(math.floor(merkez_tile_y + ((yukseklik - merkez_y_px) / TILE_BOYUTU)))

        for tx in range(ilk_tile_x, son_tile_x + 1):
            for ty in range(ilk_tile_y, son_tile_y + 1):
                px = merkez_x_px + (tx - merkez_tile_x) * TILE_BOYUTU
                py = merkez_y_px + (ty - merkez_tile_y) * TILE_BOYUTU
                anahtar = (self.zoom, tx, ty)
                pix = self.tile_onbellek.get(anahtar)
                hedef_dikdortgen = QRectF(px, py, TILE_BOYUTU, TILE_BOYUTU)
                if pix is not None:
                    ressam.drawPixmap(QPointF(px, py), pix)
                else:
                    # Gerçek karo henüz inmedi/yok - bkz. _en_yakin_atalik_tile
                    # notu: bulanık ama GERÇEK bir görüntü (üst zoom'dan
                    # kırpılmış) boş gri kareden HER ZAMAN daha iyi.
                    yedek = self._en_yakin_atalik_tile(self.zoom, tx, ty)
                    if yedek is not None:
                        ust_pix, kaynak = yedek
                        ressam.drawPixmap(hedef_dikdortgen, ust_pix, kaynak)
                    else:
                        ressam.fillRect(hedef_dikdortgen, QColor("#1a2029"))
                    self._tile_iste(*anahtar)

        # --- Kat edilen yol (iz) ---
        if len(self.iz_noktalari) > 1:
            ressam.setPen(QPen(QColor(0, 200, 255, 200), 3))
            poly = QPolygonF()
            for lat, lon in self.iz_noktalari:
                tx, ty = enlem_boylam_to_tile(lat, lon, self.zoom)
                poly.append(QPointF(
                    merkez_x_px + (tx - merkez_tile_x) * TILE_BOYUTU,
                    merkez_y_px + (ty - merkez_tile_y) * TILE_BOYUTU))
            ressam.drawPolyline(poly)

        # --- LiDAR nokta bulutu (gercek metre olcegiyle, araç govde
        # cercevesinden (ileri,sol) uydu goruntusu uzerine bindiriliyor).
        # Ayni donusum arac okunun kullandigi self.yon_derece'yi kullanir ki
        # ikisi gorsel olarak TUTARLI kalsin (bkz. asagidaki ok cizimi).
        if self.lidar_noktalari:
            metre_basina_piksel = (2 ** self.zoom) / (156543.03392 * math.cos(math.radians(self.enlem)))
            theta = math.radians(self.yon_derece)
            cos_t, sin_t = math.cos(theta), math.sin(theta)
            ressam.setBrush(QColor(255, 0, 51, 180))
            ressam.setPen(Qt.NoPen)
            for ileri_m, sol_m in self.lidar_noktalari:
                # Once govde-cercevesi yerel piksel ofseti (TaktikRadarEkrani
                # ile AYNI esleme: ileri=-y, sol=-x), sonra yon_derece kadar
                # ekran uzerinde (kuzey-yukari referansla) donduruluyor.
                yerel_dx = -sol_m * metre_basina_piksel
                yerel_dy = -ileri_m * metre_basina_piksel
                dx = yerel_dx * cos_t - yerel_dy * sin_t
                dy = yerel_dx * sin_t + yerel_dy * cos_t
                ressam.drawEllipse(QPointF(merkez_x_px + dx, merkez_y_px + dy), 3, 3)

        # --- Arac oku (merkezde, yon_derece'ye gore donuk) ---
        ressam.save()
        ressam.translate(merkez_x_px, merkez_y_px)
        ressam.rotate(self.yon_derece)
        ok = QPolygonF([QPointF(0, -16), QPointF(10, 12), QPointF(0, 6), QPointF(-10, 12)])
        ressam.setBrush(QColor("#00d4ff"))
        ressam.setPen(QPen(QColor("#ffffff"), 1.5))
        ressam.drawPolygon(ok)
        ressam.restore()

        # --- Bilgi metni ---
        ressam.setPen(QColor("#ffffff"))
        ressam.setFont(QFont("Arial", 9, QFont.Bold))
        ressam.drawText(10, 18, f"Zoom: {self.zoom}  |  Lat: {self.enlem:.6f}  Lon: {self.boylam:.6f}")
        ressam.drawText(10, 34, f"IMU: Roll {self.roll:+.1f}°  Pitch {self.pitch:+.1f}°  |  Lidar: {len(self.lidar_noktalari)} nokta")

        # RTK DURUMU (2026-09-01, kullanıcı isteği): "fix mi float mı" + hata.
        if self.rtk_fix_type is not None:
            etiket, renk = self._RTK_ETIKETLERI.get(self.rtk_fix_type, (f"BİLİNMEYEN({self.rtk_fix_type})", "#ff4444"))
            hata_str = f"±{self.rtk_h_acc_m:.2f}m" if (self.rtk_h_acc_m is not None and self.rtk_h_acc_m >= 0) else "hata: bilinmiyor"
            ressam.setPen(QColor(renk))
            ressam.drawText(10, 50, f"GPS: {etiket}  |  {hata_str}")
        else:
            ressam.setPen(QColor("#8b98a5"))
            ressam.drawText(10, 50, "GPS: veri yok")

        ressam.setPen(QColor("#ffffff"))
        ressam.drawText(10, yukseklik - 8, "Uydu goruntusu: Esri World Imagery")
