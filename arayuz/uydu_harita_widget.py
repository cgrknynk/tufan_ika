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

from PyQt5.QtCore import Qt, QPointF, QRectF, QUrl
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

        self.tile_onbellek = {}          # (z,x,y) -> QPixmap
        self.beklemede = set()           # (z,x,y) agdan istendi, cevap bekleniyor
        self.basarisiz = set()           # (z,x,y) 404/hata -- tekrar denemeyi engelle

        self.net_yoneticisi = QNetworkAccessManager(self)
        self.net_yoneticisi.finished.connect(self._tile_indi)

        os.makedirs(ONBELLEK_DIZINI, exist_ok=True)

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
        self.update()

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
        self.tile_onbellek.clear()
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
                self.tile_onbellek[anahtar] = pix
                return

        self.beklemede.add(anahtar)
        url = TILE_URL_SABLON.format(z=z, x=x, y=y)
        istek = QNetworkRequest(QUrl(url))
        istek.setAttribute(QNetworkRequest.User, anahtar)
        self.net_yoneticisi.get(istek)

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

    # ------------------------------------------------------------------ #
    # Cizim
    # ------------------------------------------------------------------ #
    def paintEvent(self, event):
        ressam = QPainter(self)
        ressam.setRenderHint(QPainter.Antialiasing)
        ressam.fillRect(self.rect(), QColor("#0d1117"))

        if self.enlem is None or self.boylam is None:
            ressam.setPen(QColor("#8b98a5"))
            ressam.setFont(QFont("Arial", 13, QFont.Bold))
            ressam.drawText(self.rect(), Qt.AlignCenter,
                             "GNSS konumu bekleniyor...\n(Here4 baglanip /fix yayinlamaya baslayinca harita gorunecek)")
            return

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
                if pix is not None:
                    ressam.drawPixmap(QPointF(px, py), pix)
                else:
                    ressam.fillRect(QRectF(px, py, TILE_BOYUTU, TILE_BOYUTU), QColor("#1a2029"))
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
        ressam.drawText(10, yukseklik - 8, "Uydu goruntusu: Esri World Imagery")
