#!/usr/bin/env python3
"""Arka kamera - SADECE acar ve goruntuyu UDP ile yayinlar, HICBIR YOLO
modeli/tespit CALISTIRMAZ (2026-08-31, kullanici istegi: "sadece ac ve
goruntu yayinla"). tabela_node.py/turret_node.py'nin kamera-acma kismiyla
ayni deseni kullanir, model kismini hic icermez.
"""
import os
import socket
import time

import cv2
import rclpy
from rclpy.node import Node


# Test edilebilirlik icin kok dizinler SABIT degil (testler sahte bir
# /sys + /dev agaci gosterebilsin diye) - uretimde ASLA degistirilmez.
_V4L_SYS_KOK = '/sys/class/video4linux'
_V4L_DEV_KOK = '/dev/v4l'
_DEV_KOK = '/dev'


def _kamera_adaylarini_topla(arama_metni):
    """Isminde arama_metni GECEN GERCEK video-capture dugumlerini toplar.
    Doner: [(index, usb_port, seri_no), ...] - index'e gore sirali.

    (/sys/.../videoN/index=="0" filtresi sart: UVC kameralar ayni fiziksel
    cihaz icin birden fazla dugum sunar, digerleri BOS-FORMAT metadata
    dugumleridir ve acilinca kare vermezler.)"""
    import glob
    adaylar = []
    for isim_dosyasi in sorted(glob.glob(_V4L_SYS_KOK + '/video*/name')):
        try:
            with open(isim_dosyasi) as f:
                isim = f.read().strip()
        except OSError:
            continue
        if arama_metni and arama_metni.lower() not in isim.lower():
            continue
        video_dizin = os.path.dirname(isim_dosyasi)
        try:
            with open(os.path.join(video_dizin, 'index')) as f:
                if f.read().strip() != '0':
                    continue
            index = int(os.path.basename(video_dizin).replace('video', ''))
        except (OSError, ValueError):
            continue
        usb_port, seri_no = None, None
        try:
            # .../videoN/device -> ".../1-2.2.4:1.0" (arayuz); onun UST
            # dizini USB CIHAZ dizinidir ve 'serial' dosyasi oradadir.
            arayuz_dizini = os.path.realpath(os.path.join(video_dizin, 'device'))
            usb_cihaz_dizini = os.path.dirname(arayuz_dizini)
            usb_port = os.path.basename(usb_cihaz_dizini)
            try:
                with open(os.path.join(usb_cihaz_dizini, 'serial')) as f:
                    seri_no = f.read().strip() or None
            except OSError:
                seri_no = None
        except OSError:
            pass
        adaylar.append((index, usb_port, seri_no))
    return adaylar


def _kararli_yolu_bul(index, dizin):
    """/dev/v4l/by-id (veya by-path) altinda /dev/video<index>'e COZULEN
    sembolik baglantiyi dondurur. Yoksa None.

    Baglanti ISMINI tahmin etmek yerine hepsini COZUP karsilastiriyoruz -
    isim formati udev surumune/cihaza gore degisiyor, cozumleme ise her
    zaman kesin."""
    import glob
    hedef = os.path.realpath(_DEV_KOK + '/video%d' % index)
    for baglanti in sorted(glob.glob(dizin + '/*')):
        try:
            if os.path.realpath(baglanti) == hedef:
                return baglanti
        except OSError:
            continue
    return None


def _kamera_kaynagi_bul(arama_metni, usb_port_ipucu=None):
    """Kamerayi BENZERSIZ KIMLIKLE acmak icin kaynak uretir (2026-09-09,
    kullanici istegi: "kodlar calistiginda id'ye gore veya benzersiz
    kimlige gore acilsin").

    Doner: (kaynak, index, aciklama)
      kaynak : cv2.VideoCapture'a verilecek deger - KARARLI YOL (str)
               ya da yedek /dev/videoN index'i (int). Bulunamazsa None.
      index  : bulunan /dev/videoN numarasi (yol acilamazsa YEDEK olarak
               kullanilir - bkz. cagiran taraftaki iki asamali acma).
      aciklama: loga yazilacak, hangi kimligin secildigini soyleyen metin.

    SIRALAMA - en benzersizden en kirilgana:
      1) /dev/v4l/by-id/...  : SADECE seri numarasi sistemde TEKSE. Iki
         ayni model C270'in seri numarasi Logitech tarafindan PAYLASILMIS
         (sahte) oldugu icin by-id o durumda ikisini de gosterebilir -
         bu yuzden once TEKLIK dogrulanir, korukorune kullanilmaz.
      2) /dev/v4l/by-path/... : fiziksel USB port yolu. Seri numarasi
         paylasilmis olsa bile PORT benzersizdir; kablo ayni porta takili
         kaldigi surece USB kesif sirasindan BAGIMSIZ sabittir.
      3) /dev/videoN index'i  : son care (eski davranis).

    NEDEN INDEX YETMIYORDU: /dev/videoN numaralari USB yeniden-
    enumerasyonunda degisiyor; ustelik "bul sonra ac" arasinda numara
    kayarsa YANLIS kamera aciliyordu (sahada goruldu: tabela modeli arka
    kameraya baglanmisti). Kararli yol bu yaristan da kurtariyor."""
    adaylar = _kamera_adaylarini_topla(arama_metni)
    if not adaylar:
        return None, None, 'aday yok'

    secilen = None
    if usb_port_ipucu:
        # TAM esitlik VEYA ipucunun bir "ust dal" (hub seviyesi) olmasi -
        # USB yeniden-enumerasyonunda port yolunun son basamagi kayabiliyor.
        for aday in adaylar:
            _, usb_port, _ = aday
            if usb_port == usb_port_ipucu or (
                    usb_port and usb_port.startswith(usb_port_ipucu + '.')):
                secilen = aday
                break
    if secilen is None:
        # PORT IPUCU TUTMADI (ya da hic verilmedi). Tek aday varsa zaten
        # belirsizlik yok - port ipucu KAYMIS olsa bile dogru kamerayi
        # aciyoruz (eskiden burada None donup index yedegine dusuluyordu,
        # yani kayma = kamera acilmiyor demekti).
        if len(adaylar) == 1:
            secilen = adaylar[0]
        else:
            return None, None, ('%d aday var, port ipucu (%s) hicbirine '
                                'uymadi - ayirt edilemiyor'
                                % (len(adaylar), usb_port_ipucu))

    index, usb_port, seri_no = secilen

    # 1) Seri numarasi TUM v4l cihazlari arasinda TEK mi?
    if seri_no:
        tum_seriler = [s for _, _, s in _kamera_adaylarini_topla('')]
        if tum_seriler.count(seri_no) == 1:
            yol = _kararli_yolu_bul(index, _V4L_DEV_KOK + '/by-id')
            if yol:
                return yol, index, 'by-id (seri no TEK: %s)' % seri_no

    # 2) Fiziksel USB port yolu
    yol = _kararli_yolu_bul(index, _V4L_DEV_KOK + '/by-path')
    if yol:
        neden = 'seri no paylasilmis/yok' if seri_no else 'seri no yok'
        return yol, index, 'by-path (%s, USB port=%s)' % (neden, usb_port)

    # 3) Son care
    return index, index, '/dev/video%d index (kararli yol bulunamadi)' % index


def _index_ismi_uyuyor_mu(index, arama_metni):
    """Yedek /dev/videoN index'inin GERCEKTEN aranan kamera olup olmadigini
    dogrular (2026-09-09, CANLI olarak yakalandi).

    NEDEN SART: yeniden acilista USB portlari komple kaymisti - arka kamera
    icin yedek olarak duran camera_index=4, o an C922'ye (SILAH kamerasi)
    denk geliyordu. Dogrulama olmadan arka kamera silah kamerasini acmaya
    calisirdi: en iyi ihtimalle "Device or resource busy", en kotusunde
    silah kamerasini calardi. Isim tutmuyorsa kamera ACILMAZ - yanlis
    kamerayi acmaktansa hic acmamak."""
    if not arama_metni:
        return True
    try:
        with open('%s/video%d/name' % (_V4L_SYS_KOK, int(index))) as f:
            isim = f.read().strip()
        with open('%s/video%d/index' % (_V4L_SYS_KOK, int(index))) as f:
            if f.read().strip() != '0':
                return False
    except (OSError, ValueError):
        return False
    return arama_metni.lower() in isim.lower()


def _kamerayi_ac(kaynak, yedek_index, logger, etiket, ayarlar=None,
                 dogrulama_ismi=None):
    """Kararli yolla acar; olmazsa /dev/videoN index'ine DUSER.

    Iki asamali olmasinin sebebi: OpenCV surumune/backend'e gore string
    yol ile acma her ortamda calismayabiliyor - yol basarisiz olursa
    kamera HIC acilmamis olmasindansa eski (index) yontemiyle acilsin."""
    import cv2

    def _uygula(cap):
        for ozellik, deger in (ayarlar or []):
            try:
                cap.set(ozellik, deger)
            except Exception:
                pass

    if isinstance(kaynak, str):
        cap = cv2.VideoCapture(kaynak, cv2.CAP_V4L2)
        _uygula(cap)
        if cap.isOpened():
            logger.info('%s kararli yol ile acildi: %s' % (etiket, kaynak))
            return cap
        try:
            cap.release()
        except Exception:
            pass
        logger.warn('%s kararli yol ile ACILAMADI (%s), /dev/video%s '
                    'index yedegine dusuluyor.'
                    % (etiket, kaynak, yedek_index))
        kaynak = yedek_index

    # YEDEK INDEX DOGRULAMASI (bkz. _index_ismi_uyuyor_mu): index BASKA
    # bir kamerayi gosteriyorsa ACMA - bos (acilmamis) VideoCapture don,
    # cagiran taraflarin hepsi zaten isOpened()/read() basarisizligini
    # tolere ediyor.
    if not _index_ismi_uyuyor_mu(kaynak, dogrulama_ismi):
        logger.error(
            '%s ACILMADI: yedek index /dev/video%s ARANAN kamera degil '
            '("%s" ile eslesmiyor) - yanlis kamerayi acmamak icin '
            'vazgecildi.' % (etiket, kaynak, dogrulama_ismi))
        return cv2.VideoCapture()

    cap = cv2.VideoCapture(int(kaynak), cv2.CAP_V4L2)
    _uygula(cap)
    if not cap.isOpened():
        logger.error('%s ACILAMADI (index=%s)' % (etiket, kaynak))
    return cap



def _kamerayi_yeniden_ac(cap, arama_metni, usb_port_ipucu, logger, etiket,
                         ayarlar=None):
    """cap.read() SUREKLI False donuyorsa kamerayi kapatip YENIDEN acar.

    NEDEN (2026-09-09, canli olcumle bulundu): USB izokron bant genisligi
    yarisini kaybeden bir UVC kamera acilir, isOpened() TRUE doner, ama
    cap.read() SONSUZA KADAR False dondurur - OpenCV bunu HIC loglamaz.
    Silah kamerasi bu yuzden "acildi" yazip tek kare gondermedi (ag
    payi olculdu: 19 KB/s = sifir). Sureç olmedigi icin launch respawn'i
    da devreye girmiyordu. Cozum: kareyi okuyamayan dugum kamerayi
    KENDISI birakip yeniden acar - bu arada diger kamera(lar) bant
    genisligini serbest birakmis olabilir ve acilis basarili olur."""
    try:
        cap.release()
    except Exception:
        pass
    kaynak, bulunan_index, aciklama = _kamera_kaynagi_bul(
        arama_metni, usb_port_ipucu)
    if kaynak is None:
        logger.error('%s yeniden acilamadi: kararli kimlik yok (%s)'
                     % (etiket, aciklama))
        kaynak = bulunan_index = -1
    logger.warn('%s YENIDEN ACILIYOR (kare okunamiyor) -> %s [%s]'
                % (etiket, kaynak, aciklama))
    return _kamerayi_ac(kaynak, bulunan_index, logger, etiket,
                        ayarlar=ayarlar, dogrulama_ismi=arama_metni)

class ArkaKameraNode(Node):
    def __init__(self):
        super().__init__('arka_kamera_node')

        # camera_index SADECE otomatik bulma (asagida) BASARISIZ olursa
        # kullanilir (yedek) - bkz. _kamera_indexini_otomatik_bul.
        self.declare_parameter('camera_index', 4)
        self.declare_parameter('kamera_arama_ismi', 'C270')
        # KAMERA FPS SINIRI (2026-09-10) - UC KAMERANIN AYNI ANDA
        # CALISABILMESI ICIN SART. Cekirdek kaniti (dmesg):
        #   usb 1-2.2.3: Not enough bandwidth for new device state.
        #   usb 1-2.2.3: Not enough bandwidth for altsetting 4
        # UVC izokron bant rezervasyonu KARE HIZI ile orantili. 15 fps'te
        # ucuncu kamera altsetting bulamiyor ve cap.read() SESSIZCE False
        # donuyordu (kamera "acildi" gorunur, tek kare gelmez). Cozunurluk
        # DUSURMEK ISE YARAMADI - 320x240, hatta 160x120 bile ayni hatayi
        # verdi; degisken cozunurluk degil FPS'ti. 10 fps'te ucu de
        # 640x480 ile calisiyor (canli olculdu: 9.9 / 9.9 / 9.9 fps).
        # ARTIRMADAN ONCE ucunu birden test et.
        # ARKA KAMERA: model YOK, sadece izleme - DUSUK fps.
        self.declare_parameter('kamera_fps', 5)
        # GORSEL OLARAK KESIN DOGRULANAN fiziksel USB port yolu (ARKA
        # kamera) - bkz. tabela_node.py'deki AYNI tarihli not. tabela'nin
        # kullandigi C270'ten FARKLI port. Kablo baska bir porta tasinirsa
        # GUNCELLENMELI - bu donanimda port numaralandirmasi TAM KARARLI
        # DEGIL (2026-09-01 gece: 1-2.2.3 -> 1-2.2.4 kaydigi GORSEL olarak
        # tekrar dogrulandi, bkz. tabela_node.py'deki ayni tarihli not).
        #
        # 2026-09-09 GUNCELLENDI (araç yeniden acilisinda CANLI olcum):
        # portlar komple kaydi - iki C270 artik 1-2.1 ve 1-2.2.1'de, C922
        # ise 1-2.2.3'te. tabela_node'un ZATEN dogru olan (kullanici
        # gorsel olarak onaylamisti) portu 1-2.2.1 oldugu icin DIGER C270
        # = arka kamera = 1-2.1. Tahmin DEGIL, ON kameranin dogrulanmis
        # konfigurasyonundan cikarildi.
        self.declare_parameter('kamera_usb_port', '1-2.2.3')
        self.declare_parameter('video_target_ip', '10.40.64.44')
        self.declare_parameter('video_target_port', 5002)  # tabela(5000)/turret(5001) ile cakismasin

        self._camera_index = self.get_parameter('camera_index').value
        self._kamera_arama_ismi = self.get_parameter('kamera_arama_ismi').value
        self._kamera_fps = int(self.get_parameter('kamera_fps').value)
        self._kamera_usb_port = self.get_parameter('kamera_usb_port').value
        self._video_target_ip = self.get_parameter('video_target_ip').value
        self._video_target_port = self.get_parameter('video_target_port').value

        self._sock_video = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._calisiyor = True

        # BENZERSIZ KIMLIKLE ACMA (bkz. _kamera_kaynagi_bul): once
        # /dev/v4l/by-id (seri no TEKSE), sonra /dev/v4l/by-path (fiziksel
        # USB portu), en son /dev/videoN index'i.
        kaynak, bulunan_index = None, None
        if self._kamera_arama_ismi:
            kaynak, bulunan_index, aciklama = _kamera_kaynagi_bul(
                self._kamera_arama_ismi, self._kamera_usb_port)
            if kaynak is not None:
                self.get_logger().info(
                    f'Arka kamera kimligi: "{self._kamera_arama_ismi}" -> '
                    f'{kaynak}  [{aciklama}]')
                self._camera_index = bulunan_index
            else:
                self.get_logger().warn(
                    f'"{self._kamera_arama_ismi}" (port={self._kamera_usb_port}) '
                    f'icin kararli kimlik BULUNAMADI ({aciklama}); '
                    f'camera_index parametresi (={self._camera_index}) YEDEK '
                    'olarak kullaniliyor.')
        if kaynak is None:
            kaynak, bulunan_index = self._camera_index, self._camera_index

        cap = _kamerayi_ac(kaynak, bulunan_index, self.get_logger(),
                           'Arka kamera',
                           dogrulama_ismi=self._kamera_arama_ismi)
        # DUZELTME (2026-09-01, canli bulundu): turret (C922) + bu kamera
        # (C270) AYNI USB hub'ini HAM YUYV formatinda paylasinca, UCUNCU
        # kamera (tabela) icin USB izokron bant genisligi HIC kalmiyordu -
        # tabela MJPG'ye gecirilse bile No space left on device (klasik
        # UVC bant genisligi hatasi) aliyordu; arka_kamera_node durdurulup
        # dogrudan ffmpeg ile izole test edilince (turret tek basinayken
        # tabela BASARILI acildi) kesin kanitlandi. Bu kamera SADECE
        # izleme icin (model yok, dusuk oncelikli) - MJPG'ye gecirmek en
        # dusuk riskli secim, tabela icin gereken bant genisligini acar.
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        # FPS, BUFFERSIZE'dan ONCE ve boyutlardan SONRA ayarlanir -
        # canli dogrulanan sira budur (bkz. kamera_fps notu).
        cap.set(cv2.CAP_PROP_FPS, self._kamera_fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self._cap = cap
        self._kamera_kaynagi = kaynak
        # KENDINI TOPARLAMA sayaci (bkz. _kamerayi_yeniden_ac). Bu dugum
        # timer tabanli oldugu icin sayac NESNE uzerinde tutulur.
        self._okuma_hatasi = 0
        self._OKUMA_HATASI_ESIGI = 45   # 15fps'te ~3 saniye

        self._timer = self.create_timer(1.0 / 15.0, self._kare_gonder)

        self.get_logger().info(
            f'arka_kamera_node aktif: kamera={self._kamera_kaynagi}, '
            f'goruntu UDP ile {self._video_target_ip}:{self._video_target_port} adresine gonderiliyor '
            '(model/tespit YOK, sadece izleme)'
        )

    def _kare_gonder(self):
        if not self._calisiyor:
            return
        ret, frame = self._cap.read()
        if not ret:
            self._okuma_hatasi += 1
            if self._okuma_hatasi >= self._OKUMA_HATASI_ESIGI:
                self._cap = _kamerayi_yeniden_ac(
                    self._cap, self._kamera_arama_ismi, self._kamera_usb_port,
                    self.get_logger(), 'Arka kamera',
                    ayarlar=[(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG')),
                             (cv2.CAP_PROP_FRAME_WIDTH, 640),
                             (cv2.CAP_PROP_FRAME_HEIGHT, 480),
                          (cv2.CAP_PROP_FPS, self._kamera_fps),
                             (cv2.CAP_PROP_BUFFERSIZE, 1)])
                self._okuma_hatasi = 0
            return
        self._okuma_hatasi = 0
        ret_enc, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
        if not ret_enc:
            return
        data = buffer.tobytes()
        if len(data) < 65000:
            try:
                self._sock_video.sendto(data, (self._video_target_ip, self._video_target_port))
            except Exception:
                pass

    def destroy_node(self):
        self._calisiyor = False
        try:
            self._cap.release()
        except Exception:
            pass
        self._sock_video.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArkaKameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
