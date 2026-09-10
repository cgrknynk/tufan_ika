#!/usr/bin/env python3
"""Tabela (trafik levhasi) algilama ROS 2 dugumu.

tabela_yolo26/jetson_server_2kamerali.py'nin tabela-tespit kismindan
tasindi. Ayri bir kameradan (turret'in kamerasindan bagimsiz) surekli
YOLO tespiti yapar; en yuksek guvenli tespiti /tabela_tespit topic'ine
yayinlar ve isaretlenmis kareyi UDP ile bir goruntuleyiciye gonderir.

Bu dugum herhangi bir motor/tetikleyici kontrol etmez - sadece algilama
yapar; surus mantigina baglamak istenirse /tabela_tespit dinlenerek
ayri bir karar dugumunde (bkz. tabela_etap_yoneticisi.py) yapilmalidir.

MODEL AC/KAPA (2026-08-31, gorev sekansi): /tabela_model_aktif (Bool)
dinler - KAPALI iken kamera ACIK KALIR (cap.read() calismaya devam eder,
UDP goruntu akisi kesilmez) ama YOLO predict() cagrilmaz ve /tabela_tespit'e
yayin YAPILMAZ. Gorev sekansinda Stop tabelasindan sonra "kamerayi kapatma,
sadece modeli durdur" ihtiyacini karsilar (bkz. tabela_etap_yoneticisi.py).
"""

import os
import socket
import threading
import time

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Bool, String
from ultralytics import YOLO

# Varsayilan model yolu: launch dosyasi olmadan (ör. "ros2 run tufan_v2_ws
# tabela_node.py") dogrudan calistirildiginda da model_path bos kalmasin diye.
# tabela_ana.engine (TensorRT, tabela_yolo26/okubeni.txt'de belirtilen
# "kullanilacak model") - .pt'den DAHA HIZLI cikarim icin (2026-08-31,
# kullanici istegi). .engine onceden derlenmis oldugu icin .to('cuda')
# GEREKMEZ (asagida kosullu atlaniyor).
_VARSAYILAN_MODEL_YOLU = os.path.join(
    get_package_share_directory('tufan_v2_ws'), 'models', 'tabela_ana.engine'
)


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

class TabelaNode(Node):
    def __init__(self):
        super().__init__('tabela_node')

        # camera_index SADECE otomatik bulma (asagida) BASARISIZ olursa
        # kullanilir (yedek) - bkz. _kamera_indexini_otomatik_bul.
        self.declare_parameter('camera_index', 2)
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
        # ON KAMERA: YOLO tabela modeli calisiyor - yuksek fps.
        self.declare_parameter('kamera_fps', 15)
        # GORSEL OLARAK KESIN DOGRULANAN fiziksel USB port yolu (ON kamera,
        # 2026-09-01): iki C270'ten de birer kare cekilip GOZLE incelendi -
        # bu port (o zamanki /dev/video0) kisi/kapi/DUBA (trafik konisi) +
        # aracin kendi paleti gorunen GERCEK on-kamera goruntusu verdi,
        # digeri (o zamanki /dev/video2, port "1-2.2.3") sadece bir masa/
        # mobilya gorundu (arka kamera). ONCEKI deger ('1-2.2.3') bu
        # dogrulamadan ONCE, harici bir testte YANLISLIKLA tersine
        # cevrilmisti - GORSEL kanitla DUZELTILDI. Port numarasi USB
        # yeniden-enumerasyonunda son basamakta kayabiliyor (orn
        # "1-2.2.4.3"->"1-2.2.4.4") - HUB SEVIYESI kismi kullanildi, alt-port
        # basamagi HARIC tutuldu (bu, arka kameranin PORTUYLA cakismadigi
        # surece guvenli - bkz. usb_port_ipucu eslesme mantigi asagida, TAM
        # segment yerine .startswith kullanilarak degistirildi, bkz.
        # _kamera_indexini_otomatik_bul).
        # IKINCI GORSEL DOGRULAMA (2026-09-01, gece, farlarla): USB portlari
        # TEKRAR kaymisti (1-2.2.4/1-2.2.3 -> 1-2.2.1/1-2.2.4.3) - port
        # numaralandirmasinin bu donanimda TAM KARARLI OLMADIGI dogrulandi
        # (muhtemelen hub/kablo yeniden-baglanmasi). Agac/bina goren kare
        # (kullanici tarafindan ON olarak dogrulandi) artik 1-2.2.1 portunda.
        self.declare_parameter('kamera_usb_port', '1-2.3')
        self.declare_parameter('model_path', _VARSAYILAN_MODEL_YOLU)
        self.declare_parameter('hedef_confidence', 0.5)
        self.declare_parameter('imgsz', 640)
        self.declare_parameter('video_target_ip', '10.40.64.44')
        self.declare_parameter('video_target_port', 5001)
        # 2026-09-01, kullanici istegi: "kalitesinden ziyade hizi onemli" -
        # UDP'ye giden goruntu bu ikisiyle kucultulur/dusuk kalitede encode
        # edilir. YOLO'nun ALDIGI kare BUNDAN ETKILENMEZ (tespit hala tam
        # cozunurlukte/imgsz'de calisir) - SADECE gonderim oncesi resize
        # edilir, boylece hiz kazanci tespit KALITESINI dusurmez.
        self.declare_parameter('yayin_genislik', 480)
        self.declare_parameter('jpeg_kalite', 40)

        self._camera_index = self.get_parameter('camera_index').value
        self._kamera_arama_ismi = self.get_parameter('kamera_arama_ismi').value
        self._kamera_fps = int(self.get_parameter('kamera_fps').value)
        self._kamera_usb_port = self.get_parameter('kamera_usb_port').value
        self._model_path = self.get_parameter('model_path').value
        self._conf = self.get_parameter('hedef_confidence').value
        self._imgsz = self.get_parameter('imgsz').value
        self._video_target_ip = self.get_parameter('video_target_ip').value
        self._video_target_port = self.get_parameter('video_target_port').value
        self._yayin_genislik = self.get_parameter('yayin_genislik').value
        self._jpeg_kalite = self.get_parameter('jpeg_kalite').value

        self._tespit_pub = self.create_publisher(String, 'tabela_tespit', 10)
        self._sock_video = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.get_logger().info(f'YOLO modeli yukleniyor: {self._model_path}')
        self._model = YOLO(self._model_path, task='detect')
        if not str(self._model_path).endswith(('.engine', '.onnx')):
            self._model.to('cuda')

        self._model_aktif = True
        self.create_subscription(Bool, '/tabela_model_aktif', self._model_aktif_cb, 10)

        self._calisiyor = True
        threading.Thread(target=self._ana_dongu, daemon=True).start()

        self.get_logger().info(
            'tabela_node aktif: tabela_tespit yayinlaniyor, goruntu UDP ile '
            f'{self._video_target_ip}:{self._video_target_port} adresine gonderiliyor'
        )

    def _kucult_ve_gonder(self, frame):
        """Yayin oncesi frame'i _yayin_genislik'e olcekler (en-boy orani
        korunur) ve dusuk JPEG kalitesiyle UDP ile gonderir - hiz oncelikli
        (bkz. __init__ yorumu). YOLO/tespit BU FONKSIYONDAN ONCE, TAM
        cozunurlukte zaten tamamlanmis olur, buradan ETKILENMEZ."""
        h, w = frame.shape[:2]
        if w > self._yayin_genislik:
            oran = self._yayin_genislik / float(w)
            frame = cv2.resize(frame, (self._yayin_genislik, int(h * oran)),
                                interpolation=cv2.INTER_LINEAR)
        ret_enc, buffer = cv2.imencode(
            '.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_kalite])
        if not ret_enc:
            return
        data = buffer.tobytes()
        if len(data) < 65000:
            try:
                self._sock_video.sendto(data, (self._video_target_ip, self._video_target_port))
            except Exception:
                pass

    def _model_aktif_cb(self, msg: Bool):
        yeni = bool(msg.data)
        if yeni != self._model_aktif:
            self.get_logger().info(f'tabela modeli {"AKTIF" if yeni else "DURDURULDU (kamera acik kalir)"}')
        self._model_aktif = yeni

    def _ana_dongu(self):
        kaynak, bulunan_index = None, None
        if self._kamera_arama_ismi:
            kaynak, bulunan_index, aciklama = _kamera_kaynagi_bul(
                self._kamera_arama_ismi, self._kamera_usb_port)
            if kaynak is not None:
                self.get_logger().info(
                    f'Tabela kamerasi kimligi: "{self._kamera_arama_ismi}" -> '
                    f'{kaynak}  [{aciklama}]')
                self._camera_index = bulunan_index
            else:
                self.get_logger().warn(
                    f'"{self._kamera_arama_ismi}" (port={self._kamera_usb_port}) '
                    f'icin kararli kimlik BULUNAMADI ({aciklama}); camera_index '
                    f'parametresi (={self._camera_index}) YEDEK olarak kullaniliyor.')
        if kaynak is None:
            kaynak, bulunan_index = self._camera_index, self._camera_index

        # BENZERSIZ KIMLIKLE ACMA (bkz. _kamera_kaynagi_bul / _kamerayi_ac).
        cap = _kamerayi_ac(kaynak, bulunan_index, self.get_logger(),
                           'Tabela kamerasi',
                           dogrulama_ismi=self._kamera_arama_ismi)
        # DUZELTME (2026-09-01, canli bulundu): ayni USB hub'i paylasan UC
        # kamerayla (turret C922 + arka C270) birlikte HAM YUYV formatinda
        # calisinca USB izokron bant genisligi tukeniyordu - cap.read()
        # sessizce SUREKLI False donuyordu (hic loglanmiyordu), VIDIOC_
        # STREAMON dogrudan test edilince "No space left on device"
        # (klasik UVC bant genisligi hatasi, disk ile alakasi yok) verdigi
        # dogrulandi. MJPG (sikistirilmis) format bant genisligini cok
        # dusurur - digerlerine dokunmadan SADECE bu kamerayi MJPG'ye
        # gecirmek yeterli oldu.
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        # FPS, BUFFERSIZE'dan ONCE ve boyutlardan SONRA ayarlanir -
        # canli dogrulanan sira budur (bkz. kamera_fps notu).
        cap.set(cv2.CAP_PROP_FPS, self._kamera_fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self._kamera_kaynagi = kaynak

        # Arka arkaya bu kadar basarisiz okuma = kamera fiilen olmus
        # (bkz. _kamerayi_yeniden_ac). ~15fps'te 45 kare ≈ 3 saniye:
        # gecici bir dalgalanmayi tetiklemeyecek kadar uzun, operatorun
        # "goruntu yok" demesinden once toparlanacak kadar kisa.
        OKUMA_HATASI_ESIGI = 45
        okuma_hatasi = 0

        try:
            while self._calisiyor and rclpy.ok():
                ret, frame = cap.read()
                if not ret:
                    # KENDINI TOPARLAMA (2026-09-09): tek basarisiz okuma
                    # normaldir (kare gecikmesi), ama ARKA ARKAYA cok
                    # olursa kamera fiilen olmustur - bkz.
                    # _kamerayi_yeniden_ac. Sayaç ilk basarili karede
                    # sifirlanir, yani gecici tek hatalar birikmez.
                    okuma_hatasi += 1
                    if okuma_hatasi >= OKUMA_HATASI_ESIGI:
                        cap = _kamerayi_yeniden_ac(
                            cap, self._kamera_arama_ismi, self._kamera_usb_port,
                            self.get_logger(), 'Tabela kamerasi',
                            ayarlar=[(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG')),
                          (cv2.CAP_PROP_FRAME_WIDTH, 640),
                          (cv2.CAP_PROP_FRAME_HEIGHT, 480),
                          (cv2.CAP_PROP_FPS, self._kamera_fps),
                          (cv2.CAP_PROP_BUFFERSIZE, 1)])
                        okuma_hatasi = 0
                        time.sleep(0.5)
                    time.sleep(0.1)
                    continue
                okuma_hatasi = 0

                if not self._model_aktif:
                    # Model KAPALI: sadece ham kareyi UDP'ye gonder, YOLO
                    # calistirma / /tabela_tespit'e yayin yok - kamera
                    # yine de ACIK/canli kalsin diye.
                    self._kucult_ve_gonder(frame)
                    continue

                try:
                    results = self._model.predict(
                        source=frame, imgsz=self._imgsz, conf=self._conf, half=True, verbose=False
                    )
                    result = results[0]
                    cizili_frame = result.plot()

                    if len(result.boxes) > 0:
                        # En yuksek guvenli tespiti bildir
                        confs = result.boxes.conf.cpu().numpy()
                        en_iyi_idx = int(confs.argmax())
                        cls_id = int(result.boxes.cls[en_iyi_idx].item())
                        cls_adi = self._model.names.get(cls_id, str(cls_id))
                        guven = float(confs[en_iyi_idx])
                        self._tespit_pub.publish(String(data=f'{cls_adi}:{guven:.2f}'))
                    else:
                        self._tespit_pub.publish(String(data='YOK'))

                    self._kucult_ve_gonder(cizili_frame)

                except Exception as e:
                    self.get_logger().error(f'Kare isleme hatasi, atlaniyor: {e}')
                    time.sleep(0.2)

        finally:
            cap.release()

    def destroy_node(self):
        self._calisiyor = False
        self._sock_video.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TabelaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
