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


def _kamera_indexini_otomatik_bul(arama_metni, usb_port_ipucu=None):
    """CANLI TESTTE BULUNDU (2026-09-01): sabit camera_index kirilgan -
    /dev/videoN numaralari USB yeniden-enumerasyonunda (guc kesilip
    verilmesi, farkli takma sirasi) DEGISEBILIYOR, bu yuzden tabela
    modeli YANLISLIKLA arka kameraya baglanmisti (kullanici geri
    bildirimi). turret_node.py'nin AYNI deseni (isimle bulma) burada
    GENISLETILDI: isminde arama_metni (kucuk/buyuk harf duyarsiz, orn.
    "C270") GECEN /dev/videoN GERCEK video-capture dugumlerini
    (/sys/.../videoN/index=="0" - digerleri ayni fiziksel kameranin
    BOS-FORMAT metadata dugumleridir) bulur. AYNI MODELDEN BIRDEN FAZLA
    kamera varsa (iki C270 gibi, ISIM ILE AYIRT EDILEMEZLER - hatta
    ID_SERIAL bile Logitech tarafindan PAYLASILMIS/sahte) usb_port_ipucu
    (fiziksel USB port yolunun TAM son segmenti, orn "1-2.2.3") ile
    filtrelenir - kablo AYNI FIZIKSEL PORTA takili kaldigi surece bu
    deger USB KESIF SIRASINDAN BAGIMSIZ SABIT kalir (index numarasinin
    aksine). Eslesme yoksa None doner (cagiran taraf camera_index
    parametresine geri duser)."""
    import glob
    adaylar = []  # (index, usb_port_adi)
    for isim_dosyasi in sorted(glob.glob('/sys/class/video4linux/video*/name')):
        try:
            with open(isim_dosyasi) as f:
                isim = f.read().strip()
        except OSError:
            continue
        if arama_metni.lower() not in isim.lower():
            continue
        video_dizin = os.path.dirname(isim_dosyasi)
        try:
            with open(os.path.join(video_dizin, 'index')) as f:
                if f.read().strip() != '0':
                    continue  # metadata dugumu, gercek capture DEGIL
            index = int(os.path.basename(video_dizin).replace('video', ''))
        except (OSError, ValueError):
            continue
        usb_port = None
        if usb_port_ipucu:
            try:
                device_link = os.path.realpath(os.path.join(video_dizin, 'device'))
                usb_port = os.path.basename(os.path.dirname(device_link))
            except OSError:
                pass
        adaylar.append((index, usb_port))

    if not adaylar:
        return None
    if usb_port_ipucu:
        # TAM esitlik VEYA usb_port_ipucu'nun bir "ust dal" (hub seviyesi)
        # olmasi: USB yeniden-enumerasyonunda port yolunun SON basamagi
        # (orn "1-2.2.4.3" -> "1-2.2.4.4") kayabildigi CANLI TESTTE
        # gozlemlendi - bu yuzden ipucu "1-2.2.4" gibi kisa/hub-seviyesi
        # verilirse ".3"/".4" gibi alt-dallarin HEPSI kabul edilir. Farkli
        # bir hub kolunu (orn "1-2.2.40") YANLISLIKLA eslestirmemek icin
        # SADECE tam-segment sinirinda ('.' sonrasi) devam KABUL edilir.
        for index, usb_port in adaylar:
            if usb_port == usb_port_ipucu or (
                    usb_port and usb_port.startswith(usb_port_ipucu + '.')):
                return index
        return None  # port ipucu verildi ama eslesen YOK - yanlis kameraya baglanmaktansa vazgec
    return min(index for index, _ in adaylar)


class TabelaNode(Node):
    def __init__(self):
        super().__init__('tabela_node')

        # camera_index SADECE otomatik bulma (asagida) BASARISIZ olursa
        # kullanilir (yedek) - bkz. _kamera_indexini_otomatik_bul.
        self.declare_parameter('camera_index', 2)
        self.declare_parameter('kamera_arama_ismi', 'C270')
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
        self.declare_parameter('kamera_usb_port', '1-2.2.1')
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
        if self._kamera_arama_ismi:
            otomatik_index = _kamera_indexini_otomatik_bul(
                self._kamera_arama_ismi, self._kamera_usb_port)
            if otomatik_index is not None:
                if otomatik_index != self._camera_index:
                    self.get_logger().info(
                        f'Kamera otomatik bulundu: "{self._kamera_arama_ismi}" '
                        f'(port={self._kamera_usb_port}) -> /dev/video{otomatik_index} '
                        f'(parametre camera_index={self._camera_index} yerine kullanildi).')
                self._camera_index = otomatik_index
            else:
                self.get_logger().warn(
                    f'"{self._kamera_arama_ismi}" (port={self._kamera_usb_port}) isminde '
                    f'kamera bulunamadi, camera_index parametresi (={self._camera_index}) '
                    'YEDEK olarak kullaniliyor.')

        cap = cv2.VideoCapture(self._camera_index, cv2.CAP_V4L2)
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
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not cap.isOpened():
            self.get_logger().error(f'Tabela kamerasi acilamadi (index={self._camera_index})')

        try:
            while self._calisiyor and rclpy.ok():
                ret, frame = cap.read()
                if not ret:
                    time.sleep(0.1)
                    continue

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
