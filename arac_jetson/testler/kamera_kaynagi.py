import os

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


