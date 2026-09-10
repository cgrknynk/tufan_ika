"""Kamera kaynagi cozumleyicisinin (by-id / by-path / index) izole testi.
SAHTE bir /sys + /dev agaci kurup gercek dosya sistemi uzerinde calisir."""
import os, sys, shutil, tempfile
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import kamera_kaynagi as K

def agac_kur(kok, kameralar):
    """kameralar: [(videoN, isim, index_dosyasi, usb_port, seri), ...]"""
    sysk = os.path.join(kok, 'sys/class/video4linux')
    devk = os.path.join(kok, 'dev')
    for d in (sysk, devk, devk + '/v4l/by-id', devk + '/v4l/by-path'):
        os.makedirs(d, exist_ok=True)
    for n, isim, idx, port, seri in kameralar:
        vd = os.path.join(sysk, 'video%d' % n)
        os.makedirs(vd, exist_ok=True)
        open(os.path.join(vd, 'name'), 'w').write(isim + '\n')
        open(os.path.join(vd, 'index'), 'w').write(idx + '\n')
        usbd = os.path.join(kok, 'sys/bus/usb/devices', port)
        os.makedirs(os.path.join(usbd, port + ':1.0'), exist_ok=True)
        if seri is not None:
            open(os.path.join(usbd, 'serial'), 'w').write(seri + '\n')
        os.symlink(os.path.join(usbd, port + ':1.0'), os.path.join(vd, 'device'))
        # gercek dugum + kararli baglantilar
        gercek = os.path.join(devk, 'video%d' % n)
        open(gercek, 'w').close()
        if idx == '0':
            os.symlink(gercek, os.path.join(
                devk, 'v4l/by-path', 'pci-usb-0:%s:1.0-video-index0' % port))
            if seri:
                bid = os.path.join(devk, 'v4l/by-id',
                                   'usb-046d_%s_%s-video-index0' % (isim.replace(' ', '_'), seri))
                if not os.path.exists(bid):
                    os.symlink(gercek, bid)
    K._V4L_SYS_KOK = sysk
    K._V4L_DEV_KOK = os.path.join(devk, 'v4l')
    K._DEV_KOK = devk

hata = 0
def kontrol(ad, got, bekle_iceren):
    global hata
    ok = bekle_iceren in str(got)
    hata += not ok
    print(f"[{'OK ' if ok else 'HATA'}] {ad}: {got}" + ("" if ok else f"  (icermeli: {bekle_iceren})"))

# --- SENARYO 1: iki AYNI C270 (PAYLASILAN sahte seri) + bir C922 ---
kok = tempfile.mkdtemp()
agac_kur(kok, [
    (0, 'C270 HD WEBCAM', '0', '1-2.2.3', 'ORTAK123'),   # tabela
    (1, 'C270 HD WEBCAM', '1', '1-2.2.3', 'ORTAK123'),   # metadata dugumu
    (4, 'C270 HD WEBCAM', '0', '1-2.2.4', 'ORTAK123'),   # arka
    (2, 'C922 Pro Stream', '0', '1-3', 'BENZERSIZ9'),    # turret
])
k, i, a = K._kamera_kaynagi_bul('C270', '1-2.2.4')
kontrol("2 adet C270, seri PAYLASILMIS -> by-path", a, 'by-path')
kontrol("  dogru porta cozuldu", k, '1-2.2.4')
kontrol("  dogru index", i, '4')
k, i, a = K._kamera_kaynagi_bul('C922', None)
kontrol("C922 seri TEK -> by-id", a, 'by-id')
kontrol("  by-id yolu", k, 'by-id')
kontrol("metadata dugumu (index!=0) ELENDI", len(K._kamera_adaylarini_topla('C270')), '2')

# --- SENARYO 2: port ipucu KAYMIS ama tek aday var ---
kok2 = tempfile.mkdtemp()
agac_kur(kok2, [(4, 'C270 HD WEBCAM', '0', '1-2.2.7', 'ORTAK123')])
k, i, a = K._kamera_kaynagi_bul('C270', '1-2.2.4')   # ipucu ARTIK YANLIS
# ESKI DAVRANIS: ipucu tutmayinca None donup camera_index yedegine
# dusuluyordu (= yanlis kamera ya da hic kamera). YENI: tek aday varsa
# belirsizlik yok, kamera BULUNUR. Hangi kimlikle acildigi (by-id mi
# by-path mi) burada onemli degil - seri no bu agacta tek oldugu icin
# by-id kazaniyor, ki bu DAHA da guclu bir kimlik.
kontrol("port kaymis + TEK aday -> yine de bulunur", i, '4')
kontrol("  kararli yol dondu (index degil)", isinstance(k, str), 'True')

# --- SENARYO 3: iki aday, ipucu hicbirine uymuyor -> AYIRT EDILEMEZ ---
kok3 = tempfile.mkdtemp()
agac_kur(kok3, [
    (0, 'C270 HD WEBCAM', '0', '1-9.1', 'ORTAK123'),
    (4, 'C270 HD WEBCAM', '0', '1-9.2', 'ORTAK123'),
])
k, i, a = K._kamera_kaynagi_bul('C270', '1-2.2.4')
kontrol("2 aday + ipucu tutmuyor -> kaynak YOK (yanlis kamera acilmaz)", k, 'None')
kontrol("  sebep loglanir", a, 'ayirt edilemiyor')

# --- SENARYO 4: hic kamera yok ---
kok4 = tempfile.mkdtemp()
agac_kur(kok4, [])
k, i, a = K._kamera_kaynagi_bul('C270', '1-2.2.4')
kontrol("hic aday yok", a, 'aday yok')

# --- SENARYO 5: by-path/by-id baglantisi YOK -> index yedegi ---
kok5 = tempfile.mkdtemp()
agac_kur(kok5, [(6, 'C270 HD WEBCAM', '0', '1-4', None)])
shutil.rmtree(os.path.join(kok5, 'dev/v4l/by-path'))
os.makedirs(os.path.join(kok5, 'dev/v4l/by-path'))
k, i, a = K._kamera_kaynagi_bul('C270', None)
kontrol("kararli yol yok -> index yedegi", a, 'index')
kontrol("  index degeri", k, '6')

for d in (kok, kok2, kok3, kok4, kok5):
    shutil.rmtree(d, ignore_errors=True)

# --- SENARYO 6: yedek index BASKA kamerayi gosteriyor (CANLI yakalandi) ---
# 2026-09-09: yeniden acilista portlar kaydi, arka kameranin yedegi
# camera_index=4 o an C922'ye (SILAH kamerasi) denk geliyordu.
kok6 = tempfile.mkdtemp()
agac_kur(kok6, [
    (0, 'C270 HD WEBCAM', '0', '1-2.1',   'ORTAK123'),
    (2, 'C270 HD WEBCAM', '0', '1-2.2.1', 'ORTAK123'),
    (4, 'C922 Pro Stream', '0', '1-2.2.3', 'BENZERSIZ9'),
])
kontrol("yedek index=4 C922 -> C270 icin REDDEDILIR",
        K._index_ismi_uyuyor_mu(4, 'C270'), 'False')
kontrol("yedek index=2 C270 -> C270 icin KABUL",
        K._index_ismi_uyuyor_mu(2, 'C270'), 'True')
kontrol("yedek index=1 (metadata dugumu) REDDEDILIR",
        K._index_ismi_uyuyor_mu(1, 'C270'), 'False')
kontrol("olmayan index REDDEDILIR", K._index_ismi_uyuyor_mu(99, 'C270'), 'False')
k, i, a = K._kamera_kaynagi_bul('C270', '1-2.2.4')   # ESKI, artik gecersiz ipucu
kontrol("2 C270 + gecersiz ipucu -> kaynak YOK", k, 'None')
shutil.rmtree(kok6, ignore_errors=True)
print("SENARYO 6:", "OK")

print("SONUC:", "TUM TESTLER GECTI" if hata == 0 else f"{hata} HATA")
raise SystemExit(1 if hata else 0)
