"""KameraThread video kaydinin izole testi - GERCEK kamera_sistemi.py kodu
kullanilir, sadece kayit kok klasoru gecici bir dizine yonlendirilir."""
import os, sys, glob, shutil, tempfile, time
sys.path.insert(0, '/home/tufan-yer/Desktop/tufan')
import numpy as np, cv2
import kamera_sistemi as KS

hata = 0
def kontrol(ad, got, bekle):
    global hata
    ok = (got == bekle)
    hata += not ok
    print(f"[{'OK ' if ok else 'HATA'}] {ad}: {got}" + ("" if ok else f"  (beklenen {bekle})"))

kok = tempfile.mkdtemp()
KS.KAYIT_KOK_KLASORU = kok

def kare(renk, w=640, h=480):
    f = np.zeros((h, w, 3), dtype=np.uint8)
    f[:, :] = renk
    return f

t = KS.KameraThread()

# 1) Buton BASILDI -> klasor adi hemen doner, dizin HENUZ olusmadi
klasor = t.kayit_durumu_degistir(True)
kontrol("buton basildi -> klasor adi dondu", isinstance(klasor, str) and klasor.startswith(kok), True)
kontrol("  dizin henuz olusmadi (ilk karede olusur)", os.path.isdir(klasor), False)
kontrol("  kayit istendi", t._kayit_isteniyor, True)
kontrol("  kayit henuz aktif degil", t._kayit_aktif, False)

# 2) run() dongusunun yaptigi gecis
t._kayit_baslat()
kontrol("baslatildi -> dizin olustu", os.path.isdir(t._kayit_klasoru), True)
kontrol("  ayni klasor", t._kayit_klasoru, klasor)

# 3) Uc kameradan kare yaz - FPS OLCUMU icin yeterli sayida
for i in range(30):
    for ad, renk in (('on_kamera', (0,0,255)), ('silah_kamerasi', (0,255,0)),
                     ('arka_kamera', (255,0,0))):
        t._kayit_karesi_yaz(ad, kare(renk))
    time.sleep(1/40.0)      # ~40 fps besleme -> ust sinir 30'a kirpilmali

for ad in ('on_kamera', 'silah_kamerasi', 'arka_kamera'):
    d = t._kayit_durumlari[ad]
    kontrol(f"{ad}: writer acildi", d['writer'] is not None, True)
    kontrol(f"{ad}: tampon bosaltildi", len(d['tampon']), 0)
    kontrol(f"{ad}: tum kareler yazildi", d['kare'], 30)

# 4) Durdur -> dosyalar release edilir ve OKUNABILIR olur
t.kayit_durumu_degistir(False)
t._kayit_bitir()
dosyalar = sorted(os.path.basename(p) for p in glob.glob(os.path.join(klasor, '*.avi')))
kontrol("uc .avi dosyasi olustu", dosyalar,
        ['arka_kamera.avi', 'on_kamera.avi', 'silah_kamerasi.avi'])
for ad in dosyalar:
    yol = os.path.join(klasor, ad)
    boyut = os.path.getsize(yol)
    cap = cv2.VideoCapture(yol)
    okunabilir = cap.isOpened()
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    ret, f = cap.read()
    cap.release()
    kontrol(f"{ad}: OpenCV ile acilabiliyor", okunabilir, True)
    kontrol(f"{ad}: ilk kare okunabiliyor", ret, True)
    kontrol(f"{ad}: kare sayisi", n, 30)
    kontrol(f"{ad}: fps ust sinirda kirpildi (<=30)", fps <= 30.0 + 0.01, True)
    print(f"       ({boyut} bayt, {fps:.1f} fps, {f.shape if ret else '-'})")

# 5) SADECE kamera goruntusu: klasorde .avi DISINDA dosya YOK
hepsi = sorted(os.listdir(klasor))
kontrol("klasorde SADECE video dosyalari var", [x for x in hepsi if not x.endswith('.avi')], [])

# 6) Idempotent: kayitta degilken durdurmak zararsiz
t.kayit_durumu_degistir(False)
kontrol("kayitta degilken durdur -> zararsiz", t._kayit_aktif, False)

# 7) Ikinci basis YENI klasor acar (uzerine yazmaz)
klasor2 = t.kayit_durumu_degistir(True)
kontrol("ikinci basis -> FARKLI klasor", klasor2 != klasor, True)
t.kayit_durumu_degistir(False)

# 8) Cozunurluk degisirse kare olceklenir (bozuk dosya olmaz)
t.kayit_durumu_degistir(True); t._kayit_baslat()
for i in range(15):
    t._kayit_karesi_yaz('on_kamera', kare((10,20,30), 640, 480))
t._kayit_karesi_yaz('on_kamera', kare((10,20,30), 320, 240))   # boyut DEGISTI
kontrol("cozunurluk degisiminde kare sayisi arttI (kirilmadi)",
        t._kayit_durumlari['on_kamera']['kare'], 16)
t.kayit_durumu_degistir(False); t._kayit_bitir()

shutil.rmtree(kok, ignore_errors=True)
print("SONUC:", "TUM TESTLER GECTI" if hata == 0 else f"{hata} HATA")
raise SystemExit(1 if hata else 0)
