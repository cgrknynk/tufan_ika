"""Tam-kare halusinasyonu filtresi: dev kutu ELENMELI, gercek tabela
BASTIRILMAMALI. GERCEK tabela_node.py'deki secim mantigi taklit edilir."""
import io, re, sys
sys.path.insert(0,'/home/tufan-yer/tufan_ika_repo/arac_jetson/tufan_v2_ws/src')
K = io.open('/home/tufan-yer/tufan_ika_repo/arac_jetson/tufan_v2_ws/src/tabela_node.py',
            encoding='utf-8').read()
m = re.search(r"declare_parameter\('maks_kutu_alan_orani',\s*([0-9.]+)\)", K)
assert m; ESIK = float(m.group(1))
hata=0
def kontrol(ad, got, bekle):
    global hata
    ok = got==bekle; hata += not ok
    print(f"[{'OK ' if ok else 'HATA'}] {ad}: {got}" + ("" if ok else f"  (beklenen {bekle})"))

W,H = 640,480; KARE = float(W*H)
def sec(tespitler):
    """tespitler: [(ad, guven, (x1,y1,x2,y2))]"""
    en_iyi=None
    for ad,guv,(x1,y1,x2,y2) in tespitler:
        alan = max(0.0,x2-x1)*max(0.0,y2-y1)
        if alan/KARE > ESIK: continue
        if en_iyi is None or guv > en_iyi[1]: en_iyi=(ad,guv)
    return en_iyi

print(f"maks_kutu_alan_orani = {ESIK}")
kontrol("esik makul (0.2-0.5)", 0.2 <= ESIK <= 0.5, True)

# SAHADA GORULEN DURUM: tam kareyi kaplayan sahte 'Ten'
dev = ('Ten', 0.63, (5,5,W-5,H-5))          # ~%97 alan
kontrol("tam-kare sahte 'Ten' ELENIR", sec([dev]), None)

# Gercek tabela: kucuk kutu
gercek = ('One', 0.60, (280,200,360,290))    # ~%2.3 alan
kontrol("gercek kucuk tabela GECER", sec([gercek]), ('One',0.60))

# ASIL ARIZA: dev sahte, gercegi BASTIRIYOR mu?
kontrol("dev sahte + gercek -> GERCEK bildirilir (bastirma YOK)",
        sec([dev, gercek]), ('One',0.60))

# Iki gercek tespit -> yuksek guvenli
kontrol("iki makul tespit -> en guvenli",
        sec([('One',0.60,(280,200,360,290)), ('Two',0.80,(100,100,180,190))]),
        ('Two',0.80))

# Sinir: tam esikte
sinir_alan = ESIK*KARE
yan = int(sinir_alan**0.5)
kontrol("esigin ALTINDAKI kutu gecer",
        sec([('Five',0.7,(0,0,yan-20,yan-20))]), ('Five',0.7))
kontrol("esigin USTUNDEKI kutu elenir",
        sec([('Five',0.7,(0,0,yan+40,yan+40))]), None)

# Hic tespit yok
kontrol("tespit yok -> None", sec([]), None)

# Kaynak kodu garantileri
kontrol("filtre argmax'tan ONCE uygulaniyor",
        re.search(r"def _makul_tespitleri_sec.*?maks_kutu_alan.*?continue.*?en_iyi",
                  K, re.S) is not None, True)
kontrol("eski 'confs.argmax()' dogrudan secimi KALDIRILDI",
        "en_iyi_idx = int(confs.argmax())" not in K, True)
kontrol("hata durumunda None donuyor (dugum olmuyor)",
        re.search(r"def _makul_tespitleri_sec.*?except Exception.*?return None", K, re.S) is not None, True)

print("SONUC:", "TUM TESTLER GECTI" if hata==0 else f"{hata} HATA")
raise SystemExit(1 if hata else 0)
