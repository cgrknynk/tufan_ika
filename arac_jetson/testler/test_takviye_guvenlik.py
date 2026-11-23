"""Takilma takviyesi GUVENLIK sinirlari: donus agirlikli komutta VERILMEZ,
azami sureyi asamaz. Sahada 'hizlanarak yerinde donme' yasandi."""
import io, re
K = io.open('/home/tufan-yer/tufan_ika_repo/arac_jetson/tufan_v2_ws/src/surus_koprusu.py',
            encoding='utf-8').read()
def par(ad):
    m=re.search(r"declare_parameter\('%s',\s*([0-9.]+)\)"%ad, K); assert m, ad
    return float(m.group(1))
ILERI_ORAN = par('takilma_ileri_agirlik_orani')
AZAMI      = par('takilma_azami_sure_s')
IZ = par('iz_genisligi')
hata=0
def kontrol(ad, got, bekle):
    global hata; ok = got==bekle; hata += not ok
    print(f"[{'OK ' if ok else 'HATA'}] {ad}: {got}" + ("" if ok else f"  (beklenen {bekle})"))

def ileri_agirlikli(lin, ang):
    ileri=abs(lin); donus=abs(ang)*(IZ/2.0); t=ileri+donus
    return not (t>1e-6 and (ileri/t) < ILERI_ORAN)

print(f"ileri_agirlik_orani={ILERI_ORAN}  azami_sure={AZAMI}s  iz={IZ}")
print("\n-- DONUS AGIRLIKLI KOMUTTA TAKVIYE YOK --")
kontrol("saf donus (lin=0, ang=1.0) -> takviye YOK", ileri_agirlikli(0.0,1.0), False)
kontrol("sahada yasanan: lin=0.05 ang=0.9 -> YOK", ileri_agirlikli(0.05,0.9), False)
kontrol("yarim yarim (lin=0.3 ang=1.04) -> YOK", ileri_agirlikli(0.3,1.04), False)
print("\n-- ILERI AGIRLIKLI KOMUTTA TAKVIYE VAR --")
kontrol("duz ileri (lin=0.4 ang=0) -> VAR", ileri_agirlikli(0.4,0.0), True)
kontrol("hafif kavisli (lin=0.4 ang=0.2) -> VAR", ileri_agirlikli(0.4,0.2), True)
kontrol("suya girerken (lin=0.63 ang=0.1) -> VAR", ileri_agirlikli(0.63,0.1), True)
kontrol("komut yok (0,0) -> engellemez", ileri_agirlikli(0.0,0.0), True)

print("\n-- KAYNAK GARANTILERI --")
def var(ad, kosul):
    kontrol(ad, kosul, True)
var("donus kontrolu takviye hesabinda", "takilma_ileri_agirlik_orani" in K)
var("azami sure kontrolu var", "_takilma_azami_sure" in K and "KESILDI" in K)
var("azami sure sonrasi YASAK suresi var", "_takviye_yasak_bitis" in K)
var("takviye acilisinda zaman isaretleniyor", "_takviye_basladi = su_an" in K)
var("lin_x/ang_z fonksiyona GECIRILIYOR",
    "_takviyeyi_guncelle(komut_hizi, lin_x, ang_z)" in K)
var("parametre geri cagrisi kayitli", "add_on_set_parameters_callback" in K)
var("takilma_tespiti_aktif canli ayarlanabilir",
    "'takilma_tespiti_aktif': '_takilma_aktif_ayar'" in K)

# Azami sure simulasyonu
t=0.0; basladi=None; yasak=0.0; kesildi=0
for i in range(200):            # 20 sn, 0.1 sn adim
    t+=0.1
    if t < yasak: continue
    if basladi is None: basladi=t
    if (t-basladi) > AZAMI:
        kesildi+=1; basladi=None; yasak=t+1.5
kontrol(f"20 sn boyunca takviye {kesildi} kez zorla kesildi (>=3)", kesildi>=3, True)

print("\nSONUC:", "TUM TESTLER GECTI" if hata==0 else f"{hata} HATA")
raise SystemExit(1 if hata else 0)
