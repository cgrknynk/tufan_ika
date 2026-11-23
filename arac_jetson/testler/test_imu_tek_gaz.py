"""Tabelasiz dik engel: IMU tek basina ORANTILI gaz acar.
Eski koruma (kucuk egimlerde tam gaz YOK) bozulmamali."""
import io, re
K = io.open('/home/tufan-yer/tufan_ika_repo/arac_jetson/tufan_v2_ws/src/surus_koprusu.py',
            encoding='utf-8').read()
def vars_(ad):
    m = re.search(r"declare_parameter\('%s',\s*([0-9.\-]+)\)" % ad, K); assert m, ad
    return float(m.group(1))
MAX   = vars_('max_pwm'); EGIM_MAX = vars_('egim_max_pwm')
ESIK  = vars_('imu_tek_basina_esigi_derece'); TAM = vars_('imu_tam_gaz_derece')
EGIM_ESIK = vars_('egim_esigi_derece')
hata=0
def kontrol(ad, got, bekle, tol=0.6):
    global hata
    ok = abs(got-bekle)<tol if isinstance(bekle,(int,float)) and not isinstance(bekle,bool) else got==bekle
    hata += not ok
    print(f"[{'OK ' if ok else 'HATA'}] {ad}: {got}" + ("" if ok else f"  (beklenen {bekle})"))

def tavan(egim, tabela_var, hizlanma=False, imu_aktif=True):
    if hizlanma: return 255.0
    egim_var = egim > EGIM_ESIK
    if egim_var and tabela_var:          # TABELA YOLU (degismedi)
        return EGIM_MAX
    if imu_aktif and egim >= ESIK:       # TABELASIZ YOL (yeni)
        oran = min(max((egim-ESIK)/max(TAM-ESIK,0.1),0.0),1.0)
        return MAX + oran*(EGIM_MAX-MAX)
    return MAX

print(f"max={MAX} egim_max={EGIM_MAX} imu_esik={ESIK} tam_gaz={TAM}")
kontrol("esik 5 derece", ESIK, 5.0)
kontrol("tam gaz 12 derece", TAM, 12.0)

print("\n-- ESKI KORUMA BOZULMADI MI (tabela YOK, kucuk egim) --")
kontrol("duz yol (0 der)", tavan(0.0, False), MAX)
kontrol("3 der -> NORMAL (esik altinda)", tavan(3.0, False), MAX)
kontrol("4.9 der -> hala NORMAL", tavan(4.9, False), MAX)

print("\n-- TABELASIZ DIK ENGEL: ORANTILI --")
kontrol("5 der (esik) -> gaz baslıyor", tavan(5.0, False), MAX)
kontrol("8 der -> orantili", tavan(8.0, False), MAX+((8.0-ESIK)/(TAM-ESIK))*(EGIM_MAX-MAX))
kontrol("10 der (dik engel) -> guclu gaz", tavan(10.0, False), MAX+((10.0-ESIK)/(TAM-ESIK))*(EGIM_MAX-MAX))
kontrol("12 der -> TAM GAZ", tavan(12.0, False), EGIM_MAX)
kontrol("20 der -> TAM GAZ (kirpilir)", tavan(20.0, False), EGIM_MAX)
kontrol("kalktikca ARTIYOR", tavan(11.0,False) > tavan(7.0,False), True)

print("\n-- TABELA YOLU DEGISMEDI --")
kontrol("tabela + 5 der -> TAM GAZ (eski davranis)", tavan(5.0, True), EGIM_MAX)
kontrol("tabela + 20 der -> TAM GAZ", tavan(20.0, True), EGIM_MAX)

print("\n-- DIGER --")
kontrol("hizlanma parkuru oncelikli", tavan(0.0, False, hizlanma=True), 255.0)
kontrol("kapatilabilir", tavan(20.0, False, imu_aktif=False), MAX)
kontrol("inis (negatif egim) gaz acmaz", tavan(-15.0, False), MAX)

kontrol("kaynak: tabela yolu ONCE donuyor (ezilme yok)",
        re.search(r"if tirmaniyor:\s*\n\s*return self\._egim_max_pwm", K) is not None, True)
print("\nSONUC:", "TUM TESTLER GECTI" if hata==0 else f"{hata} HATA")
raise SystemExit(1 if hata else 0)
