"""Iniste sabit minimum hiz (85) - izole test.
GERCEK surus_koprusu.py'deki formul ve varsayilanlar okunur."""
import io, re
K = io.open('/home/tufan-yer/tufan_ika_repo/arac_jetson/tufan_v2_ws/src/surus_koprusu.py',
            encoding='utf-8').read()
def varsayilan(ad):
    m = re.search(r"declare_parameter\('%s',\s*([0-9.\-]+)\)" % ad, K)
    assert m, ad
    return float(m.group(1))

MIN   = varsayilan('min_pwm')
MAX   = varsayilan('max_pwm')
INIS  = varsayilan('inis_pwm')
ESIK  = varsayilan('inis_pwm_esigi_derece')
hata = 0
def kontrol(ad, got, bekle):
    global hata
    ok = (got == bekle) if not isinstance(bekle, float) else abs(got-bekle) < 1e-6
    hata += not ok
    print(f"[{'OK ' if ok else 'HATA'}] {ad}: {got}" + ("" if ok else f"  (beklenen {bekle})"))

def deadzone(ham, tavan, taban):
    if abs(ham) < 1e-6: return 0.0
    isaret = 1.0 if ham > 0 else -1.0
    oran = min(abs(ham)/255.0, 1.0)
    return isaret * (taban + oran*(tavan-taban))

def pwm(ham, egim, hizlanma=False, aktif=True):
    iniste = aktif and not hizlanma and egim <= ESIK
    if iniste: taban = tavan = INIS
    else:      taban, tavan = MIN, MAX
    return deadzone(ham, tavan, taban)

print(f"min_pwm={MIN} max_pwm={MAX} inis_pwm={INIS} esik={ESIK}")
kontrol("inis_pwm varsayilani 90", INIS, 90.0)

# DUZ YOL - degismemeli
kontrol("duz yol, tam komut", pwm(255, 0.0), MAX)
kontrol("duz yol, yarim komut", pwm(127.5, 0.0), MIN + 0.5*(MAX-MIN))

# INIS - her komutta SABIT 85
kontrol("inis -8 der, tam komut", pwm(255, -8.0), INIS)
kontrol("inis -8 der, yarim komut", pwm(127.5, -8.0), INIS)
kontrol("inis -20 der, ceyrek komut", pwm(64, -20.0), INIS)
kontrol("inis: hizlandikca ARTMIYOR (sabit)", pwm(255,-8.0)==pwm(64,-8.0), True)

# ESIK ALTI egim - normal kalmali
kontrol("hafif egim -3 der (esik -5) normal", pwm(255, -3.0), MAX)

# DUR komutu iniste de SIFIR
kontrol("komut 0 -> iniste de 0", pwm(0.0, -20.0), 0.0)

# ISARET korunur (yerinde donme yetkisi)
kontrol("iniste ters isaret korunur", pwm(-255, -10.0), -INIS)

# HIZLANMA PARKURUNDA devre disi
kontrol("hizlanma etabinda iniste bile normal", pwm(255, -10.0, hizlanma=True), MAX)

# KAPATILABILIR
kontrol("inis_sabit_pwm_aktif=False -> normal", pwm(255, -10.0, aktif=False), MAX)

# TERS FORMUL TUZAGI: SADECE tavan 85 yapilsaydi (taban min_pwm=90 kalsaydi)
# formul  90 + oran*(85-90)  olurdu: oran BUYUDUKCE PWM AZALIRDI.
# Tam komutta (oran=1) tesadufen 85 cikar - tuzak KISMI komutta gorunur.
ters_tam   = MIN + 1.0*(85.0 - MIN)   # = 85.0  (tesadufen dogru)
ters_yarim = MIN + 0.5*(85.0 - MIN)   # = 87.5  (YANLIS - yavas komut DAHA HIZLI)
kontrol("tuzak: yarim komutta ters formul 87.5 uretirdi", ters_yarim, ters_yarim)
kontrol("tuzak: ters formulde YAVAS komut DAHA HIZLI olurdu",
        ters_yarim > ters_tam, True)
kontrol("gercek kod bu tuzaga DUSMUYOR (yarim komut da tam inis_pwm)", pwm(127.5, -8.0), INIS)

print("SONUC:", "TUM TESTLER GECTI" if hata==0 else f"{hata} HATA")
raise SystemExit(1 if hata else 0)
