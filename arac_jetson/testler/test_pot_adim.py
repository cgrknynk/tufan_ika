# Pot -> (a) arac PWM ust siniri  (b) manuel silah adim sayisi
PWM_ALT, PWM_UST = 85, 255
SILAH_ADIM_MIN, SILAH_ADIM_MAX = 2, 100

def pwm_haritasi(ham):                       # DEGISMEDI (referans)
    return max(PWM_ALT, min(PWM_UST, int(round(PWM_ALT + (ham/1023.0)*(PWM_UST-PWM_ALT)))))

def adim_haritasi(pwm):                      # kontrol_paneli_node._pot_adim_sayisi
    aralik = float(PWM_UST - PWM_ALT)
    oran = 0.0 if aralik <= 0 else (float(pwm) - PWM_ALT)/aralik
    oran = min(max(oran, 0.0), 1.0)
    return int(round(SILAH_ADIM_MIN + oran*(SILAH_ADIM_MAX - SILAH_ADIM_MIN)))

hata = 0
# 1) arac hiz haritasi bozulmadi mi (altin deger kontrolu)
for ham, bekle in [(0,85),(256,128),(512,170),(767,212),(1023,255)]:
    got = pwm_haritasi(ham)
    ok = got == bekle
    hata += not ok
    print(f"[{'OK ' if ok else 'HATA'}] PWM  ham={ham:5d} -> {got:3d} (beklenen {bekle})")
# 2) adim ucları
for ham, bekle in [(0,2),(1023,100)]:
    got = adim_haritasi(pwm_haritasi(ham))
    ok = got == bekle
    hata += not ok
    print(f"[{'OK ' if ok else 'HATA'}] ADIM ham={ham:5d} -> {got:3d} (beklenen {bekle})")
# 3) monotonluk + sinir
onceki = -1
for ham in range(0, 1024):
    a = adim_haritasi(pwm_haritasi(ham))
    if a < onceki or not (2 <= a <= 100):
        print(f"[HATA] monotonluk/sinir ham={ham} adim={a}"); hata += 1; break
    onceki = a
else:
    print("[OK ] adim monoton artiyor ve 2..100 araliginda")
# 4) turret_node clamp
MIN, MAX = 2, 100
for gelen, bekle in [(-5,2),(0,2),(2,2),(50,50),(100,100),(9999,100)]:
    got = int(max(MIN, min(MAX, int(gelen))))
    ok = got == bekle; hata += not ok
    print(f"[{'OK ' if ok else 'HATA'}] clamp {gelen} -> {got} (beklenen {bekle})")
# 5) etkin hiz (20Hz heartbeat)
print(f"     etkin hiz: pot dip {2*20} adim/sn, pot tepe {100*20} adim/sn")
print("SONUC:", "TUM TESTLER GECTI" if hata == 0 else f"{hata} HATA")
raise SystemExit(1 if hata else 0)
