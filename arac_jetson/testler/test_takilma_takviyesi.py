"""surus_koprusu takilma (stall) takviyesinin izole testi.
GERCEK _takviyeyi_guncelle mantigi taklit edilir; en kritik kural:
KOMUT YOKKEN ASLA takviye olmayacak (Stop tabelasi / rampa beklemesi /
gorev vetosu hepsi komutu sifirliyor)."""

class Kopru:
    def __init__(self):
        self.aktif = True
        self.hiz_esigi = 0.03
        self.birakma_hizi = 0.10
        self.suresi = 1.2
        self.max_pwm_takilma = 255.0
        self.artis = 70.0
        self.soguma = 1.5
        self.max_pwm = 110.0
        self.olculen = 0.0
        self.baslangic = None
        self.takviye = 0.0
        self.t = 0.0
    def adim(self, dt, komut, olculen):
        self.t += dt
        self.olculen = olculen
        if not self.aktif:
            return 0.0
        if komut < 1e-3:
            self.baslangic = None; self.takviye = 0.0; return 0.0
        esik = self.birakma_hizi if self.takviye > 0 else self.hiz_esigi
        if olculen >= esik:
            self.baslangic = None
            if self.takviye > 0:
                self.takviye = max(0.0, self.takviye -
                                   (self.max_pwm_takilma / self.soguma) * dt)
            return self.takviye
        if self.baslangic is None:
            self.baslangic = self.t; return self.takviye
        if (self.t - self.baslangic) < self.suresi:
            return self.takviye
        self.takviye = min(self.takviye + self.artis * dt,
                           max(0.0, self.max_pwm_takilma - self.max_pwm))
        return self.takviye

hata = 0
def kontrol(ad, got, bekle):
    global hata
    ok = got == bekle; hata += not ok
    print(f"[{'OK ' if ok else 'HATA'}] {ad}: {got}" + ("" if ok else f"  (beklenen {bekle})"))

# 1) KOMUT YOK -> hicbir kosulda takviye olmaz (EN KRITIK KURAL)
k = Kopru()
for i in range(100):
    t = k.adim(0.05, komut=0.0, olculen=0.0)
kontrol("komut yok, 5sn hareketsiz -> takviye YOK", t, 0.0)

# 2) Normal surus (komut var, arac gidiyor) -> takviye YOK
k = Kopru()
for i in range(100):
    t = k.adim(0.05, komut=0.4, olculen=0.35)
kontrol("normal surus -> takviye YOK", t, 0.0)

# 3) Kalkis gecikmesi (1sn hareketsiz) -> HENUZ takviye YOK
k = Kopru()
for i in range(20):   # 1.0 sn
    t = k.adim(0.05, komut=0.4, olculen=0.0)
kontrol("1.0sn hareketsiz (esik 1.2) -> takviye YOK", t, 0.0)

# 4) SUYA GIRDI: komut var, arac hic ilerlemiyor -> takviye BUYUR
k = Kopru()
for i in range(60):   # 3 sn
    t = k.adim(0.05, komut=0.4, olculen=0.0)
kontrol("3sn takili -> takviye var", t > 0, True)
print(f"       3sn sonunda takviye = {t:.0f} PWM (tavan {110+t:.0f})")
for i in range(60):   # 3 sn daha
    t = k.adim(0.05, komut=0.4, olculen=0.0)
kontrol("uzun takilma -> takviye TAVANDA (255-110=145)", round(t), 145)

# 5) Arac kurtuldu -> takviye kademeli geri cekilir
adimlar = 0
while t > 0 and adimlar < 200:
    t = k.adim(0.05, komut=0.4, olculen=0.30); adimlar += 1
kontrol("hareket basladi -> takviye sifirlanir", t, 0.0)
kontrol("geri cekilme kademeli (aninda degil)", adimlar > 5, True)
print(f"       geri cekilme {adimlar*0.05:.1f} sn surdu")

# 6) HISTEREZIS: takviye acikken 0.05 m/s 'hareket' sayilmaz
k = Kopru(); 
for i in range(60): t = k.adim(0.05, komut=0.4, olculen=0.0)
once = t
t = k.adim(0.05, komut=0.4, olculen=0.05)   # esik 0.03 ustu ama birakma 0.10 alti
kontrol("takviye acikken 0.05m/s -> hala takili sayilir", t >= once, True)

# 7) Komut kesilirse takviye ANINDA sifir (Stop tabelasi senaryosu)
t = k.adim(0.05, komut=0.0, olculen=0.0)
kontrol("Stop tabelasi (komut 0) -> takviye ANINDA sifir", t, 0.0)

# 8) Devre disi birakilabilir
k = Kopru(); k.aktif = False
for i in range(100): t = k.adim(0.05, komut=0.4, olculen=0.0)
kontrol("takilma_tespiti_aktif=False -> takviye YOK", t, 0.0)

print("SONUC:", "TUM TESTLER GECTI" if hata == 0 else f"{hata} HATA")
raise SystemExit(1 if hata else 0)
