"""Yeni rampa dizisi: tirmanista DURMA YOK, tepede IMU duzlesince DUR +
silah fazi (9. tabela BEKLENMEDEN). GERCEK kaynaktaki sabitler/durumlar
okunur, karar mantigi birebir taklit edilir."""
import io, re
KAYNAK = '/home/tufan-yer/tufan_ika_repo/arac_jetson/tufan_v2_ws/src/tabela_etap_yoneticisi.py'
K = io.open(KAYNAK, encoding='utf-8').read()

def sabit(ad):
    m = re.search(r'^%s = ([0-9.]+)' % ad, K, re.M)
    assert m, ad
    return float(m.group(1))

IMU_ESIGI   = sabit('RAMPA_IMU_ESIGI_DERECE')
DUZ_ESIGI   = sabit('RAMPA_DUZ_ESIGI_DERECE')
DUZ_GECIKME = sabit('RAMPA_DUZLESME_GECIKMESI_S')
DEVAM_GEC   = sabit('SILAH_SONRASI_DEVAM_GECIKMESI_S')
_BOS, _TIRMANIS, _DUZLESME, _SILAH, _INIS = 0, 1, 6, 7, 3

hata = 0
def kontrol(ad, got, bekle):
    global hata
    ok = got == bekle; hata += not ok
    print(f"[{'OK ' if ok else 'HATA'}] {ad}: {got}" + ("" if ok else f"  (beklenen {bekle})"))

class Rampa:
    def __init__(self):
        self.durum=_BOS; self.esik_t=None; self.t=0.0
        self.durdu=0; self.silah_basladi=0; self.otonom=True
    def tetikle(self):                      # 8 / Stop goruldu
        if self.durum==_BOS: self.durum=_TIRMANIS
    def odom(self, egim, dt=0.1):
        self.t += dt
        d=self.durum
        if d==_TIRMANIS:
            if egim >= IMU_ESIGI:
                self.durum=_DUZLESME; self.esik_t=None   # DURMA YOK
        elif d==_DUZLESME:
            if abs(egim) <= DUZ_ESIGI:
                if self.esik_t is None: self.esik_t=self.t
                elif (self.t-self.esik_t) >= DUZ_GECIKME:
                    self.durum=_SILAH; self.esik_t=None
                    self.durdu += 1; self.otonom=False
                    self.silah_basladi += 1
            else:
                self.esik_t=None
    def uc_atis_bitti(self):
        self.t += DEVAM_GEC
        self.otonom=True
        if self.durum==_SILAH: self.durum=_INIS

# --- SENARYO: 8 goruldu -> tirman -> tepede duzles ---
r=Rampa(); r.tetikle()
for _ in range(30): r.odom(2.0)      # henuz duz yol
kontrol("duz yolda tirmanis durumunda bekler", r.durum, _TIRMANIS)
kontrol("  hic durmadi", r.durdu, 0)

for _ in range(60): r.odom(18.0)     # 6 sn boyunca DIK EGIM
kontrol("egim algilandi -> duzlesme bekleniyor", r.durum, _DUZLESME)
kontrol("  TIRMANISTA DURMADI (eski kod dururdu)", r.durdu, 0)
kontrol("  otonom surus hic kesilmedi", r.otonom, True)

for _ in range(10): r.odom(1.0)      # 1 sn duz (teyit suresi 1.5)
kontrol("kisa duzluk teyit olmadan durdurmaz", r.durum, _DUZLESME)
r.odom(20.0)                          # tekrar egim -> sayac sifirlanmali
for _ in range(10): r.odom(1.0)
kontrol("sarsinti sonrasi sayac sifirlandi, hala beklemede", r.durum, _DUZLESME)

for _ in range(10): r.odom(1.0)      # toplam >1.5 sn duz
kontrol("TEPEDE DUZLESTI -> DURDU", r.durum, _SILAH)
kontrol("  tam 1 kez duruldu", r.durdu, 1)
kontrol("  SILAH FAZI BASLADI (9. tabela GORULMEDEN)", r.silah_basladi, 1)
kontrol("  otonom surus kesildi", r.otonom, False)

r.uc_atis_bitti()
kontrol("3 atis + gecikme -> INIS adimina gecti", r.durum, _INIS)
kontrol("  otonom surus geri acildi", r.otonom, True)

# --- Kaynak kodu garantileri ---
kontrol("tirmanista _gecici_dur CAGRILMIYOR",
        bool(re.search(r"if d == _R_TIRMANIS_BEKLE:.*?_gecici_dur", K, re.S)
             and not re.search(r"if d == _R_TIRMANIS_BEKLE:(?:(?!elif d ==).)*?_gecici_dur", K, re.S)) or
        not re.search(r"if d == _R_TIRMANIS_BEKLE:(?:(?!elif d ==).)*?_gecici_dur", K, re.S), True)
kontrol("duzlesme durumunda _gecici_dur VAR",
        re.search(r"elif d == _R_DUZLESME_BEKLE:(?:(?!elif d ==).)*?_gecici_dur", K, re.S) is not None, True)
kontrol("duzlesmede silah tetigi cagriliyor",
        re.search(r"elif d == _R_DUZLESME_BEKLE:(?:(?!elif d ==).)*?_silah_fazini_tetikle", K, re.S) is not None, True)
kontrol("9. tabela yolu da ayni tetigi kullaniyor",
        "self._silah_fazini_tetikle('ETAP 9 dogrulandi')" in K, True)
kontrol("cift baslatma korumasi (_silah_fazi_aktif)",
        re.search(r"def _silah_fazini_tetikle.*?if self\._silah_fazi_aktif:\s*\n\s*return", K, re.S) is not None, True)
kontrol("3 atis sonrasi GECIKMELI devam (hemen degil)",
        "_silah_sonrasi_devam" in K and "SILAH_SONRASI_DEVAM_GECIKMESI_S" in K, True)

print("SONUC:", "TUM TESTLER GECTI" if hata==0 else f"{hata} HATA")
raise SystemExit(1 if hata else 0)
