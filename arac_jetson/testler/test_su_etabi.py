"""SU ETABI mantiginin izole testi (ROS'suz).

Kritik kurallar:
  * etap 1 -> ACIK, etap 2+ -> KAPALI
  * azami sure dolunca KENDILIGINDEN kapanir (kor kalmasin)
  * nokta atici karar zincirinde su etabi EGIM MODLARINDAN ONCE gelir
  * surus_koprusu: su etabinda inis kilidi KALKAR
"""
import io, re

SU_BASLAT, SU_BITIS, AZAMI = 1, 2, 45.0
hata = 0
def kontrol(ad, got, bekle):
    global hata
    ok = got == bekle; hata += not ok
    print(f"[{'OK ' if ok else 'HATA'}] {ad}: {got}" + ("" if ok else f"  (beklenen {bekle})"))

# --- 1) Etap gecisleri ---
def su_acik_mi(etap, onceki):
    yeni = (etap == SU_BASLAT)
    if etap >= SU_BITIS:
        yeni = False
    return yeni

durum = False
for etap, bekle in [(0, False), (1, True), (2, False), (3, False), (8, False)]:
    durum = su_acik_mi(etap, durum)
    kontrol(f"etap {etap} -> su etabi", durum, bekle)

# --- 2) Emniyet zaman asimi ---
def zaman_asimi(aktif, gecen):
    return False if (aktif and gecen >= AZAMI) else aktif
kontrol("44sn -> hala acik", zaman_asimi(True, 44.0), True)
kontrol("45sn -> KENDILIGINDEN kapanir", zaman_asimi(True, 45.0), False)
kontrol("kapaliyken zaman asimi zararsiz", zaman_asimi(False, 99.0), False)

# --- 3) KARAR SIRASI: su etabi egim modlarindan ONCE olmali ---
kaynak = io.open('/home/tufan-yer/tufan_ika_repo/arac_jetson/tufan_v2_ws/src/'
                 'on_bosluk_nokta_atici.py', encoding='utf-8').read()
i_su = kaynak.index('if self._su_etabi:')
i_tirmanma = kaynak.index('if egim_on > egim_esigi:')
i_inis = kaynak.index('if egim_on < -egim_esigi:')
i_rampa = kaynak.index('if self._rampa_etabi:')
kontrol("su etabi TIRMANMA'dan once", i_su < i_tirmanma, True)
kontrol("su etabi INIS'ten once", i_su < i_inis, True)
kontrol("rampa etabi hala egim modlarindan SONRA (bozulmadi)",
        i_rampa > i_inis, True)
kontrol("su etabinda engel kirpmasi YOK",
        'engel kirpmasi YOK - suya duz giris' in kaynak, True)

# --- 4) surus_koprusu: su etabinda inis kilidi kalkar ---
k2 = io.open('/home/tufan-yer/tufan_ika_repo/arac_jetson/tufan_v2_ws/src/'
             'surus_koprusu.py', encoding='utf-8').read()
kontrol("inis kilidi su etabinda muaf",
        'not (self._su_etabi and self._takilma_suda_inis)' in k2, True)
kontrol("/su_etabi aboneligi var", "'/su_etabi'" in k2, True)

def inis_kilidi(egim, su_etabi, muaf=True):
    """True = takviye ENGELLENIR"""
    return egim < -3.0 and not (su_etabi and muaf)
kontrol("kuru inis (-8) -> takviye ENGELLENIR", inis_kilidi(-8.0, False), True)
kontrol("su etabinda inis (-8) -> takviye SERBEST", inis_kilidi(-8.0, True), False)
kontrol("duz zemin -> zaten engellenmez", inis_kilidi(0.0, False), False)

# --- 5) Etap yoneticisi yayin/tekrar deseni ---
k3 = io.open('/home/tufan-yer/tufan_ika_repo/arac_jetson/tufan_v2_ws/src/'
             'tabela_etap_yoneticisi.py', encoding='utf-8').read()
kontrol("/su_etabi yayincisi var", "'/su_etabi'" in k3, True)
kontrol("periyodik tekrar var (kendini iyilestirme)",
        '_su_etabi_durumunu_tekrarla' in k3, True)
kontrol("azami sure emniyeti var", 'SU_ETABI_AZAMI_SURE_S' in k3, True)

print("SONUC:", "TUM TESTLER GECTI" if hata == 0 else f"{hata} HATA")
raise SystemExit(1 if hata else 0)
