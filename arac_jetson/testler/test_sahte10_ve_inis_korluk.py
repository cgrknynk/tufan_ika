"""1) Sahte 10 silah fazini KESMEMELI (atislar bitince devam).
   2) Rampa etabinda INIS'te de LIDAR KOR olmali."""
import io, re
T = io.open('/home/tufan-yer/tufan_ika_repo/arac_jetson/tufan_v2_ws/src/tabela_etap_yoneticisi.py',
            encoding='utf-8').read()
N = io.open('/home/tufan-yer/tufan_ika_repo/arac_jetson/tufan_v2_ws/src/on_bosluk_nokta_atici.py',
            encoding='utf-8').read()
hata=0
def kontrol(ad, kosul):
    global hata; hata += not kosul
    print(f"[{'OK ' if kosul else 'HATA'}] {ad}")

print("-- 1) SAHTE 10 KORUMASI --")
kontrol("faz aktifken 10 YOK SAYILIYOR (erken return)",
        re.search(r"elif etap == SILAH_BITIS_ETABI:.*?if self\._silah_fazi_aktif:.*?return",
                  T, re.S) is not None)
kontrol("eski 'yedek guvence' fazi kesen dal KALDIRILDI",
        "(3 atis TAMAMLANMADAN) yedek guvence" not in T)
kontrol("10 gorulunce _silah_fazini_bitir ARTIK CAGRILMIYOR",
        re.search(r"elif etap == SILAH_BITIS_ETABI:(?:(?!\n    def ).)*?_silah_fazini_bitir",
                  T, re.S) is None)
kontrol("emniyet suresi sabiti var", "SILAH_FAZI_AZAMI_SURE_S = 60.0" in T)
kontrol("emniyet kontrolu periyodik cagriliyor",
        "self._silah_emniyet_suresini_kontrol_et()" in T)
kontrol("emniyet suresi surus modundan BAGIMSIZ (mod kontrolunden ONCE)",
        re.search(r"_silah_emniyet_suresini_kontrol_et\(\)\s*\n\s*if self\._surus_modu != 'OTONOM'",
                  T, re.S) is not None)
kontrol("emniyet dolunca rampa _R_SILAH_BEKLE'de takili kalmiyor",
        re.search(r"def _silah_emniyet_suresini_kontrol_et.*?_R_SILAH_BEKLE.*?_R_INIS_BEKLE",
                  T, re.S) is not None)
kontrol("3 atis bitince baslangic zamani temizleniyor",
        T.count("self._silah_fazi_baslangici = None") >= 3)

print("\n-- 2) INISTE LIDAR KORLUGU --")
i_rampa = N.index("if self._rampa_etabi:")
i_tirman = N.index("if egim_on > egim_esigi:")
i_inis   = N.index("if egim_on < -egim_esigi:")
kontrol("rampa korlugu TIRMANMA modundan ONCE", i_rampa < i_tirman)
kontrol("rampa korlugu INIS modundan ONCE", i_rampa < i_inis)
kontrol("tek bir rampa dali kaldi (kopya yok)",
        N.count("self._rampa_etabinda_ilerle(yaw, odom, durum)") == 1)
kontrol("su etabi korlugu hala en onde",
        N.index("if self._su_etabi:") < i_rampa)

# Mod secimi simulasyonu (kaynaktaki SIRA ile birebir)
def mod(su, rampa, egim_on, egim_yan=0.0, esik=3.0):
    if su: return 'SU'
    if rampa: return 'RAMPA_KOR'
    if egim_on > esik: return 'TIRMANMA'
    if egim_on < -esik: return 'INIS'
    if abs(egim_yan) > esik: return 'YAN_EGIM'
    return 'NORMAL'

print()
for ad, got, bek in (
    ("rampa acik + INIS (-15) -> KOR", mod(False, True, -15.0), 'RAMPA_KOR'),
    ("rampa acik + TIRMANMA (+18) -> KOR", mod(False, True, 18.0), 'RAMPA_KOR'),
    ("rampa acik + duz -> KOR", mod(False, True, 0.0), 'RAMPA_KOR'),
    ("rampa KAPALI + inis -> INIS modu", mod(False, False, -15.0), 'INIS'),
    ("su etabi her seyin onunde", mod(True, True, -15.0), 'SU'),
):
    kontrol(f"{ad}  [{got}]", got == bek)

print("\nSONUC:", "TUM TESTLER GECTI" if hata==0 else f"{hata} HATA")
raise SystemExit(1 if hata else 0)
