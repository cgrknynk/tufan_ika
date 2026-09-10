"""turret_node ACIL STOP kapilarinin izole (ROS'suz) davranis testi."""
import queue

class SahteTurret:
    def __init__(self):
        self._acil_stop = False
        self._arduino_kuyruk = queue.Queue(maxsize=1)
        self._adim_kuyruk = queue.Queue(maxsize=1)
        self._lazer_kuyruk = queue.Queue(maxsize=1)
    # --- turret_node.py ile BIREBIR ayni kapi mantigi ---
    def _komut_gonder(self, karakter, hiz=255):
        if self._acil_stop and karakter != 'x':
            karakter, hiz = 'x', 0
        if self._arduino_kuyruk.full():
            self._arduino_kuyruk.get_nowait()
        self._arduino_kuyruk.put((karakter, hiz))
    def _coklu_adim_gonder(self, yon, n):
        if self._acil_stop:
            return
        if self._adim_kuyruk.full():
            self._adim_kuyruk.get_nowait()
        self._adim_kuyruk.put((yon, n))
    def _tek_adim_gonder(self, yon):
        self._coklu_adim_gonder(yon, 1)
    def _lazer_gonder(self, k):
        if self._acil_stop:
            k = 'k'
        if self._lazer_kuyruk.full():
            self._lazer_kuyruk.get_nowait()
        self._lazer_kuyruk.put(k)
    def _arac_komut_cb(self, data):
        k = data.strip()
        if k == 'EMERGENCY_STOP_CMD':
            self._acil_stop = True
            self._komut_gonder('x'); self._lazer_gonder('k')
        elif k == 'DEVAM_CMD':
            self._acil_stop = False

def bosalt(q):
    r = []
    while not q.empty(): r.append(q.get_nowait())
    return r

h = 0
def kontrol(ad, got, bekle):
    global h
    ok = got == bekle
    h += not ok
    print(f"[{'OK ' if ok else 'HATA'}] {ad}: {got}" + ("" if ok else f"  (beklenen {bekle})"))

t = SahteTurret()
# 1) NORMAL: her sey gecer
t._komut_gonder('d', 255); kontrol("normal hareket", bosalt(t._arduino_kuyruk), [('d',255)])
t._coklu_adim_gonder('w', 50); kontrol("normal coklu adim", bosalt(t._adim_kuyruk), [('w',50)])
t._lazer_gonder('l'); kontrol("normal lazer ac", bosalt(t._lazer_kuyruk), ['l'])

# 2) ACIL STOP GELDI -> aninda 'x' + 'k'
t._arac_komut_cb('EMERGENCY_STOP_CMD')
kontrol("acil stop aninda dur", bosalt(t._arduino_kuyruk), [('x',255)])
kontrol("acil stop aninda lazer kapat", bosalt(t._lazer_kuyruk), ['k'])

# 3) ACIL STOP SURERKEN hicbir hareket/ates gecmez
t._komut_gonder('a', 255); kontrol("kilitli: surekli hareket -> x", bosalt(t._arduino_kuyruk), [('x',0)])
t._coklu_adim_gonder('s', 100); kontrol("kilitli: coklu adim YUTULDU", bosalt(t._adim_kuyruk), [])
t._tek_adim_gonder('w'); kontrol("kilitli: tek adim YUTULDU", bosalt(t._adim_kuyruk), [])
t._lazer_gonder('l'); kontrol("kilitli: lazer ac -> k", bosalt(t._lazer_kuyruk), ['k'])
t._komut_gonder('x'); kontrol("kilitli: 'x' aynen gecer", bosalt(t._arduino_kuyruk), [('x',255)])

# 4) DEVAM -> serbest
t._arac_komut_cb('DEVAM_CMD')
t._komut_gonder('d', 200); kontrol("devam: hareket serbest", bosalt(t._arduino_kuyruk), [('d',200)])
t._coklu_adim_gonder('w', 7); kontrol("devam: coklu adim serbest", bosalt(t._adim_kuyruk), [('w',7)])
t._lazer_gonder('l'); kontrol("devam: lazer serbest", bosalt(t._lazer_kuyruk), ['l'])

# 5) Yer istasyonu: acil stopta ates bayragi ACIK olsa bile yayin FALSE
def ates_yayini(ates_manuel_aktif, acil):
    return bool(ates_manuel_aktif) and not acil
kontrol("yer ist. ates: acil+switch acik", ates_yayini(True, True), False)
kontrol("yer ist. ates: acil yok+switch acik", ates_yayini(True, False), True)
# 6) Yer istasyonu: acil stopta turret komutu SIFIRLANIR (susmaz)
def turret_komutu(x, y, acil):
    return (0.0, 0.0) if acil else (x, y)
kontrol("yer ist. turret: acilda sifir", turret_komutu(0.8, -0.4, True), (0.0, 0.0))
kontrol("yer ist. turret: normalde aynen", turret_komutu(0.8, -0.4, False), (0.8, -0.4))

print("SONUC:", "TUM TESTLER GECTI" if h == 0 else f"{h} HATA")
raise SystemExit(1 if h else 0)
