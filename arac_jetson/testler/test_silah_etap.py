"""9/10 tabela ile silah fazi testi - ARAC YOK."""
import sys, time
sys.path.insert(0,'/home/tufan-yer/tufan_ika_repo/arac_jetson/tufan_v2_ws/src')
import rclpy
from std_msgs.msg import String, Bool, Int32
from nav_msgs.msg import Odometry
import tabela_etap_yoneticisi as T

rclpy.init()
def _siraya_getir(n, hedef):
    """Sirali durum makinesini 'hedef' tabelasina kadar yurutur.
    (Sirali kabul 2026-09-07'de eklendi; testler artik dogrudan 9
    gonderemiyor - once 1..8 ve Stop gecilmeli.)"""
    import time as _t
    from std_msgs.msg import String as _S
    import tabela_etap_yoneticisi as _T
    for c in _T.BEKLENEN_SIRA:
        if c == hedef:
            break
        if c == 'Stop':
            for _ in range(_T.STOP_DOGRULAMA_ADEDI):
                n._tespit_cb(_S(data='Stop:0.95'))
        else:
            for _ in range(_T.STOP_DOGRULAMA_ADEDI):
                n._tespit_cb(_S(data='%s:0.9' % c))
        _t.sleep(1.05)

n=T.TabelaEtapYoneticisi() if hasattr(T,'TabelaEtapYoneticisi') else None
if n is None:
    import inspect
    cls=[c for _,c in inspect.getmembers(T,inspect.isclass) if c.__module__==T.__name__]
    n=cls[0]()
kayit={'turret':[], 'silah':[], 'tabela':[], 'otonom':[], 'goal':[]}
n._turret_model_pub.publish=lambda m: kayit['turret'].append(m.data)
n._silah_modu_pub.publish  =lambda m: kayit['silah'].append(m.data)
n._tabela_model_pub.publish=lambda m: kayit['tabela'].append(m.data)
n._otonom_surus_pub.publish=lambda m: kayit['otonom'].append(m.data)
n._goal_pub.publish        =lambda m: kayit['goal'].append((round(m.pose.position.x,2),round(m.pose.position.y,2)))
n._etap_pub.publish=lambda m: None
n._parkur_pub.publish=lambda m: None
n._kayar_engel_pub.publish=lambda m: None
o=Odometry(); o.pose.pose.position.x=7.0; o.pose.pose.position.y=2.0; o.pose.pose.orientation.w=1.0
n._son_odom=o
n._surus_modu='OTONOM'

def temizle():
    for k in kayit: kayit[k].clear()
def gonder(cls, kere=3, guven=0.9):
    for i in range(kere): n._tespit_cb(String(data=f'{cls}:{guven}'))

_siraya_getir(n, 'Nine')
print("1) TEK KARE 'Nine' -> hicbir sey olmamali (sahte tespit korumasi)")
temizle(); gonder('Nine', kere=1)
print("   turret=%s silah=%s otonom=%s  (hepsi bos olmali)" % (kayit['turret'],kayit['silah'],kayit['otonom']))
assert not kayit['turret'] and not kayit['silah']

print("2) 3 ARDISIK 'Nine' -> SILAH FAZI BASLAMALI")
temizle(); gonder('Nine', kere=3)
print(f"   turret_model={kayit['turret']}  silah_modu={kayit['silah']}")
print(f"   tabela_model={kayit['tabela']}  otonom_surus={kayit['otonom']}")
print(f"   hedef (yerinde dur)={kayit['goal']}")
assert kayit['turret']==[True], kayit['turret']
assert kayit['silah']==['OTONOM'], kayit['silah']
assert kayit['tabela']==[False], "silah fazinda tabela modeli KAPALI olmali (GPU turret'e)"
assert kayit['otonom']==[False], kayit['otonom']
assert kayit['goal']==[(7.0,2.0)], kayit['goal']
print("   -> OK")

print("3) Tekrar 'Nine' -> IKINCI KEZ tetiklenmemeli")
temizle(); gonder('Nine', kere=3)
print(f"   turret={kayit['turret']} silah={kayit['silah']} (bos olmali)")
assert not kayit['turret'] and not kayit['silah']

print("4) 3 ATIS TAMAMLANDI -> on kamera ACIK, silah KAPALI")
temizle()
n._hedef_vuruldu_cb(Int32(data=3))
print(f"   turret_model={kayit['turret']}  silah_modu={kayit['silah']}")
print(f"   tabela_model={kayit['tabela']}  otonom_surus={kayit['otonom']}")
assert kayit['turret']==[False], kayit['turret']
assert kayit['silah']==['MANUEL'], kayit['silah']
assert kayit['tabela']==[True], kayit['tabela']
print("   -> OK")

print("4b) ARDINDAN 10. tabela -> ikinci kez calismamali (faz zaten bitti)")
temizle(); gonder('Ten', kere=3)
print(f"   turret={kayit['turret']} silah={kayit['silah']} (bos olmali)")
assert not kayit['turret'] and not kayit['silah']
print("   -> OK")

print("5) STOP tabelasi -> ARTIK silah fazini tetiklememeli")
temizle()
for i in range(5): n._tespit_cb(String(data='Stop:0.95'))
print(f"   turret={kayit['turret']} silah={kayit['silah']} (bos olmali)")
assert not kayit['turret'] and not kayit['silah']
print("   -> OK")

print("6) SAHTE tespit dizisi (9,8,9,2,9) -> tetiklenmemeli")
n._silah_fazi_aktif=False; temizle()
for c in ('Nine','Eight','Nine','Two','Nine'): n._tespit_cb(String(data=f'{c}:0.8'))
print(f"   turret={kayit['turret']} (bos olmali - ardisik degil)")
assert not kayit['turret']
print("   -> OK")
n.destroy_node(); rclpy.shutdown()
print("\nTUM TESTLER GECTI")
