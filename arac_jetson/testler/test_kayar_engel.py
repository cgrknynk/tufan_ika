"""Kayar engel (6. tabela) davranis testi - ARAC YOK, sentetik tarama."""
import math, sys, time
import numpy as np, rclpy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
sys.path.insert(0,'/home/tufan-yer/tufan_ika_repo/arac_jetson/tufan_v2_ws/src')
from on_bosluk_nokta_atici import OnBoslukNoktaAtici

N=720; INC=2*math.pi/N
def scan(fn):
    m=LaserScan(); m.angle_min=-math.pi; m.angle_max=math.pi; m.angle_increment=INC
    m.range_min=0.05; m.range_max=25.0
    r=[]
    for i in range(N):
        ham=m.angle_min+i*INC
        a=math.atan2(math.sin(ham-math.pi),math.cos(ham-math.pi))
        r.append(float(fn(a)))
    m.ranges=r; return m
def odom():
    o=Odometry(); o.pose.pose.position.x=3.0; o.pose.pose.position.y=1.0
    o.pose.pose.orientation.w=1.0; return o

def kapi(mesafe, yan_bosluk=False):
    """Onde kapali kapi; yan_bosluk=True ise SAGDA genis bir gecis var."""
    def f(a):
        c,s=math.cos(a),math.sin(a)
        d=float('inf')
        if c>1e-6:
            t=mesafe/c
            if t>0 and abs(t*s)<2.5: d=min(d,t)      # kapi 5m genis
        if not yan_bosluk:
            if abs(s)>1e-6:
                for yw in (2.5,-2.5):
                    t=yw/s
                    if t>0 and abs(t*c)<20: d=min(d,t)
        else:
            if s>1e-6:                                # sadece SOL duvar
                t=2.5/s
                if t>0 and abs(t*c)<20: d=min(d,t)
        return d
    return f
def acik(a):   # her yer bos
    return float('inf')

rclpy.init()
n=OnBoslukNoktaAtici(); n._aktif_elle=True
hedefler=[]; n._goal_pub.publish=lambda m: hedefler.append((round(m.pose.position.x,2),round(m.pose.position.y,2)))
durumlar=[]; n._durum_yayinla=lambda d: durumlar.append(dict(d))

def kos(ad, fn, engel_etabi, sifirla=True):
    if sifirla:
        n._son_hedef_xy=None; n._son_yayin_zamani=0.0; n._kayar_engel_son_tutma=0.0
        n._referans_yon=None; n._son_yayin_konumu=None
    n._kayar_engel_cb(Bool(data=engel_etabi))
    hedefler.clear(); durumlar.clear()
    n._son_scan=scan(fn); n._son_scan_zamani=time.monotonic()
    n._son_odom=odom(); n._son_odom_zamani=time.monotonic()
    n._tick()
    d=durumlar[-1] if durumlar else {}
    print(f"  {ad:44s} mod={d.get('mod','-'):24s} theta={str(d.get('theta','-')):6s} "
          f"on_serbest={str(d.get('on_serbest','-')):6s} hedef={hedefler[-1] if hedefler else None}")
    return d, list(hedefler)

print("=== 1) KAPI KAPALI (1.5m), etap AKTIF -> beklemeli, konum sabitlenmeli ===")
d,h = kos("kapi 1.5m + iki yan duvar", kapi(1.5), True)
assert d.get('mod')=='KAYAR_ENGEL_BEKLIYOR', d
assert h and h[-1]==(3.0,1.0), f"konum sabitlenmedi: {h}"
print("     -> OK: bekliyor ve hedef = aracin KENDI konumu (3.0, 1.0)")

print("=== 2) KAPI KAPALI ama SAGDA GENIS GECIT VAR -> yine de BEKLEMELI (kacmamali) ===")
d,h = kos("kapi 1.5m + sag taraf tamamen acik", kapi(1.5, yan_bosluk=True), True)
assert d.get('mod')=='KAYAR_ENGEL_BEKLIYOR', d
assert h and h[-1]==(3.0,1.0), f"YANA KACTI: {h}"
print("     -> OK: yandaki gecidi GORMEZDEN geldi, yerinde bekledi")

print("=== 3) AYNI SAHNE ama etap KAPALI -> normal davranis (yandan gecmeli) ===")
d,h = kos("ayni sahne, kayar engel etabi KAPALI", kapi(1.5, yan_bosluk=True), False)
print(f"     -> mod={d.get('mod')} theta={d.get('theta')} (yana yonelmesi BEKLENIR)")
assert d.get('mod')!='KAYAR_ENGEL_BEKLIYOR'

print("=== 4) KAPI ACILDI -> duz gecmeli ===")
d,h = kos("kapi acik (onu bos)", acik, True)
assert d.get('mod')=='KAYAR_ENGEL', d
assert abs(d.get("theta"))<=3.0, d
print("     -> OK: theta=0 (duz), mesafe=", d.get('mesafe'))

print("=== 5) RAMPA MODU bu asamada TETIKLENMEMELI ===")
d,h = kos("govdeye dik duz yuzey 1.2m + etap AKTIF", kapi(1.2), True)
assert d.get('mod')!='RAMPA_YAKLASMA', f"RAMPA tetiklendi - kapiya SURERDI: {d}"
print(f"     -> OK: mod={d.get('mod')} (RAMPA_YAKLASMA DEGIL)")

print("=== 6) Etap kapaninca ayni yuzey RAMPA sayilir mi (regresyon) ===")
d,h = kos("ayni yuzey, etap KAPALI", kapi(1.2), False)
print(f"     -> mod={d.get('mod')} (normal mantik calisiyor)")
n.destroy_node(); rclpy.shutdown()
print("\nTUM TESTLER GECTI")
