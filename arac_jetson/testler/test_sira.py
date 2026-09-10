"""Sirali tabela kabulu + komsu tabela korumasi + Stop egim sarti."""
import sys, time, math, inspect
sys.path.insert(0,'/home/tufan-yer/tufan_ika_repo/arac_jetson/tufan_v2_ws/src')
import rclpy
from std_msgs.msg import String, Bool, Int32
from nav_msgs.msg import Odometry
import tabela_etap_yoneticisi as T
rclpy.init()
cls=[c for _,c in inspect.getmembers(T,inspect.isclass) if c.__module__==T.__name__][0]
n=cls()
etaplar=[]; otonom=[]
n._etap_pub.publish=lambda m: etaplar.append(m.data)
n._otonom_surus_pub.publish=lambda m: otonom.append(m.data)
for a in ('_parkur_pub','_kayar_engel_pub','_silah_fazi_pub','_surus_modu_pub',
          '_rampa_etabi_pub','_silah_modu_pub','_tabela_model_pub','_turret_model_pub','_goal_pub'):
    getattr(n,a).publish=lambda m: None
o=Odometry(); o.pose.pose.position.x=1.0; o.pose.pose.orientation.w=1.0
n._son_odom=o; n._surus_modu='OTONOM'
def egim(d):
    yari=math.radians(d)/2.0
    oo=Odometry(); oo.pose.pose.orientation.w=math.cos(yari); oo.pose.pose.orientation.y=-math.sin(yari)
    oo.pose.pose.position.x=1.0
    n._son_odom=oo; n._egimi_guncelle(oo)
def gor(c,k=3):
    for i in range(k): n._tespit_cb(String(data='%s:0.9'%c))
egim(0.0)

print("1) SIRA DISI tabela (once 5 gonder) -> kabul edilmemeli")
gor('Five'); print(f"   etaplar={etaplar}  (bos olmali)"); assert not etaplar

print("2) SIRADAKI tabela (1) -> kabul")
gor('One'); print(f"   etaplar={etaplar}"); assert etaplar==[1]

print("3) 1 HALA KADRAJDA iken 2 -> BEKLETILMELI")
n._tespit_cb(String(data='One:0.9'))      # 1 hala goruluyor
gor('Two'); print(f"   etaplar={etaplar}  (hala [1] olmali)"); assert etaplar==[1]

print("4) 1 KADRAJDAN CIKTI (1.1 sn) -> 2 kabul")
time.sleep(1.1); gor('Two')
print(f"   etaplar={etaplar}"); assert etaplar==[1,2]

print("5) 3..8 sirayla")
for c in ('Three','Four','Five','Six','Seven','Eight'):
    time.sleep(1.05); gor(c)
print(f"   etaplar={etaplar}"); assert etaplar==[1,2,3,4,5,6,7,8]

print("6) Stop -> ARTIK dogrudan durdurmuyor, RAMPA DIZISI tetigi kuruyor")
# 2026-09-07: Stop'un eski "3 sn dur" davranisi KALDIRILDI. Parkurda
# 8+Stop yan yana oldugu icin Stop'un gorulme ani aracin nerede oldugunu
# soylemiyor - durus karari artik IMU'ya bagli (bkz. test_rampa_dizisi.py).
time.sleep(1.05); otonom.clear(); gor('Stop',3)
print(f"   otonom={otonom}  (bos olmali - IMU esigi asilmadi)")
assert not otonom, "Stop dogrudan durdurdu - artik oyle olmamali"
print(f"   rampa durumu={n._rampa_durum} (1=TIRMANIS_BEKLE olmali)")
assert n._rampa_durum == T._R_TIRMANIS_BEKLE
print(f"   sirada: {T.BEKLENEN_SIRA[n._sira_index]}  (Nine olmali)")
assert T.BEKLENEN_SIRA[n._sira_index]=='Nine'
n.destroy_node(); rclpy.shutdown()
print("\nTUM TESTLER GECTI")
