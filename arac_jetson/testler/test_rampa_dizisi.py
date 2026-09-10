"""8/Stop -> IMU tabanli rampa dizisi testi."""
import sys, time, math, inspect
sys.path.insert(0,'/home/tufan-yer/tufan_ika_repo/arac_jetson/tufan_v2_ws/src')
import rclpy
from std_msgs.msg import String, Bool, Int32
from nav_msgs.msg import Odometry
import tabela_etap_yoneticisi as T
rclpy.init()
cls=[c for _,c in inspect.getmembers(T,inspect.isclass) if c.__module__==T.__name__][0]
n=cls()
ot=[]; ra=[]; par=[]
n._otonom_surus_pub.publish=lambda m: ot.append(m.data)
n._rampa_etabi_pub.publish=lambda m: ra.append(m.data)
n._parkur_pub.publish=lambda m: par.append(m.data)
for a in ('_etap_pub','_kayar_engel_pub','_silah_fazi_pub','_surus_modu_pub',
          '_silah_modu_pub','_tabela_model_pub','_turret_model_pub','_goal_pub','_hizlanma_pub'):
    getattr(n,a).publish=lambda m: None
n._surus_modu='OTONOM'
def egim(d):
    yari=math.radians(d)/2.0
    o=Odometry(); o.pose.pose.orientation.w=math.cos(yari); o.pose.pose.orientation.y=-math.sin(yari)
    o.pose.pose.position.x=1.0
    n._son_odom=o; n._odom_cb(o)
def bekle(sn):
    t=time.time()
    while time.time()-t<sn:
        egim(n._egim_derece); time.sleep(0.05)

# Rampa bolgesine gel (Stop ancak sirada 8+ iken kabul ediliyor)
egim(0.0)
for c in ('One','Two','Three','Four','Five','Six','Seven'):
    for i in range(3): n._tespit_cb(String(data='%s:0.9'%c))
    time.sleep(1.05)
print(f"   sirada: {T.BEKLENEN_SIRA[n._sira_index]} (Eight olmali)")
assert T.BEKLENEN_SIRA[n._sira_index]=='Eight'

print("1) Stop goruldu (8 ile yan yana) -> LIDAR KOR olmali, durus YOK")
ra.clear(); ot.clear()
for i in range(3): n._tespit_cb(String(data='Stop:0.95'))
print(f"   rampa_etabi={ra}  (True olmali = LIDAR kor)")
print(f"   otonom={ot}       (bos olmali = daha durmadi)")
assert ra and ra[-1] is True and not ot
print(f"   durum={n._rampa_durum} (1=TIRMANIS_BEKLE)"); assert n._rampa_durum==T._R_TIRMANIS_BEKLE

print("2) IMU +8 derece (esik 15 altinda) -> hala durmamali")
egim(8.0); bekle(0.4)
print(f"   otonom={ot} (bos)"); assert not ot

print("3) IMU +18 derece -> 2 sn sonra DURMALI")
egim(18.0); bekle(0.5)
print(f"   0.5 sn sonra otonom={ot} (henuz bos)"); assert not ot
bekle(1.8)
print(f"   2.3 sn sonra otonom={ot} (False olmali)"); assert ot and ot[-1] is False
assert n._rampa_durum==T._R_TEPEDE_DUR

print("4) 2 sn bekleme -> DEVAM, inis bekleniyor")
ot.clear(); bekle(2.3)
print(f"   otonom={ot} (True)"); assert ot and ot[-1] is True
print(f"   durum={n._rampa_durum} (3=INIS_BEKLE)"); assert n._rampa_durum==T._R_INIS_BEKLE

print("5) IMU -18 derece -> 2 sn sonra DUR")
ot.clear(); egim(-18.0); bekle(2.4)
print(f"   otonom={ot} (False)"); assert ot and ot[-1] is False
assert n._rampa_durum==T._R_ALTTA_DUR

print("6) 2 sn bekleme -> DEVAM, LIDAR tekrar AKTIF")
ot.clear(); ra.clear(); bekle(2.4)
print(f"   otonom={ot} (True)  rampa_etabi={ra} (False = lidar aktif)")
assert ot and ot[-1] is True
assert ra and ra[-1] is False
assert n._rampa_durum==T._R_BITTI
print(f"   -> PARKUR 1 {T.PARKUR1_BITIS_ILERLEME_S:.0f} sn sonra bitecek")
n.destroy_node(); rclpy.shutdown()
print("\nRAMPA DIZISI TESTI GECTI")
