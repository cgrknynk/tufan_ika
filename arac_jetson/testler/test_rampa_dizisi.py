"""8/Stop -> IMU tabanli rampa dizisi testi."""
# TEST IZOLASYONU (2026-09-11): bu test node'u SPIN ediyor. Spin
# acikken dugum GERCEK ROS agina (domain 77) baglanip calisan araçtan
# gelen /surus_modu, /arac_komut gibi mesajlari ALIYOR - testin kurdugu
# durumu eziyor ve testi rastgele dusuruyordu (canli olarak yasandi:
# surus_modu OTONOM yapiliyor, arac MANUEL yayinlayinca test kaliyordu).
# Ayri bir domain testi tamamen yalitir.
import os
os.environ['ROS_DOMAIN_ID'] = '99'
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
# NOT (2026-09-11): node /otonom_surus_aktif'i 1 Hz PERIYODIK tekrarliyor
# (kendini onarma). Spin acilinca bu tekrarlar da listeye dusuyor, bu
# yuzden "liste bos mu" testi anlamsiz. Gercek degismez sudur:
# TIRMANIS BOYUNCA HIC 'False' (durdurma) YAYINLANMAMALI.
def hic_durmadi(kayit):
    return not any(v is False for v in kayit)

def egim(d):
    yari=math.radians(d)/2.0
    o=Odometry(); o.pose.pose.orientation.w=math.cos(yari); o.pose.pose.orientation.y=-math.sin(yari)
    o.pose.pose.position.x=1.0
    n._son_odom=o; n._odom_cb(o)
def bekle(sn):
    # SPIN SART (2026-09-11): silah sonrasi devam gecikmesi bir ROS
    # zamanlayicisi (create_timer). Gercek node spin ediyor; test
    # etmiyordu, bu yuzden zamanlayici hic tetiklenmiyordu ve "otonom
    # geri acilmadi" gibi YANLIS bir basarisizlik uretiyordu.
    t=time.time()
    while time.time()-t<sn:
        egim(n._egim_derece)
        rclpy.spin_once(n, timeout_sec=0.0)
        time.sleep(0.05)

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
print(f"   otonom={ot}  (hic False olmamali = daha durmadi)")
assert ra and ra[-1] is True and hic_durmadi(ot)
print(f"   durum={n._rampa_durum} (1=TIRMANIS_BEKLE)"); assert n._rampa_durum==T._R_TIRMANIS_BEKLE

print("2) IMU +8 derece (esik 15 altinda) -> hala durmamali")
egim(8.0); bekle(0.4)
print(f"   otonom={ot} (hic False yok)"); assert hic_durmadi(ot)

# 2026-09-11 SARTNAME DEGISTI (kullanici): "dik egimde durmayi kapat,
# 8 veya stop gorunce durmasin, dik egimi ciksin, imu duz olunca dursun".
# Bu adimlar ESKIDEN tirmanista durmayi dogruluyordu - artik TAM TERSINI
# dogruluyorlar: egimde DURMAMALI, tepede DUZLESINCE durmali.
print("3) IMU +18 derece -> DURMAMALI, duzlesme beklenmeli")
egim(18.0); bekle(2.5)
print(f"   2.5 sn dik egimde otonom={ot} (hic False olmamali)")
assert hic_durmadi(ot), "TIRMANISTA DURDU - sartname ihlali"
print(f"   durum={n._rampa_durum} (6=DUZLESME_BEKLE)")
assert n._rampa_durum==T._R_DUZLESME_BEKLE

print("4) Tepede IMU duzlesti -> DUR + SILAH FAZI (9. tabela BEKLENMEDEN)")
ot.clear(); egim(1.0); bekle(0.5)
print(f"   0.5 sn duzluk (teyit {T.RAMPA_DUZLESME_GECIKMESI_S} sn) -> henuz durmamali: {ot}")
assert hic_durmadi(ot)
bekle(1.5)
print(f"   teyit sonrasi otonom={ot} (False olmali = DURDU)")
assert ot and ot[-1] is False
print(f"   durum={n._rampa_durum} (7=SILAH_BEKLE)")
assert n._rampa_durum==T._R_SILAH_BEKLE

print("5) 3 atis tamamlandi -> gecikmeli DEVAM, inis bekleniyor")
ot.clear()
n._hedef_vuruldu_cb(Int32(data=T.HEDEF_VURULDU_ESIGI))
bekle(T.SILAH_SONRASI_DEVAM_GECIKMESI_S + 0.6)
print(f"   otonom={ot} (True)"); assert ot and ot[-1] is True
print(f"   durum={n._rampa_durum} (3=INIS_BEKLE)"); assert n._rampa_durum==T._R_INIS_BEKLE

print("6) IMU -18 derece -> 2 sn sonra DUR")
ot.clear(); egim(-18.0); bekle(2.4)
print(f"   otonom={ot} (False)"); assert ot and ot[-1] is False
assert n._rampa_durum==T._R_ALTTA_DUR

print("7) 2 sn bekleme -> DEVAM, LIDAR tekrar AKTIF")
ot.clear(); ra.clear(); bekle(2.4)
print(f"   otonom={ot} (True)  rampa_etabi={ra} (False = lidar aktif)")
assert ot and ot[-1] is True
assert ra and ra[-1] is False
assert n._rampa_durum==T._R_BITTI
print(f"   -> PARKUR 1 {T.PARKUR1_BITIS_ILERLEME_S:.0f} sn sonra bitecek")
n.destroy_node(); rclpy.shutdown()
print("\nRAMPA DIZISI TESTI GECTI")
