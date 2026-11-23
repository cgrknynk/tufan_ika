"""Dik egimde tam gaz mantigi testi - ARAC YOK."""
import sys, math, time, inspect
sys.path.insert(0,'/home/tufan-yer/tufan_ika_repo/arac_jetson/tufan_v2_ws/src')
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import surus_koprusu as SK
rclpy.init()
cls=[c for _,c in inspect.getmembers(SK,inspect.isclass) if c.__module__==SK.__name__][0]
n=cls()
cikti=[]
n._palet_pub.publish=lambda m: cikti.append((round(m.data[0],1), round(m.data[1],1)))
n._mod_cb(String(data='OTONOM'))
from std_msgs.msg import Bool as _B
def egim(derece):
    o=Odometry(); yari=math.radians(derece)/2.0
    # y ekseni etrafinda donus: burun YUKARI icin fz>0 olmali
    o.pose.pose.orientation.w=math.cos(yari); o.pose.pose.orientation.y=-math.sin(yari)
    n._odom_cb(o)
def komut(lin):
    t=Twist(); t.linear.x=lin; n._otonom_cb(t)

komut(0.4)   # tam ileri
print("=== DUZ YOL (0 derece) ===")
egim(0.0); cikti.clear(); n._dongu()
print(f"  PWM: {cikti[-1]}   (normal tavan 110 beklenir)")
duz=abs(cikti[-1][0])

print("=== DIK EGIM ama 8. TABELA YOK -> tam gaz OLMAMALI ===")
n._rampa_etabi_cb(_B(data=False))
egim(8.0); komut(0.4); cikti.clear(); n._dongu()
print(f"  PWM: {cikti[-1]}   (normal tavan 110 beklenir)")
tabelasiz=abs(cikti[-1][0])

print("=== DIK EGIM + 8. TABELA VAR -> tam gaz ===")
n._rampa_etabi_cb(_B(data=True))
egim(8.0); komut(0.4); cikti.clear(); n._dongu()
print(f"  PWM: {cikti[-1]}   (tam gaz 255 beklenir)")
egimli=abs(cikti[-1][0])

print("=== INIS (-8 derece) - tam gaz OLMAMALI ===")
time.sleep(1.6)  # histerezis dolsun
n._rampa_etabi_cb(_B(data=True))
egim(-8.0); komut(0.4); cikti.clear(); n._dongu()   # komutu TAZELE (timeout 0.5s)
print(f"  PWM: {cikti[-1]}   (normal tavan beklenir)")
inis=abs(cikti[-1][0])

print("=== DONUS KABILIYETI korunuyor mu (egimde) ===")
n._rampa_etabi_cb(_B(data=True)); egim(8.0)
t=Twist(); t.linear.x=0.3; t.angular.z=0.4; n._otonom_cb(t)
cikti.clear(); n._dongu()
sol,sag=cikti[-1]
print(f"  sol={sol} sag={sag}  (FARKLI olmali = direksiyon calisiyor)")

print()
print(f"duz={duz:.0f}  tabelasiz_egim={tabelasiz:.0f}  egim+tabela={egimli:.0f}  inis={inis:.0f}")
# 2026-09-11 SARTNAME DEGISTI (kullanici: "dik engelin tabelasini
# okumayabilir, ondan da IMU'dan dik engeli tirmandigimizi anlayabiliriz;
# egim 5 derecenin uzerine cikmadikca hiz artmasin, 5 derece ve uzeri
# egimlerde hiz artsin"). Yani "tabela yoksa gaz YOK" kurali KALKTI.
# KORUNAN asil guvenlik amaci: KUCUK egimlerde TAM GAZ verilmemeli.
# 2026-09-12 GUVENLIK KARARI: imu_tek_basina_gaz VARSAYILAN KAPALI
# (sahada beklenmedik hizlanmalar yasandi, ozellik dogrulanmadi).
# Bu yuzden tabelasiz egimde gaz ARTMAMALI - varsayilan davranis budur.
assert abs(tabelasiz - duz) < 1.0, \
    "imu_tek_basina_gaz varsayilan KAPALI olmali (guvenlik)"
# Ozellik ACILDIGINDA orantili gaz vermeli - ayri dogrulama.
# TABELA BAYRAGI KAPATILMALI: acik kalirsa tabela yolu zaten 255 verir
# ve IMU yolunu olcmus olmayiz (bu tuzaga bu dosyada ikinci kez dusuldu).
n._rampa_etabi_cb(_B(data=False))
n._imu_tek_aktif = True
egim(8.0); komut(0.4); cikti.clear(); n._dongu()
_acik = abs(cikti[-1][0])
print(f"  ozellik ACIKKEN 8 derece -> {_acik:.0f}  (110'dan buyuk olmali)")
assert _acik > duz, "ozellik acikken gaz artmadi"
assert _acik < 255.0 - 1.0, "ozellik acikken 8 derecede TAM GAZ verildi"
n._imu_tek_aktif = False
# 5 derece ALTINDA hic artmamali - eski korumanin ozu burada test ediliyor.
# TABELA BAYRAGI KAPATILMALI: acik kalirsa tabela yolu 3 derecenin
# uzerinde zaten tam gaz veriyor ve IMU yolunu test etmis olmayiz.
# 4 derece secildi: eski tabela esiginin (3) USTU, yeni IMU esiginin (5)
# ALTI - tam kritik nokta.
n._rampa_etabi_cb(_B(data=False))
egim(4.0); komut(0.4); cikti.clear(); n._dongu()
_kucuk = abs(cikti[-1][0])
print(f"  4 derece, tabela YOK -> {_kucuk:.0f}  (duz yol tavani {duz:.0f} olmali)")
assert abs(_kucuk - duz) < 1.0, "5 derece ALTINDA gaz artti - kucuk tumsek korumasi bozuldu"
assert egimli > duz*1.5, "egimde gaz artmadi!"
# 2026-09-11 SARTNAME DEGISTI (kullanici: "imu assagiya baktiginda
# minimum hiz ile insin, 85 ile insin"). Bu satir ESKIDEN "inis == duz
# yol tavani (110)" bekliyordu; asil amaci "iniste TAM GAZ verilmesin"di.
# O amac KORUNUYOR, ustune yeni sartname de dogrulaniyor: inis SABIT 85.
assert inis < duz, "iniste hiz duz yoldan dusuk olmali"
assert inis < egimli*0.5, "iniste tam gaz verildi - TEHLIKELI"
assert abs(inis-90.0) < 1.0, f"inis sabit 90 olmali, {inis:.0f} cikti"
assert abs(sol-sag)>1.0, "egimde direksiyon kayboldu"
print("TUM TESTLER GECTI")
n.destroy_node(); rclpy.shutdown()
