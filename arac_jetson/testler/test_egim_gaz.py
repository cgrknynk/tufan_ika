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
assert tabelasiz < duz*1.2, "8. tabela YOKKEN tam gaz verildi!"
assert egimli > duz*1.5, "egimde gaz artmadi!"
assert abs(inis-duz)<1.0, "iniste de tam gaz verildi - TEHLIKELI"
assert abs(sol-sag)>1.0, "egimde direksiyon kayboldu"
print("TUM TESTLER GECTI")
n.destroy_node(); rclpy.shutdown()
