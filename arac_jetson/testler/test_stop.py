"""Stop tabelasi -> 3 sn dur -> otonom devam testi."""
import sys, time, inspect
sys.path.insert(0,'/home/tufan-yer/tufan_ika_repo/arac_jetson/tufan_v2_ws/src')
import rclpy
from std_msgs.msg import String, Bool
from nav_msgs.msg import Odometry
import tabela_etap_yoneticisi as T
rclpy.init()
cls=[c for _,c in inspect.getmembers(T,inspect.isclass) if c.__module__==T.__name__][0]
n=cls()
k={'otonom':[], 'goal':[], 'turret':[]}
n._otonom_surus_pub.publish=lambda m: k['otonom'].append(m.data)
n._goal_pub.publish=lambda m: k['goal'].append((round(m.pose.position.x,1),round(m.pose.position.y,1)))
n._turret_model_pub.publish=lambda m: k['turret'].append(m.data)
for a in ('_etap_pub','_parkur_pub','_kayar_engel_pub','_silah_fazi_pub','_surus_modu_pub','_rampa_etabi_pub','_silah_modu_pub','_tabela_model_pub'):
    getattr(n,a).publish=lambda m: None
o=Odometry(); o.pose.pose.position.x=5.0; o.pose.pose.position.y=2.0; o.pose.pose.orientation.w=1.0
n._son_odom=o; n._surus_modu='OTONOM'

print("1) TEK Stop karesi -> tetiklenmemeli")
n._tespit_cb(String(data='Stop:0.9'))
print(f"   otonom_surus={k['otonom']} goal={k['goal']}  (bos olmali)")
assert not k['otonom'] and not k['goal']

print("2) 3 ARDISIK Stop -> DURMALI")
for i in range(2): n._tespit_cb(String(data='Stop:0.9'))
print(f"   otonom_surus={k['otonom']}   (False olmali)")
print(f"   hedef        ={k['goal']}    (aracin KENDI konumu)")
print(f"   turret       ={k['turret']}  (BOS olmali - silah fazi Stop'la baslamaz)")
assert k['otonom']==[False], k['otonom']
assert k['goal']==[(5.0,2.0)], k['goal']
assert not k['turret'], "Stop silah fazini baslatti - olmamali"

print("3) 3 saniye dolunca -> OTONOM DEVAM")
k['otonom'].clear()
t0=time.time()
while time.time()-t0 < 4.0:
    rclpy.spin_once(n, timeout_sec=0.1)
print(f"   otonom_surus={k['otonom']}   (True olmali)")
assert k['otonom']==[True], k['otonom']
gecen=time.time()-t0
print(f"   -> OK (bekleme {T.STOP_BEKLEME_S} sn)")

print("4) Bekleme bitince yeni etap tabelasi -> Stop tetigi yeniden kurulur")
n._tespit_cb(String(data='Five:0.9'))
print(f"   _stop_tetiklendi={n._stop_tetiklendi}  (False olmali)")
assert n._stop_tetiklendi == False
n.destroy_node(); rclpy.shutdown()
print("\nTUM TESTLER GECTI")
