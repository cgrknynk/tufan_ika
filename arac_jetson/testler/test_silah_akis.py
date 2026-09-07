"""9 -> silah OTONOM + arac MANUEL -> 3 atis -> arac OTONOM akisi."""
import sys, inspect
sys.path.insert(0,'/home/tufan-yer/tufan_ika_repo/arac_jetson/tufan_v2_ws/src')
import rclpy
from std_msgs.msg import String, Bool, Int32
from nav_msgs.msg import Odometry
import tabela_etap_yoneticisi as T
rclpy.init()
cls=[c for _,c in inspect.getmembers(T,inspect.isclass) if c.__module__==T.__name__][0]
n=cls()
k={'turret':[], 'silah':[], 'tabela':[], 'otonom':[], 'faz':[], 'surus':[], 'goal':[]}
n._turret_model_pub.publish  =lambda m: k['turret'].append(m.data)
n._silah_modu_pub.publish    =lambda m: k['silah'].append(m.data)
n._tabela_model_pub.publish  =lambda m: k['tabela'].append(m.data)
n._otonom_surus_pub.publish  =lambda m: k['otonom'].append(m.data)
n._silah_fazi_pub.publish    =lambda m: k['faz'].append(m.data)
n._surus_modu_pub.publish    =lambda m: k['surus'].append(m.data)
n._goal_pub.publish          =lambda m: k['goal'].append((round(m.pose.position.x,1),round(m.pose.position.y,1)))
n._etap_pub.publish=lambda m: None; n._parkur_pub.publish=lambda m: None
n._kayar_engel_pub.publish=lambda m: None
o=Odometry(); o.pose.pose.position.x=12.0; o.pose.pose.position.y=3.0; o.pose.pose.orientation.w=1.0
n._son_odom=o
def temizle():
    for v in k.values(): v.clear()

print("== ARAYUZ OTONOM iken 9. tabela ==")
n._surus_modu='OTONOM'; temizle()
for i in range(3): n._tespit_cb(String(data='Nine:0.93'))
print(f"  /silah_modu        = {k['silah']}          (OTONOM olmali)")
print(f"  /surus_modu        = {k['surus']}          (MANUEL olmali)")
print(f"  /silah_fazi_aktif  = {k['faz']}            (True olmali)")
print(f"  /turret_model      = {k['turret']}         (True olmali)")
print(f"  /tabela_model      = {k['tabela']}         (False - GPU turret'e)")
print(f"  yerinde dur hedefi = {k['goal']}")
assert k['silah']==['OTONOM'] and k['surus']==['MANUEL'] and k['faz']==[True]
assert k['turret']==[True] and k['tabela']==[False] and k['goal']==[(12.0,3.0)]
print("  -> OK")

print("== 3 ATIS TAMAMLANDI ==")
temizle()
n._hedef_vuruldu_cb(Int32(data=3))
print(f"  /turret_model      = {k['turret']}         (False olmali)")
print(f"  /silah_modu        = {k['silah']}          (MANUEL olmali)")
print(f"  /tabela_model      = {k['tabela']}         (True - on kamera geri)")
print(f"  /silah_fazi_aktif  = {k['faz']}            (False -> arayuz OTONOM'a doner)")
print(f"  /otonom_surus_aktif= {k['otonom']}         (True - hedef uretimi geri)")
print(f"  ileri hedef        = {k['goal']}")
assert k['turret']==[False] and k['silah']==['MANUEL'] and k['tabela']==[True]
assert k['faz']==[False] and k['otonom']==[True] and len(k['goal'])==1
print("  -> OK")
n.destroy_node(); rclpy.shutdown()
print("\nAKIS DOGRULANDI")
