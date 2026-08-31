#!/usr/bin/env python3
"""Turret (hedef takip) ROS 2 dugumu.

Kamera + YOLO modeli ile hedefi tespit edip 2 eksenli turret'i (pan/tilt)
hedefe dogru yonlendirir.

ONEMLI: Bu dugum artik silah_ws/sketch_jul28b.ino'daki GERCEK Arduino
firmware'inin protokolune gore calisir - UC ayri komut turu:
  - SUREKLI HAREKET (KABA mod, hedeften uzakken): 2 bayt, yon karakteri
    ('w'/'a'/'s'/'d') + hiz (0-255). 'x' (dur) tek bayt, hiz gerekmez.
    "Heartbeat" modeli: bir yon komutu geldiginde o yonde SUREKLI
    (non-blocking) adim atmaya devam eder; ayni komut tekrar tekrar
    gonderilirse hareket kesintisiz surer. Firmware 250ms boyunca yeni
    komut/heartbeat almazsa kendisi otomatik durur (donanim seviyesi
    guvenlik). Ayni anda sadece TEK eksen hareket eder.
  - TEK ADIM (HASSAS mod, hedefe yakinken): 2 bayt, 'p' + yon karakteri.
    Heartbeat GEREKTIRMEZ - firmware TEK bir step-pulse'u uretip hemen
    durur (bkz. tekAdimAt). Boylece duzeltme buyuklugu firmware'in en
    yavas surekli-hiz modundan degil, Jetson'un kare hizindan (kare basina
    EN FAZLA 1 fiziksel adim) sinirlanir - surekli mod, iki kare arasinda
    (frame rate ~10-15fps) onlarca adim atip kucuk tolerans pencerelerini
    (birkac px) her zaman asabiliyordu (sahada gozlemlendi). Bkz.
    _tek_adim_gonder / HASSAS SURUNME asagida.
  - Lazer (pin 9 -> role kontrol girisi, role AKTIF-DUSUK) hareketten
    TAMAMEN bagimsiz kendi komutu ('l' ac, 'k' kapat) ve kendi 250ms
    guvenlik zaman asimiyla kontrol edilir.
  - Arduino'ya ayrica bir TF02-Pro lidar (donanimsal Serial3, pin 14/15)
    bagli; olculen mesafe "D:<cm>\\n" seklinde USB seri hat uzerinden
    Jetson'a geri raporlanir - taze/guvenilirse (medyan+EMA filtreli,
    bkz. asagida) paralaks hesabinda GERCEKTEN kullanilir.

PARALAKS DUZELTME (trigonometri + lidar mesafesi + sahada olculmus trim):
Kamera ve lazer DIKEY olarak ust uste (ayni eksende, kamera_lazer_dikey_mm
kadar oteleme) monte edilmis. Lazerin hedefe TAM isabet etmesi icin
goruntude nisan alinmasi gereken nokta (aim_center'dan dx,dy kadar
kaydirilmis) uc bilesenden olusur: (1) trigonometri: aci = atan2(offset_mm,
mesafe_mm), piksel_kaymasi = tan(aci) * kamera_odak_px - mesafe olarak
lidar TAZEYSE (mesafe_timeout_s icinde) olculen deger, degilse sabit
paralaks_sabit_mesafe_cm kullanilir; (2) nisan_bias_x/y_px: sahada olculup
eklenen SABIT (mesafeden bagimsiz) bir trim - mm-formulunun modellemedigi
kucuk mekanik acisal sapmayi telafi eder. kamera_odak_px, OpenCV
checkerboard kalibrasyonuyla (tools/camera_calibrate.py) OLCULMUS gercek
odak uzakligidir (tahmin degil). Lidar ham okumalari once bir minimum-
gecerlilik esiginden (lidar_gecersiz_esik_cm), sonra bir medyan penceresinden
(tekil "spike" okumalara dayaniklilik icin) ve son olarak EMA'dan gecer -
bu katmanlar olmadan ham lidar gurultusu nisan noktasini hedefle alakasiz
yerlere zipratiyordu (sahada gozlemlendi). Bkz. _paralaks_ofseti_hesapla.

MANUEL modda: /turret_manuel_cmd'den gelen son yon, kamera/YOLO donguisunden
BAGIMSIZ, ayri ve sabit hizli bir zamanlayici (_manuel_heartbeat, ~20Hz) ile
Arduino'ya SABIT TAM HIZDA (255) heartbeat olarak gonderilir - boylece tus
tepkisi YOLO cikarim suresine bagli kalmaz. MANUEL modda lazer HICBIR ZAMAN
otomatik yanmaz.

Otonom modda hareket, hedef (YOLO) konumu ile paralaks-duzeltilmis nisan
noktasi arasindaki hataya gore IKI KADEMELI calisir - KABA mod ile HASSAS
mod arasindaki gecis, KENDI HISTEREZISLI esik ciftiyle (ince_ayar_toleransi_px
girmek icin siki, ince_ayar_kayip_toleransi_px cikmak icin genis) belirlenir,
_hassas_mod state'i:
  - KABA mod (hata BUYUK): SUREKLI HAREKET, bir PID kontrolcusunden gelen
    DEGISKEN hizla tek bir yon komutu ('w'/'a'/'s'/'d') gonderilir (hata
    kuculdukce yavaslar, boylece asiri duzeltip titreme/oscillation yapmaz).
  - HASSAS mod (hata KUCUK): TEK ADIM ('p'+yon), kare basina en fazla 1
    fiziksel adim - hassas_epsilon_px'ten kucuk hata icin hic komut
    gonderilmez (zaten "yeterince iyi").
Lazer acma/kapama (kilit) karari, hassas_epsilon_px (kilide GIRMEK icin,
HASSAS modda VE hata artik duzeltmeye deger olmayacak kadar kucukken) ile
kilit_kaybi_esigi_px (kilidi KAYBETMEK icin, daha genis - histerezis, kucuk
gurultu lazeri surekli acip kapatmasin) arasinda verilir. KILITLIYKEN
(_kilitli_mi=True, lazer ACIK) turret HICBIR hareket komutu (ne kaba ne
hassas) GONDERMEZ - tam durgunluk sarttir, lazer sadece turret GERCEKTEN
durmusken yanar (sahada "lazer acikken hareket edebiliyor" sorunu boyle
cozuldu). KILIT BEKLEME (kilit_bekleme_s, varsayilan 5s): kilit
SAGLANDIKTAN sonra bu sure boyunca lazer ACIK / motorlar DURGUN GARANTI
kalir - bu pencere icinde kucuk pozisyon dalgalanmalari kilidi bozamaz
(hedef TAMAMEN kaybolmadikca). PID kazanclari (pid_kp/pid_ki/pid_kd) ve
hiz sinirlari (pid_min_hiz/pid_max_hiz, SADECE kaba modu etkiler) ROS
parametreleridir, `ros2 param set /turret_node <isim> <deger>` ile yeniden
baslatmadan canli ayarlanabilir.
"""

import math
import os
import queue
import socket
import statistics
import threading
import time
from collections import deque

import cv2
import numpy as np
import rclpy
import serial
import serial.tools.list_ports
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import String, Bool, Int32
from ultralytics import YOLO

# Silahin turret Arduino'su (silah_ws/sketch_jul28b.ino) bir Arduino MEGA
# 2560 - tekerlek motor kartindan (Uno) FARKLI bir VID:PID'e sahip, bu
# yuzden ikisi USB'de ayni anda takiliyken birbirine karismiyor.
ARDUINO_MEGA_VID_PID = {
    (0x2341, 0x0042),  # Mega 2560 R3
    (0x2341, 0x0010),  # Mega 2560 (eski bootloader)
    (0x2A03, 0x0042),  # Mega 2560 R3 (bazi resmi lisansli klonlar)
}

# Varsayilan model yolu: launch dosyasi olmadan (ör. "ros2 run tufan_v2_ws
# turret_node.py") dogrudan calistirildiginda da model_path bos kalmasin diye.
_VARSAYILAN_MODEL_YOLU = os.path.join(
    get_package_share_directory('tufan_v2_ws'), 'models', 'hedefv3-seg.pt'
)

# HASSAS TEK-ADIM modunda "sicrama bastirma" icin: bir adimin ekseni+yonu,
# BIR ONCEKI adimin TAM TERSIYSE (orn. once 's' sonra 'w'), bu GENELDE bir
# onceki adimin hedefi HAFIFCE gectigini (overshoot) gosterir - fiziksel
# adim boyutu piksel-hassasiyetinden buyukse, tam merkeze asla oturamayan
# sonsuz bir "asagi-yukari-asagi-yukari" sicramaya sebep olur (sahada
# gozlemlendi). Ters yondeki adimi ALMAYIP mevcut konumu kabul etmek,
# gercek konumu fiziksel adim cozunurlugunun izin verdigi EN YAKIN noktaya
# (yani zaten en iyi ulasilabilir hassasiyete) sabitler - bkz. _ana_dongu
# HASSAS SURUNME blogu ve _hassas_son_yon.
_TERS_YON = {'w': 's', 's': 'w', 'a': 'd', 'd': 'a'}


class TurretNode(Node):
    def __init__(self):
        super().__init__('turret_node')

        # camera_index SADECE kamera_arama_ismi ile eslesen bir cihaz
        # BULUNAMAZSA kullanilir (yedek). Birden fazla kamera bagliyken
        # (orn. C270'ler de takiliyken) /dev/videoN numaralari degisebilir/
        # ongorulemez olabilir - bu yuzden varsayilan davranis, ismi
        # kamera_arama_ismi (varsayilan "C922") GECEN ilk /dev/videoN
        # cihazini OTOMATIK bulmaktir (bkz. _kamera_indexini_otomatik_bul).
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('kamera_arama_ismi', 'C922')
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('model_path', _VARSAYILAN_MODEL_YOLU)
        # TensorRT (.engine) export'lari genelde SABIT bir giris boyutuyla
        # derlenir (bkz. optimize.py'deki model.export - imgsz belirtilmedi,
        # varsayilan 640) - calisma zamaninda FARKLI bir imgsz vermek shape
        # hatasina sebep olabilir, bu yuzden modelin derlendigi boyutla
        # AYNI olmali. Farkli bir model/export kullanilirsa buna gore
        # degistirilmeli.
        self.declare_parameter('model_imgsz', 800)
        self.declare_parameter('hedef_confidence', 0.20)
        # INCE AYAR: hedeften uzakken (hata BUYUK) KABA surekli-heartbeat
        # modu, hedefe yaklasinca (hata bu histerezis ciftinin icine
        # girince) HASSAS TEK-ADIM moduna gecilir (bkz. tekAdimAt
        # firmware'de, _tek_adim_gonder burada) - boylece duzeltme buyuklugu
        # firmware'in en yavas hizindan degil, Jetson'un kare hizindan (kare
        # basina EN FAZLA 1 fiziksel adim) sinirlanir. ONEMLI: HISTEREZISLI
        # olmali - SADECE tek bir sabit esik kullanilirsa hata tam esik
        # sinirinda salinip surekli mod gecisine (osilasyon) sebep olur.
        # GUNCELLEME (sahada 10m'de test edildi): "adim adim donsun, PID
        # salinimi hala var" geri bildirimiyle bu esikler genisletildi -
        # boylece hataya SUREKLI PID yerine (overshoot/osilasyon riski)
        # cok daha genis bir aralikta HASSAS TEK-ADIM modu (kare basina en
        # fazla 1 fiziksel adim, artik 12800 pulses/rev mikro-adimla COK
        # ince) devreye giriyor - eski 8/15px degerleri cok dardi.
        self.declare_parameter('ince_ayar_toleransi_px', 25.0)
        self.declare_parameter('ince_ayar_kayip_toleransi_px', 35.0)
        # HASSAS SURUNME: bu esikten KUCUK hata icin (fiziksel adim
        # boyutunun cozemeyecegi kadar kucuk) hic komut gonderilmez - "yeterince
        # iyi" kabul edilir.
        self.declare_parameter('hassas_epsilon_px', 0.8)
        # HASSAS moddaki hatayi (px) DOGRUDAN adim sayisina cevirir - bkz.
        # _coklu_adim_gonder. Baslangic degeri KASITLI dusuk/temkinli
        # secildi (asiri gitme riskini azaltmak icin); sahada px-basina-
        # gercek-adim orani olculup buna gore yukseltilebilir/dusurulebilir
        # (ros2 param set ile canli).
        self.declare_parameter('hassas_adim_kazanci', 0.5)
        # Tek bir HASSAS komutunda gonderilebilecek EN FAZLA adim sayisi -
        # kazanc yanlislikla cok yuksek girilse bile tek kare buyuk bir
        # sicrama yapmasin diye guvenlik tavani.
        self.declare_parameter('hassas_maks_adim_burst', 20)
        # LAZER/KILIT KARARI: lazer SADECE turret GERCEKTEN durmus VE tam
        # merkezdeyken yanar - kilide GIRMEK icin esik dogrudan
        # hassas_epsilon_px'tir (yukarida, "artik duzeltmeye gerek yok"
        # noktasi), kilidi KAYBETMEK icin ise daha GENIS bu esik kullanilir
        # (histerezis - kucuk gurultu kilidi/lazeri surekli acip
        # kapatmasin). Kilitliyken (_kilitli_mi=True) turret HICBIR hareket
        # komutu (ne kaba ne hassas) GONDERMEZ - "lazer acikken hareket
        # edebiliyor" sorunu sahada gozlemlenip boyle duzeltildi.
        self.declare_parameter('kilit_kaybi_esigi_px', 2.5)
        # KILIT BEKLEME: hedef VURULUP (kilit saglanip) lazer ACILDIKTAN
        # sonra, en az bu kadar saniye lazer ACIK / motorlar DURGUN kalir -
        # bu sure icinde kucuk pozisyon dalgalanmalari kilidi/lazeri
        # KESINLIKLE bozamaz (hedef tamamen kaybolmadikça). Sure dolunca
        # normal kilit/kayip mantigina doner.
        self.declare_parameter('kilit_bekleme_s', 5.0)
        # Kilit (5sn bekleme) BITTIKTEN sonra, hareket komutlari bu kadar
        # (s) daha bastirilir - lazer bu sureyi BEKLEMEZ, kilit bitince
        # hemen soner, sadece HAREKET birkac yuz milisaniye ertelenir. Sahada
        # gozlemlendi: kilit tam bittigi anda hemen o anki (henuz
        # durulmamis) hataya tepki verilmesi kucuk bir "sicrama" yapiyordu -
        # bu kisa ek bekleme, olcumun/hedefin durulmasina pay tanir.
        self.declare_parameter('kilit_sonrasi_bekleme_s', 0.4)
        self.declare_parameter('nudge_araligi', 0.03)
        self.declare_parameter('komut_timeout', 0.5)
        self.declare_parameter('x_yonu_ters_mi', False)
        self.declare_parameter('y_yonu_ters_mi', False)
        self.declare_parameter('motor_eksenlerini_yer_degistir', False)
        self.declare_parameter('video_target_ip', '10.40.64.48')
        self.declare_parameter('video_target_port', 5000)
        # PID: hataya (piksel) orantili degisken hiz uretir - eskiden hep
        # sabit tam hizdi, bu da merkeze yaklasinca asiri duzeltip titremeye
        # (oscillation) sebep oluyordu. ros2 param set /turret_node pid_kp ...
        # ile yeniden baslatmadan canli ayarlanabilir.
        # GUNCELLEME (sahada test edildi): kp/kd dusurulerek KABA moddaki
        # kalan salinim/titreme azaltildi - bkz. yukarida ince_ayar_*
        # esiklerinin genisletilmesi (artik hedefe yakinken zaten HASSAS
        # tek-adim moduna geciliyor, KABA mod SADECE uzak/buyuk hatalarda
        # calisiyor).
        self.declare_parameter('pid_kp', 0.6)
        self.declare_parameter('pid_ki', 0.0)
        self.declare_parameter('pid_kd', 0.05)
        self.declare_parameter('pid_min_hiz', 5)
        self.declare_parameter('pid_max_hiz', 255)
        self.declare_parameter('pid_hata_doygunlugu_px', 150.0)
        # Hedef merkez noktasi kare-kare kucuk miktarlarda titreyebiliyor
        # (YOLO algilama gurultusu) - PID bu gurultuyu ne kadar iyi
        # ayarlanirsa ayarlansin kovalar. Bunu PID'e girmeden once bir
        # ustel yumusatma (EMA) ile filtreliyoruz. 1.0 = yumusatma yok
        # (ham deger), kucuk deger = cok yumusak ama gec tepki.
        self.declare_parameter('konum_yumusatma_alpha', 0.08)
        # PARALAKS DUZELTME: kamera ve lazer dikey eksende ust uste (sadece
        # oteleme, acisal sapma yok, kamera_lazer_dikey_mm kadar). Mesafe
        # olarak, taze/guvenilirse (medyan penceresi + EMA ile agir
        # filtrelenmis, bkz. _arduino_satir_isle) lidar okumasi, degilse
        # sabit paralaks_sabit_mesafe_cm kullanilir. kamera_odak_px, OpenCV
        # checkerboard kalibrasyonuyla (tools/camera_calibrate.py) OLCULMUS
        # gercek odak uzakligidir (tahmin degil). Bkz. _paralaks_ofseti_hesapla.
        self.declare_parameter('kamera_lazer_dikey_mm', 55.0)
        self.declare_parameter('kamera_lazer_yatay_mm', 0.0)
        self.declare_parameter('kamera_odak_px', 296.92)
        # Lidar verisi yoksa/eskiyse (mesafe_timeout_s'den eski) paralaks
        # hesabinda (VE _beklenen_hedef_yaricapi_px'te, bkz. asagida OPENCV
        # ON-TESPIT) kullanilacak varsayilan mesafe - gercek hedefleme
        # mesafesine yakin secilmeli (ros2 param set ile canli ayarlanabilir).
        # Model 10m'de egitildigi icin (kullanicinin bildirdigi test mesafesi)
        # varsayilan 1000cm.
        self.declare_parameter('paralaks_sabit_mesafe_cm', 1000.0)
        self.declare_parameter('mesafe_timeout_s', 1.0)
        # BORESIGHT TRIM: teorik mm/aci hesabi mekanik montajdaki KUCUK
        # acisal sapmalari (kamera-lazer tam paralel degilse) ya da
        # turret'in mekanik "merkez" konumunun kameranin gordugu goruntu
        # merkeziyle birebir ortusmemesini modellemez. Bu artik kalan hata
        # sahada gozlemlenip (lazer hedefin ne kadar/hangi yone kaydigina
        # bakilarak) bu iki bias parametresiyle EMPIRIK olarak sifirlanir -
        # teorik hesaba (dx,dy) DOGRUDAN eklenir, ros2 param set ile canli
        # ayarlanir. Ornek: lazer hedefin USTUNE vuruyorsa (goruntude hedef
        # olmasi gerekenden daha AZ asagi kaydirilmis demektir) nisan_bias_y_px
        # ARTIRILIR (referans daha da asagi kayar).
        # GUNCELLEME: 10m'de TILT backlash telafisi (bkz. firmware) + DM556
        # akim ayari duzeltmesinden SONRA yeniden kalibre edildi (sahada
        # ikili arama ile: 6.8/8.8 arasi -> 7.8 -> 8.3 X icin dogrulandi,
        # Y tek denemede -14.3 ile tam oturdu).
        self.declare_parameter('nisan_bias_x_px', 8.3)
        self.declare_parameter('nisan_bias_y_px', -14.3)
        # GUVENLIK: TF02-Pro ara sira (yansima/multipath, checksum'dan gecen
        # ama fiziksel olarak imkansiz) COK yakin bir mesafe raporlayabiliyor
        # (sahada 6-36cm gibi degerler gorduk) - bu esigin ALTINDAKI ham
        # okumalar yok sayilir (paralaks hesabinda GERCEKTEN kullanildigi
        # icin bu filtre artik sadece kozmetik degil, guvenlik onemli).
        self.declare_parameter('lidar_gecersiz_esik_cm', 30.0)
        # GUVENLIK: sahada gozlemlendi ki TF02-Pro, 30cm esiginin UZERINDE
        # ama gercek mesafeden COK sapan tekil ("spike") okumalar da
        # uretebiliyor (yansima/multipath, orn. 580cm dizisi icinde tek bir
        # 153cm okumasi). EMA tek basina buna karsi yeterince dayanikli
        # degil (her yeni ornege sabit agirlik verir). Bunun onune gecmek
        # icin ham okuma once son N ornegin MEDYANI ile degistirilir (medyan,
        # tekil aykiri degerlere EMA'dan cok daha dayaniklidir), SONRA EMA
        # uygulanir.
        self.declare_parameter('lidar_medyan_pencere_n', 5)
        # GUVENLIK: yanlislikla girilen bir parametre (orn. cok kucuk bir
        # paralaks_sabit_mesafe_cm) ile bile hesaplanan paralaks kaymasi
        # (dx, dy) bu piksel sinirinin disina asla cikamaz.
        self.declare_parameter('paralaks_maks_ofset_px', 100.0)
        # Lidar'in dar huzmesi kucuk turret hareketlerinde bazen hedefi
        # bazen arka plani olctugu icin ham deger paralaks hesabinda
        # kullanilmadan once ustel yumusatma (EMA) ile filtrelenir.
        # Kucuk deger = cok yumusak.
        self.declare_parameter('mesafe_yumusatma_alpha', 0.15)
        # YOLO tek bir karede hedefi kacirabilir (hedef hala oradayken bile,
        # esik-siniri confidence, motion blur vb.) - bu TEK KARELIK kacirma
        # yuzunden konum/mesafe yumusatma durumunu hemen sifirlamiyoruz.
        # Hedef bu sureden (s) daha UZUN suredir hic gorulmediyse GERCEKTEN
        # kaybedildigi kabul edilip sifirlanir.
        self.declare_parameter('hedef_kayip_esigi_s', 1.5)

        # OPENCV ON-TESPIT: hedef, bilinen oranli (6cm/12cm/18cm yaricap,
        # 1:2:3) ic ice siyah halkalardan olusan sabit bir "puan tahtasi"
        # seklinde. ILK DENEME (HoughCircles / kontur+esikleme ile kenar/
        # sekil tespiti) 5m'de BASARISIZ OLDU: hedef bu mesafede kamerada
        # sadece ~21px capinda gorunuyor (kamera_odak_px*18cm/500cm), bu
        # olcekte halkalar arasindaki ince beyaz bosluklar kameranin
        # cozunurlugunde kayboluyor - hem Hough hem kontur testinde hedef,
        # ayni boyuttaki DUZ/HALKASIZ bir "decoy" daireden AYIRT EDILEMEDI
        # (bkz. eski surumdeki test_hough_hedef*.py betikleri).
        #
        # COZUM: kenar/sekil aramak yerine SABLON ESLESTIRME (template
        # matching, cv2.matchTemplate + TM_CCOEFF_NORMED) kullanilir.
        # Beklenen boyutta (mesafeden hesaplanan, bkz.
        # _beklenen_hedef_yaricapi_px), hedefin GERCEK oranlarina (1:2:3,
        # koyu-acik-koyu radyal profil) uygun KUCUK bir sablon goruntu
        # ureteip TUM karede kaydirilarak normallesitirilmis capraz-korelasyon
        # ile en iyi eslesen konum aranir - bu, TEK bir edge/kenara degil
        # TUM radyal parlaklik profiline bakar, bu yuzden kucuk olcekte bile
        # duz bir decoy daireden istatistiksel olarak ayirt edilebiliyor
        # (test: gercek hedef skor~0.87-0.92, boyutu benzer duz decoy
        # skor~0.3-0.7 - bkz. sablon_esik). Skor esigin altindaysa (hicbir
        # aday yeterince eslesmiyorsa) GUVENLIK AGI olarak TAM kareye geri
        # dusulur, YOLO hicbir zaman tamamen korletilmez.
        # GUNCELLEME: sahada 10m'de test edildi, model bu mesafede zaten
        # egitildigi icin dogrudan tam karede iyi calisiyor ve ROI hicbir
        # zaman anlamli bir eslesme bulamadi (sablon skoru hep esigin
        # altinda kaldi) - varsayilan KAPATILDI, gereksiz CPU harciyordu.
        self.declare_parameter('opencv_on_tespit_aktif', False)
        # Her karede DEGIL, bu kadar karede BIR tam sablon taramasi yapilir;
        # aradaki karelerde son bulunan ROI (CPU'suz) tekrar kullanilir -
        # sahada Hough'un/tespitin her karede calismasi gozle gorulur
        # kasmaya sebep oluyordu.
        self.declare_parameter('roi_yeniden_tespit_kare_araligi', 4)
        # Hedefin GERCEK disi yaricapi (cm) - varsayilan 18cm (bkz. hedef
        # goruntusu: 6/12/18cm oranli ic ice halkalar).
        self.declare_parameter('hedef_gercek_yaricap_cm', 18.0)
        # cv2.matchTemplate (TM_CCOEFF_NORMED) skoru bu esigin ALTINDAYSA
        # aday reddedilir (GUVENLIK AGI: TAM kareye geri dusulur). Test
        # edildi: 10m'de (model bu mesafede egitildi) gercek hedef
        # ~0.996-0.998, boyutu hedefe cok yakin duz/halkasiz bir decoy en
        # fazla ~0.785'e cikiyor - 0.85, ikisi arasinda rahat bir pay
        # birakiyor. Cok fazla "bulunamadi" yasaniyorsa dusurulebilir,
        # yanlis kilitlenme oluyorsa yukseltilebilir.
        self.declare_parameter('sablon_esik', 0.85)
        # ROI, beklenen yaricapin bu kat kadar etrafini kapsar - hedef
        # kareler arasi biraz kaysa/mesafe tahmini biraz sapsa bile YOLO'nun
        # onu kaybetmemesi icin payli tutulur.
        self.declare_parameter('roi_padding_orani', 1.25)
        # Uzak/kucuk hedeflerde (ör. 10m'de ~5px yaricap) oransal pay bile
        # cok kucuk kalip ROI'yi birkac kare arasindaki kucuk turret
        # kaymalarina karsi savunmasiz birakiyordu (sahada: kilit/lazer
        # surekli acilip kapaniyordu) - ROI yaricapi HER ZAMAN en az bu
        # kadar (px) olur.
        self.declare_parameter('roi_min_pay_px', 45.0)

        self._camera_index = self.get_parameter('camera_index').value
        self._kamera_arama_ismi = self.get_parameter('kamera_arama_ismi').value
        self._serial_port = self.get_parameter('serial_port').value
        self._baudrate = self.get_parameter('baudrate').value
        self._model_path = self.get_parameter('model_path').value
        self._model_imgsz = self.get_parameter('model_imgsz').value
        self._conf = self.get_parameter('hedef_confidence').value
        self._kilit_kaybi_esigi = self.get_parameter('kilit_kaybi_esigi_px').value
        self._kilit_bekleme_s = self.get_parameter('kilit_bekleme_s').value
        self._kilit_sonrasi_bekleme_s = self.get_parameter('kilit_sonrasi_bekleme_s').value
        self._ince_ayar_tol = self.get_parameter('ince_ayar_toleransi_px').value
        self._ince_ayar_kayip_tol = self.get_parameter('ince_ayar_kayip_toleransi_px').value
        self._hassas_epsilon = self.get_parameter('hassas_epsilon_px').value
        self._hassas_adim_kazanci = self.get_parameter('hassas_adim_kazanci').value
        self._hassas_maks_adim_burst = self.get_parameter('hassas_maks_adim_burst').value
        self._nudge_araligi = self.get_parameter('nudge_araligi').value
        self._timeout = self.get_parameter('komut_timeout').value
        self._x_ters = self.get_parameter('x_yonu_ters_mi').value
        self._y_ters = self.get_parameter('y_yonu_ters_mi').value
        self._eksen_yer_degistir = self.get_parameter('motor_eksenlerini_yer_degistir').value
        self._video_target_ip = self.get_parameter('video_target_ip').value
        self._video_target_port = self.get_parameter('video_target_port').value
        self._opencv_on_tespit_aktif = self.get_parameter('opencv_on_tespit_aktif').value
        self._roi_yeniden_tespit_araligi = self.get_parameter('roi_yeniden_tespit_kare_araligi').value
        self._hedef_gercek_yaricap_cm = self.get_parameter('hedef_gercek_yaricap_cm').value
        self._sablon_esik = self.get_parameter('sablon_esik').value
        self._roi_padding_orani = self.get_parameter('roi_padding_orani').value
        self._roi_min_pay_px = self.get_parameter('roi_min_pay_px').value
        self._son_sablon_r = None
        self._son_sablon = None
        self._pid_kp = self.get_parameter('pid_kp').value
        self._pid_ki = self.get_parameter('pid_ki').value
        self._pid_kd = self.get_parameter('pid_kd').value
        self._pid_min_hiz = self.get_parameter('pid_min_hiz').value
        self._pid_max_hiz = self.get_parameter('pid_max_hiz').value
        self._pid_doygunluk = self.get_parameter('pid_hata_doygunlugu_px').value
        self._konum_alpha = self.get_parameter('konum_yumusatma_alpha').value
        self._kamera_lazer_dikey_mm = self.get_parameter('kamera_lazer_dikey_mm').value
        self._kamera_lazer_yatay_mm = self.get_parameter('kamera_lazer_yatay_mm').value
        self._kamera_odak_px = self.get_parameter('kamera_odak_px').value
        self._paralaks_sabit_mesafe = self.get_parameter('paralaks_sabit_mesafe_cm').value
        self._mesafe_timeout = self.get_parameter('mesafe_timeout_s').value
        self._nisan_bias_x = self.get_parameter('nisan_bias_x_px').value
        self._nisan_bias_y = self.get_parameter('nisan_bias_y_px').value
        self._lidar_gecersiz_esik = self.get_parameter('lidar_gecersiz_esik_cm').value
        self._lidar_medyan_pencere_n = self.get_parameter('lidar_medyan_pencere_n').value
        self._paralaks_maks_ofset = self.get_parameter('paralaks_maks_ofset_px').value
        self._mesafe_alpha = self.get_parameter('mesafe_yumusatma_alpha').value
        self._hedef_kayip_esigi = self.get_parameter('hedef_kayip_esigi_s').value

        # ros2 param set /turret_node <isim> <deger> ile yeniden baslatmadan
        # canli ayarlanabilmesi icin: declare_parameter+get_parameter().value
        # SADECE baslangicta bir kere okur, sonrasinda degeri hic
        # guncellemez. Bu yuzden asagidaki parametreler degistiginde ilgili
        # self._xxx degiskenini guncelleyen bir callback kaydediyoruz.
        self._param_attr_haritasi = {
            'hedef_confidence': '_conf',
            'ince_ayar_toleransi_px': '_ince_ayar_tol',
            'ince_ayar_kayip_toleransi_px': '_ince_ayar_kayip_tol',
            'hassas_epsilon_px': '_hassas_epsilon',
            'hassas_adim_kazanci': '_hassas_adim_kazanci',
            'hassas_maks_adim_burst': '_hassas_maks_adim_burst',
            'kilit_kaybi_esigi_px': '_kilit_kaybi_esigi',
            'kilit_bekleme_s': '_kilit_bekleme_s',
            'kilit_sonrasi_bekleme_s': '_kilit_sonrasi_bekleme_s',
            'nudge_araligi': '_nudge_araligi',
            'komut_timeout': '_timeout',
            'x_yonu_ters_mi': '_x_ters',
            'y_yonu_ters_mi': '_y_ters',
            'motor_eksenlerini_yer_degistir': '_eksen_yer_degistir',
            'pid_kp': '_pid_kp',
            'pid_ki': '_pid_ki',
            'pid_kd': '_pid_kd',
            'pid_min_hiz': '_pid_min_hiz',
            'pid_max_hiz': '_pid_max_hiz',
            'pid_hata_doygunlugu_px': '_pid_doygunluk',
            'konum_yumusatma_alpha': '_konum_alpha',
            'kamera_lazer_dikey_mm': '_kamera_lazer_dikey_mm',
            'kamera_lazer_yatay_mm': '_kamera_lazer_yatay_mm',
            'kamera_odak_px': '_kamera_odak_px',
            'paralaks_sabit_mesafe_cm': '_paralaks_sabit_mesafe',
            'mesafe_timeout_s': '_mesafe_timeout',
            'nisan_bias_x_px': '_nisan_bias_x',
            'nisan_bias_y_px': '_nisan_bias_y',
            'lidar_gecersiz_esik_cm': '_lidar_gecersiz_esik',
            'paralaks_maks_ofset_px': '_paralaks_maks_ofset',
            'mesafe_yumusatma_alpha': '_mesafe_alpha',
            'hedef_kayip_esigi_s': '_hedef_kayip_esigi',
        }
        self.add_on_set_parameters_callback(self._param_degisti_cb)

        # --- Durum: guvenli varsayilan MANUEL - /silah_modu mesaji gelene
        # kadar otonom hedefleme ASLA baslamaz. ---
        self._mod = 'MANUEL'
        self._manuel_x = 0.0
        self._manuel_y = 0.0
        self._manuel_zaman = None
        # Manuel (joystick) ates: /silah_ates_manuel - MANUEL modda lazer
        # HICBIR ZAMAN otomatik yanmadigi icin (bkz. dosya basi notu) elle
        # tetiklenmesi gerekiyor. Kendi heartbeat'i var (bkz. _manuel_heartbeat)
        # - sinyal kesilirse (arayuz coktu/baglanti gitti) lazer otomatik
        # kapanir, acik takili kalmaz.
        self._ates_manuel_aktif = False
        self._ates_manuel_zaman = None
        self._ATES_MANUEL_TIMEOUT_S = 0.5
        self._tam_kilit_log = False
        # MODEL AC/KAPA (2026-08-31, gorev sekansi): /turret_model_aktif ile
        # kontrol edilir. VARSAYILAN KAPALI - launch aninda kamera acilir
        # (asagida _ana_dongu, cap acma kismi) ama izleme/YOLO/lazer
        # calismaz; gorev sekansi (tabela_etap_yoneticisi.py) Stop
        # tabelasindan sonra bunu ACAR. tabela_node.py'deki ayni desenin
        # (model kapali -> kamera acik, sadece UDP goruntu) esidir.
        self._model_aktif = False
        # HEDEF VURULDU SAYACI (2026-08-31, kullanici istegi): "1 kere
        # kilitlenip 5sn beklemek DEGIL, 3 kere kilitlenip 5er sn
        # bekledikten sonra devam etsin". Her TAMAMLANAN kilit+5sn-bekleme
        # dongusunde (asagida _kilit_bitis_zamani ayarlanan noktada, bkz.
        # "kilit YENİ bitti" yorumu) sayac 1 artar. 3'e ulasinca
        # /silah_hedef_vuruldu yayinlanir VE sayac sifirlanir (bir sonraki
        # hedef/gorev icin taze baslasin diye).
        self._basarili_kilit_sayaci = 0
        self._HEDEF_VURULDU_ESIGI = 3
        self._kilitli_mi = False  # histerezis durumu (bkz. kilit_kaybi_esigi_px)
        self._pozisyon_kilitli = False  # _ana_dongu icinde hesaplanir
        self._hassas_mod = False  # hareket modu histerezisi (bkz. ince_ayar_kayip_toleransi_px)
        self._kilit_baslangic_zamani = None  # bkz. kilit_bekleme_s
        self._kilit_bitis_zamani = None  # bkz. kilit_sonrasi_bekleme_s
        self._hassas_son_yon = None  # bkz. _TERS_YON - "sicrama bastirma" icin

        # PID durumu (pan ve tilt icin ayri, sadece o an aktif eksen guncellenir)
        self._pan_integral = 0.0
        self._pan_onceki_hata = 0.0
        self._pan_onceki_zaman = None
        self._tilt_integral = 0.0
        self._tilt_onceki_hata = 0.0
        self._tilt_onceki_zaman = None

        # Hedef merkez konumu icin ustel yumusatma (EMA) durumu
        self._duz_ocx = None
        self._duz_ocy = None

        # Arduino'ya bagli TF02-Pro lidar'dan gelen "D:<cm>\n" raporlari -
        # taze/guvenilirse paralaks hesabinda GERCEKTEN kullanilir (bkz.
        # _paralaks_ofseti_hesapla), ayrica goruntude sag ust kosede gosterilir.
        self._son_mesafe_cm = None
        self._son_mesafe_zamani = None
        self._mesafe_ham_debug = None
        # Tekil aykiri ("spike") lidar okumalarina karsi: EMA'dan once
        # medyan penceresi (bkz. lidar_medyan_pencere_n).
        self._mesafe_ham_pencere = deque(maxlen=max(1, self._lidar_medyan_pencere_n))
        self._hedef_son_gorulme_zamani = None
        self._son_lidar_debug_log = 0.0

        # OPENCV ON-TESPIT durumu: bulunan ROI birkac kare boyunca YENIDEN
        # ARAMADAN tekrar kullanilir (bkz. _ana_dongu) - Hough her karede
        # calisirsa gozle gorulur kasmaya sebep oluyordu (sahada
        # gozlemlendi), tespit periyodik yapilip aradaki karelerde son
        # bilinen ROI korunarak CPU yuku dusuruldu.
        self._son_roi = None
        self._roi_atlama_sayaci = 0
        self._debug_toplam_satir_sayisi = 0
        self._debug_d_satir_sayisi = 0
        self._debug_son_ham_deger = None

        self.create_subscription(String, 'silah_modu', self._mod_cb, 10)
        self.create_subscription(Twist, 'turret_manuel_cmd', self._manuel_cmd_cb, 10)
        self.create_subscription(Bool, 'silah_ates_manuel', self._ates_manuel_cb, 10)
        self.create_subscription(Bool, '/turret_model_aktif', self._model_aktif_cb, 10)
        # Bool DEGIL Int32: gorev sekansi sadece "vuruldu oldu" anini degil,
        # o ana kadar KACINCI basarili kilit oldugunu da izleyebilsin diye
        # (ör. ilerleme gostergesi/log) - 3'e ulasinca "vuruldu" sayilir.
        self._hedef_vuruldu_pub = self.create_publisher(Int32, '/silah_hedef_vuruldu', 10)

        # MANUEL komut gonderimi kamera/YOLO dongusunden bagimsiz, sabit
        # hizli (20Hz) bir zamanlayici ile yapilir - boylece tus tepkisi
        # YOLO cikarim suresine bagli kalmaz. Firmware da heartbeat modeline
        # gectigi icin ayni yon tekrar tekrar gonderilmesi kesintisiz harekete
        # karsilik gelir.
        self.create_timer(0.05, self._manuel_heartbeat)

        self._sock_video = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self._arduino = None
        self._arduino_kuyruk = queue.Queue(maxsize=1)
        # Lazer, hareket komutlarindan tamamen ayri kendi kuyrugu/yazici
        # thread'i ve firmware'de kendi guvenlik zaman asimi ile calisir -
        # boylece hareket trafigiyle kuyrukta yer kapismaz.
        self._lazer_kuyruk = queue.Queue(maxsize=1)
        # Tek-adim (hassas surunme) komutlari da kendi ayri kuyrugunda -
        # firmware'de kendi kendine biten, heartbeat GEREKTIRMEYEN bir
        # komut oldugu icin hareket kuyruguyla karismasin. Bkz. _tek_adim_gonder.
        self._adim_kuyruk = queue.Queue(maxsize=1)
        self._baglanti_kur()

        self.get_logger().info(f'YOLO modeli yukleniyor: {self._model_path}')
        # task='detect' ZORLANMAZ: hem duz tespit (.pt) hem segmentasyon
        # (.engine/.pt) modelleri desteklenebilsin diye checkpoint'ten
        # OTOMATIK algilanir - segmentasyon sonuclarinda da result.boxes
        # ayni sekilde mevcuttur (maskeler ayrica gelir, biz kullanmiyoruz).
        self._model = YOLO(self._model_path)
        if self._model_path.endswith('.pt'):
            # .engine/.onnx gibi ONCEDEN DERLENMIS formatlar zaten hedef
            # cihaza gore derlenmistir - .to() sadece ham .pt icin gerekli/
            # anlamlidir.
            self._model.to('cuda')

        self._calisiyor = True
        threading.Thread(target=self._arduino_yazici_thread, daemon=True).start()
        threading.Thread(target=self._lazer_yazici_thread, daemon=True).start()
        threading.Thread(target=self._adim_yazici_thread, daemon=True).start()
        threading.Thread(target=self._arduino_okuyucu_thread, daemon=True).start()
        threading.Thread(target=self._ana_dongu, daemon=True).start()

        self.get_logger().info(
            'turret_node aktif (PARALAKS DUZELTME: lidar mesafesi (taze '
            'ise) + trigonometri + sahada olculmus bias trim ile nisan '
            'noktasi kaydirilir, tam kilitlenince lazer yanar): '
            'silah_modu / turret_manuel_cmd dinleniyor'
        )

    def _turret_arduino_portu_bul(self):
        """Takili USB-seri cihazlari tarar, Arduino MEGA'yi (silahin turret
        karti) VID:PID'e, bulamazsa tanimlayicida "arduino mega" gecen porta,
        o da yoksa sadece "arduino" gecen ilk porta gore bulur. Hicbiri yoksa
        None doner (bu durumda serial_port parametresine geri dusulur)."""
        yedek = None
        for p in serial.tools.list_ports.comports():
            if (p.vid, p.pid) in ARDUINO_MEGA_VID_PID:
                return p.device
            etiket = ' '.join(filter(None, [p.manufacturer, p.description, p.product])).lower()
            if 'arduino mega' in etiket:
                return p.device
            if yedek is None and 'arduino' in etiket:
                yedek = p.device
        return yedek

    def _baglanti_kur(self):
        # Sabit parametre yerine, USB'de Arduino MEGA'yi otomatik bul -
        # tekerlek motor Arduino'su (Uno) ya da Cube Orange farkli ACM
        # numaralarina dusse/kayса bile dogru portu bulur. Bulunamazsa
        # (ör. USB taramasi gecici basarisiz oldu) launch/param ile verilen
        # serial_port'a geri dusulur.
        port = self._turret_arduino_portu_bul() or self._serial_port
        try:
            self._arduino = serial.Serial(port, self._baudrate, timeout=0)
            time.sleep(2)
            self._serial_port = port
            self.get_logger().info(f'Turret Arduino (MEGA) baglandi: {port}')
        except Exception as e:
            self._arduino = None
            self.get_logger().error(f'Turret Arduino baglanti hatasi ({port}): {e}')

    def _param_degisti_cb(self, params):
        """ros2 param set /turret_node <isim> <deger> ile gelen degisiklikleri
        ilgili self._xxx onbellek degiskenine yansitir (bkz. __init__'teki
        _param_attr_haritasi)."""
        for p in params:
            attr = self._param_attr_haritasi.get(p.name)
            if attr is not None:
                setattr(self, attr, p.value)
                self.get_logger().info(f'Parametre guncellendi: {p.name} = {p.value}')
        return SetParametersResult(successful=True)

    def _mod_cb(self, msg: String):
        yeni = msg.data.strip().upper()
        if yeni in ('OTONOM', 'MANUEL'):
            if yeni != self._mod:
                self.get_logger().info(f'silah modu degisti -> {yeni}')
                if yeni != 'OTONOM':
                    # Guvenlik: OTONOM'dan cikildigi an lazeri hemen kapat,
                    # firmware'in kendi 250ms zaman asimini beklemeye gerek yok.
                    self._lazer_gonder('k')
                    self._pid_sifirla()
                    self._hedef_takibi_sifirla()
            self._mod = yeni

    def _manuel_cmd_cb(self, msg: Twist):
        self._manuel_x = msg.linear.x
        self._manuel_y = msg.linear.y
        self._manuel_zaman = time.time()

    def _ates_manuel_cb(self, msg: Bool):
        self._ates_manuel_aktif = bool(msg.data)
        self._ates_manuel_zaman = time.time()

    def _model_aktif_cb(self, msg: Bool):
        yeni = bool(msg.data)
        if yeni != self._model_aktif:
            self.get_logger().info(
                f'turret modeli {"AKTIF" if yeni else "DURDURULDU (kamera acik kalir)"}')
        if not yeni:
            # Model kapatilirken kilit/sayac durumu da temizlenir - bir
            # sonraki acilista taze baslasin (yarim kalmis bir kilit/sayim
            # tasinmasin).
            self._kilitli_mi = False
            self._kilit_baslangic_zamani = None
            self._kilit_bitis_zamani = None
            self._basarili_kilit_sayaci = 0
            self._lazer_gonder('k')
        self._model_aktif = yeni

    def _canli_mi(self, zaman, timeout=None):
        sinir = self._timeout if timeout is None else timeout
        return zaman is not None and (time.time() - zaman) < sinir

    def _manuel_heartbeat(self):
        """Kamera dongusunden bagimsiz calisir: MANUEL modda son komutu
        firmware'e heartbeat olarak sabit hizla (~20Hz) gonderir."""
        if self._mod != 'MANUEL':
            return
        if self._canli_mi(self._manuel_zaman):
            komut = self._yon_komutunu_hesapla(self._manuel_x, self._manuel_y)
        else:
            komut = 'x'
        self._komut_gonder(komut)
        # Manuel ates: sadece sinyal TAZE iken (arayuzdeki buton basili
        # tutulup surekli tekrarlanirken) lazer acik kalir - buton
        # birakilinca VEYA baglanti kesilince (0.5sn icinde yeni sinyal
        # gelmezse) otomatik kapanir, acik takili kalmaz.
        if self._ates_manuel_aktif and self._canli_mi(self._ates_manuel_zaman, timeout=self._ATES_MANUEL_TIMEOUT_S):
            self._lazer_gonder('l')
        else:
            self._lazer_gonder('k')

    def _arduino_yazici_thread(self):
        while self._calisiyor:
            try:
                karakter, hiz = self._arduino_kuyruk.get(timeout=0.5)
                if self._arduino and self._arduino.is_open:
                    # Hareket komutlari 2 bayt (yon+hiz) TEK write() cagrisiyla
                    # gonderilir ki firmware'in framing'i (yon sonrasi hiz
                    # bekleme) bozulmasin. 'x' tek bayttir, hiz gerekmez.
                    if karakter == 'x':
                        self._arduino.write(b'x')
                    else:
                        self._arduino.write(karakter.encode('utf-8') + bytes([max(0, min(255, hiz))]))
                self._arduino_kuyruk.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Seri port yazma hatasi: {e}')
                time.sleep(0.1)

    def _komut_gonder(self, karakter, hiz=255):
        """Firmware hareket komutunu ('w'/'a'/'s'/'d' + hiz 0-255, ya da
        tek basina 'x') kuyruga koyar."""
        if self._arduino_kuyruk.full():
            try:
                self._arduino_kuyruk.get_nowait()
            except queue.Empty:
                pass
        self._arduino_kuyruk.put((karakter, hiz))

    def _lazer_yazici_thread(self):
        while self._calisiyor:
            try:
                komut = self._lazer_kuyruk.get(timeout=0.5)
                if self._arduino and self._arduino.is_open:
                    self._arduino.write(komut.encode('utf-8'))
                self._lazer_kuyruk.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Lazer seri port yazma hatasi: {e}')
                time.sleep(0.1)

    def _lazer_gonder(self, karakter):
        """'l' (ac) / 'k' (kapat) komutunu lazerin kendi kuyruguna koyar."""
        if self._lazer_kuyruk.full():
            try:
                self._lazer_kuyruk.get_nowait()
            except queue.Empty:
                pass
        self._lazer_kuyruk.put(karakter)

    def _adim_yazici_thread(self):
        while self._calisiyor:
            try:
                yon, adim_sayisi = self._adim_kuyruk.get(timeout=0.5)
                if self._arduino and self._arduino.is_open:
                    self._arduino.write(
                        b'b' + yon.encode('utf-8') + bytes([max(1, min(255, adim_sayisi))])
                    )
                self._adim_kuyruk.task_done()
            except queue.Empty:
                continue
            except Exception as e:
                self.get_logger().error(f'Adim seri port yazma hatasi: {e}')
                time.sleep(0.1)

    def _coklu_adim_gonder(self, yon, adim_sayisi):
        """Firmware'e N step pulse'u ('b' + yon + sayi) komutu koyar - HASSAS
        modda hatayi kare basina EN FAZLA 1 adimla sinirlamak yerine (bu,
        "kilitlenmesi cok uzun suruyor" sikayetine sebep oluyordu - sahada
        gozlemlendi), hatayi DOGRUDAN gereken adim sayisina cevirip TEK
        seferde gonderir. Bkz. HASSAS SURUNME (_ana_dongu)."""
        if self._adim_kuyruk.full():
            try:
                self._adim_kuyruk.get_nowait()
            except queue.Empty:
                pass
        self._adim_kuyruk.put((yon, adim_sayisi))

    def _tek_adim_gonder(self, yon):
        """Geriye-donuk uyumluluk: TEK adim (bkz. _coklu_adim_gonder)."""
        self._coklu_adim_gonder(yon, 1)

    def _arduino_okuyucu_thread(self):
        """Arduino'nun USB seri hat uzerinden geri gonderdigi satirlari
        (su an sadece TF02-Pro lidar mesafe raporu: 'D:<cm>') okur."""
        buf = b''
        while self._calisiyor:
            try:
                if self._arduino and self._arduino.is_open:
                    bekleyen = self._arduino.in_waiting
                    if bekleyen:
                        buf += self._arduino.read(bekleyen)
                        while b'\n' in buf:
                            satir, buf = buf.split(b'\n', 1)
                            self._arduino_satir_isle(satir.decode('utf-8', errors='ignore').strip())
                time.sleep(0.02)
            except Exception as e:
                self.get_logger().error(f'Arduino okuma hatasi: {e}')
                time.sleep(0.1)

    def _arduino_satir_isle(self, satir):
        self._debug_toplam_satir_sayisi += 1
        if satir.startswith('D:'):
            self._debug_d_satir_sayisi += 1
            try:
                deger = int(satir[2:])
                self._debug_son_ham_deger = deger
                if deger >= self._lidar_gecersiz_esik:
                    self._mesafe_ham_debug = deger
                    # Tekil aykiri okumalara (yansima/multipath) karsi once
                    # medyan penceresinden gecir (medyan tek bir ucuk degerden
                    # neredeyse etkilenmez), SONRA ustel yumusatma (EMA)
                    # uygula - lidar'in dar huzmesi kucuk duzeltme
                    # hareketleri sirasinda bazen hedefi bazen arka plani
                    # olcebiliyor, EMA bu daha yumusak/gercek degisimi filtreler.
                    self._mesafe_ham_pencere.append(deger)
                    medyan_deger = statistics.median(self._mesafe_ham_pencere)
                    if self._son_mesafe_cm is None:
                        self._son_mesafe_cm = medyan_deger
                    else:
                        a = self._mesafe_alpha
                        self._son_mesafe_cm = a * medyan_deger + (1 - a) * self._son_mesafe_cm
                    self._son_mesafe_zamani = time.time()
            except ValueError:
                self._debug_son_ham_deger = f'PARSE_HATASI({satir!r})'

    def _paralaks_ofseti_hesapla(self):
        """Kamera-lazer paralaks duzeltmesinin kalbi: lazerin hedefe tam
        isabet etmesi icin goruntude nisan alinmasi gereken noktanin,
        kamera merkezinden (aim_center) ne kadar (dx, dy piksel) kaydirilmasi
        gerektigini hesaplar.

        Fizik: kamera ve lazer ayni eksende, sadece dikeyde
        kamera_lazer_dikey_mm kadar oteli (acisal sapma yok). Lazer, kamera
        ile PARALEL (ayni yon) atesleniyor, bu yuzden hedef duzleminde
        (mesafe_mm uzaklikta) kamera'nin baktigi noktadan kamera_lazer_*_mm
        kadar kayan bir noktaya vurur. Bu gercek-dunya kaymasi, kamera'dan
        bakildiginda aci = atan2(offset_mm, mesafe_mm) kadarlik bir acisal
        farka, o da goruntude piksel_kaymasi = tan(aci) * kamera_odak_px
        kadar bir piksel farkina karsilik gelir. kamera_odak_px, OpenCV
        checkerboard kalibrasyonuyla olculmus gercek odak uzakligidir.

        Mesafe olarak lidar TAZEYSE (mesafe_timeout_s icinde, medyan+EMA
        filtrelenmis) olculen deger kullanilir, degilse paralaks_sabit_mesafe_cm'e
        dusulur. nisan_bias_x/y_px, mm-formulunun modellemedigi (mekanik
        acisal sapma vb.) sabit artik hatayi sahada olculup eklenen bir
        trim'dir - mesafeden BAGIMSIZ sabit bir piksel degeridir."""
        lidar_taze_mi = self._son_mesafe_cm and self._canli_mi(self._son_mesafe_zamani, self._mesafe_timeout)
        mesafe_cm = self._son_mesafe_cm if lidar_taze_mi else self._paralaks_sabit_mesafe
        mesafe_mm = mesafe_cm * 10.0

        aci_pan = math.atan2(self._kamera_lazer_yatay_mm, mesafe_mm)
        aci_tilt = math.atan2(self._kamera_lazer_dikey_mm, mesafe_mm)
        dx = math.tan(aci_pan) * self._kamera_odak_px + self._nisan_bias_x
        dy = math.tan(aci_tilt) * self._kamera_odak_px + self._nisan_bias_y

        # GUVENLIK: yanlislikla cok kucuk bir paralaks_sabit_mesafe_cm
        # girilse bile (tan() sonsuza yaklasir) kayma bu sinirin disina
        # cikamaz.
        maks = self._paralaks_maks_ofset
        dx = max(min(dx, maks), -maks)
        dy = max(min(dy, maks), -maks)
        return dx, dy

    def _pid_sifirla(self):
        """Sadece PID integral/turev durumunu sifirlar. komut=='x' oldugunda
        (hedef merkezde VE duruyor OLSUN, ya da kayip OLSUN, ikisinde de)
        guvenle her zaman cagrilabilir - konum/mesafe yumusatmasina
        DOKUNMAZ (bkz. _hedef_takibi_sifirla)."""
        self._pan_integral = 0.0
        self._pan_onceki_hata = 0.0
        self._pan_onceki_zaman = None
        self._tilt_integral = 0.0
        self._tilt_onceki_hata = 0.0
        self._tilt_onceki_zaman = None

    def _hedef_takibi_sifirla(self):
        """Konum (EMA) ve mesafe (EMA) yumusatma durumunu sifirlar. SADECE
        hedef GERCEKTEN kayboldugunda ya da OTONOM moddan cikildiginda
        cagrilmali - hedef merkezde durup 'x' gonderilirken DEGIL, aksi
        halde mesafe/konum filtresi her kilitlenmede gereksiz sifirlanir
        (bu tam olarak yasadigimiz "mesafe_yumusatilmis_cm=None" bug'uydu)."""
        self._duz_ocx = None
        self._duz_ocy = None
        self._son_mesafe_cm = None
        self._mesafe_ham_pencere.clear()
        # ONEMLI: _kilit_baslangic_zamani BURADA SIFIRLANMAZ - onun
        # yasam dongusu TAMAMEN kilit-durum blogunun (_ana_dongu icinde,
        # kilit_bekleme_aktif hesabi) sorumlulugunda. Burada sifirlamak,
        # KOSULSUZ 5sn bekleme SOZUNU (kullanicinin acik talebi: "lazer
        # acikken kesinlikle oynamayacak") tam da bu fonksiyonun cagrildigi
        # senaryoda (hedef_kayip_esigi_s asilinca) BOZUYORDU - sahada
        # dogrulandi: bekleme her zaman TAM hedef_kayip_esigi_s kadar
        # surup erken bitiyordu, 5sn degil.
        self._hassas_son_yon = None

    def _pid_hiz_hesapla(self, hata, eksen):
        """Isaretli piksel hatasindan (PID ile) 0-255 araliginda hiz baytı
        uretir. eksen: 'pan' ya da 'tilt' - her eksenin kendi integral/turev
        durumu ayri tutulur."""
        su_an = time.time()
        if eksen == 'pan':
            onceki_hata, onceki_zaman, integral = self._pan_onceki_hata, self._pan_onceki_zaman, self._pan_integral
        else:
            onceki_hata, onceki_zaman, integral = self._tilt_onceki_hata, self._tilt_onceki_zaman, self._tilt_integral

        dt = min(su_an - onceki_zaman, 0.2) if onceki_zaman is not None else 0.0

        integral += hata * dt
        integral = max(min(integral, 500.0), -500.0)  # windup sinirlamasi
        turev = (hata - onceki_hata) / dt if dt > 0 else 0.0

        cikti = (self._pid_kp * hata) + (self._pid_ki * integral) + (self._pid_kd * turev)

        if eksen == 'pan':
            self._pan_integral, self._pan_onceki_hata, self._pan_onceki_zaman = integral, hata, su_an
        else:
            self._tilt_integral, self._tilt_onceki_hata, self._tilt_onceki_zaman = integral, hata, su_an

        oran = min(abs(cikti) / self._pid_doygunluk, 1.0) if self._pid_doygunluk > 0 else 1.0
        hiz = int(round(self._pid_min_hiz + oran * (self._pid_max_hiz - self._pid_min_hiz)))
        return max(min(hiz, self._pid_max_hiz), self._pid_min_hiz)

    def _yon_komutunu_hesapla(self, gx, gy):
        """Isaretli gx (pan) / gy (tilt) hatasindan tek eksenli firmware
        komutunu ('w'/'a'/'s'/'d'/'x') secer - buyuk mutlak hataya sahip
        eksen once duzeltilir (firmware ayni anda tek eksen kaldirir)."""
        if gx == 0 and gy == 0:
            return 'x'
        if abs(gx) >= abs(gy):
            return 'd' if gx > 0 else 'a'
        return 's' if gy > 0 else 'w'

    def _kamera_indexini_otomatik_bul(self, arama_metni):
        """Isminde arama_metni (kucuk/buyuk harf duyarsiz, orn. "C922")
        GECEN ilk /dev/videoN cihazinin index'ini bulur - birden fazla
        kamera bagliyken (orn. C270'ler de takiliyken) /dev/videoN
        numaralari ongorulemez sekilde degisebiliyor, sabit camera_index
        yanlis kameraya baglanmaya sebep olabiliyordu. UVC kameralar
        genelde ayni fiziksel cihaz icin birden fazla /dev/videoN
        DUGUMU (video+metadata) sunar - GERCEK goruntu yakalama dugumu
        bunlarin En KUCUK index'lisi olma egilimindedir (sahada
        dogrulandi: C270 icin video0=yakalama, video1=metadata), bu
        yuzden eslesen index'lerin EN KUCUGU secilir. Eslesme yoksa
        None doner (cagiran taraf camera_index parametresine geri duser)."""
        import glob
        eslesen_indexler = []
        for isim_dosyasi in sorted(glob.glob('/sys/class/video4linux/video*/name')):
            try:
                with open(isim_dosyasi) as f:
                    isim = f.read().strip()
            except OSError:
                continue
            if arama_metni.lower() in isim.lower():
                try:
                    index = int(isim_dosyasi.split('/video4linux/video')[1].split('/name')[0])
                except (IndexError, ValueError):
                    continue
                eslesen_indexler.append(index)
        return min(eslesen_indexler) if eslesen_indexler else None

    def _beklenen_hedef_yaricapi_px(self):
        """Bilinen hedef yaricapi (hedef_gercek_yaricap_cm) + kalibre kamera
        odagi (kamera_odak_px) + GUNCEL mesafe (lidar taze ise canli
        olculen, degilse paralaks_sabit_mesafe_cm - _paralaks_ofseti_hesapla
        ile AYNI kaynak) ile goruntude hedefin KAC piksel yaricapinda
        gorunmesi GEREKTIGINI hesaplar. Sabit genis bir Hough arama araligi
        yerine bu dar/dogru araligi kullanmak, hem yanlis-boyutlu adaylari
        elemede hem de ROI'yi gercek hedef boyutuna yakin (kucuk, oynak
        olmayan) tutmada cok daha etkili oldu (sahada 5m'de dogrulandi)."""
        lidar_taze_mi = self._son_mesafe_cm and self._canli_mi(self._son_mesafe_zamani, self._mesafe_timeout)
        mesafe_cm = self._son_mesafe_cm if lidar_taze_mi else self._paralaks_sabit_mesafe
        mesafe_cm = max(mesafe_cm, 1.0)  # sifira bolme koruma
        return self._kamera_odak_px * self._hedef_gercek_yaricap_cm / mesafe_cm

    def _hedef_sablonu_al(self, beklenen_r):
        """Hedefin GERCEK oranlarina (1:2:3, koyu-acik-koyu radyal profil)
        uygun, beklenen piksel yaricapinda sentetik bir sablon goruntu
        uretir/onbellekler (beklenen_r ~1px'ten fazla degismedikce yeniden
        uretmez - her karede ayni sablonu cizmek gereksiz)."""
        if self._son_sablon is not None and abs(self._son_sablon_r - beklenen_r) < 1.0:
            return self._son_sablon
        r = max(3.0, beklenen_r)
        boyut = int(r * 2 * 1.3) | 1  # tek sayi genislik/yukseklik
        merkez = boyut // 2
        arka_plan = 180
        sablon = np.full((boyut, boyut), arka_plan, dtype=np.uint8)
        r1 = max(1, int(r * 1 / 3))   # 6cm - ic siyah
        r2 = max(1, int(r * 2 / 3))   # 12cm - beyaz bosluk
        cv2.circle(sablon, (merkez, merkez), int(r), 10, -1)          # 18cm - dis siyah
        cv2.circle(sablon, (merkez, merkez), r2, arka_plan, -1)
        cv2.circle(sablon, (merkez, merkez), r1, 10, -1)
        sablon = cv2.GaussianBlur(sablon, (3, 3), 0.6)  # kamera/lens bulanikligi benzetimi
        self._son_sablon = sablon
        self._son_sablon_r = beklenen_r
        return sablon

    def _hedef_roi_bul(self, frame):
        """Hedefin (ic ice siyah halkalar) YAKLASIK konumunu, KENAR/SEKIL
        arayan Hough/kontur yontemleri YERINE SABLON ESLESTIRME (template
        matching) ile bulur - bkz. opencv_on_tespit_aktif parametresinin
        yorumundaki gerekce (5m'de kenar tabanli yontemler hedefi ayni
        boyutlu duz bir nesneden ayirt edemiyordu, sablon eslestirme TUM
        radyal parlaklik profiline baktigi icin cok daha ayirt edici oldu).
        YOLO'ya verilecek kirpilmis bolgeyi (x0,y0,x1,y1) olarak dondurur.
        Yeterince iyi bir eslesme yoksa None doner (cagiran taraf TAM
        kareye geri duser)."""
        beklenen_r = self._beklenen_hedef_yaricapi_px()
        sablon = self._hedef_sablonu_al(beklenen_r)
        sh, sw = sablon.shape
        if frame.shape[0] < sh or frame.shape[1] < sw:
            return None

        gri = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        sonuc = cv2.matchTemplate(gri, sablon, cv2.TM_CCOEFF_NORMED)
        _, maks_skor, _, maks_konum = cv2.minMaxLoc(sonuc)
        if maks_skor < self._sablon_esik:
            return None

        cx = maks_konum[0] + sw / 2.0
        cy = maks_konum[1] + sh / 2.0

        yukseklik, genislik = frame.shape[:2]
        # SAHADA BULUNDU (10m): beklenen_r cok kucukken (~5px) pay da
        # oranla kuculuyor (~6-7px) - ROI ~13px capinda kaliyor, turret
        # birkac kare arasi kucuk bir miktar kaysa bile (roi_yeniden_
        # tespit_kare_araligi karede bir tazeleniyor, aradaki karelerde ROI
        # SABIT) hedef ROI DISINA tasip tespit dusuyor, bu da kilit/lazer
        # dongusunun (kilit_bekleme_s'yi ATLAYARAK - hedef_surdurulebilir
        # False oluyor) surekli acilip kapanmasina sebep oluyordu (sahada
        # gozlemlendi: "TAM KILITLENDI" -> <1s icinde "kilit kayboldu").
        # Minimum bir mutlak piksel payi ile bu taban durum onleniyor.
        pay = max(beklenen_r * self._roi_padding_orani, self._roi_min_pay_px)
        x0 = max(0, int(cx - pay))
        y0 = max(0, int(cy - pay))
        x1 = min(genislik, int(cx + pay))
        y1 = min(yukseklik, int(cy + pay))
        if x1 - x0 < 10 or y1 - y0 < 10:
            return None
        return (x0, y0, x1, y1)

    def _ana_dongu(self):
        kamera_index = self._camera_index
        if self._kamera_arama_ismi:
            otomatik_index = self._kamera_indexini_otomatik_bul(self._kamera_arama_ismi)
            if otomatik_index is not None:
                self.get_logger().info(
                    f'Kamera otomatik bulundu: "{self._kamera_arama_ismi}" -> /dev/video{otomatik_index} '
                    f'(camera_index parametresi={self._camera_index} yok sayildi)'
                )
                kamera_index = otomatik_index
            else:
                self.get_logger().warn(
                    f'"{self._kamera_arama_ismi}" isminde kamera bulunamadi, '
                    f'camera_index parametresine ({self._camera_index}) dusuluyor'
                )
        cap = cv2.VideoCapture(kamera_index)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        # Odak sabit (sonsuz) tutulur: kamera_odak_px kalibrasyonu bu
        # odak ayariyla OLCULDU (tools/camera_calibrate.py) - otomatik
        # odaklama acik kalirsa gercek odak uzakligi kayar ve paralaks
        # duzeltmesi tutarsizlasir.
        cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        cap.set(cv2.CAP_PROP_FOCUS, 0)

        if not cap.isOpened():
            self.get_logger().error(f'Turret kamerasi acilamadi (index={kamera_index})')

        aim_center_x, aim_center_y = 320, 240
        son_komut = 'x'
        son_nudge_zamani = 0.0

        try:
            while self._calisiyor and rclpy.ok():
                ret, frame = cap.read()
                if not ret:
                    time.sleep(0.1)
                    continue

                if not self._model_aktif:
                    # Model KAPALI: kamera ACIK/canli kalsin diye ham kareyi
                    # UDP'ye gonder, YOLO/izleme/lazer HIC calistirilmaz
                    # (bkz. tabela_node.py'deki ayni desen).
                    ret_enc, buffer = cv2.imencode(
                        '.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 60])
                    if ret_enc:
                        data = buffer.tobytes()
                        if len(data) < 65000:
                            try:
                                self._sock_video.sendto(
                                    data, (self._video_target_ip, self._video_target_port))
                            except Exception:
                                pass
                    time.sleep(0.05)
                    continue

                try:
                    su_an = time.time()

                    # OPENCV ON-TESPIT: butun isi YOLO'ya birakmak yerine,
                    # once sablon eslestirmeyle (bkz. _hedef_roi_bul, hedef
                    # boyutu mesafeden hesaplanir) hedefin yaklasik bolgesi
                    # bulunur, YOLO SADECE o kirpilmis bolgede (ROI)
                    # calistirilir. Yeterince iyi eslesme bulunamazsa (ör.
                    # kotu isik/hedef henuz goruntude degil) GUVENLIK AGI
                    # olarak TAM kareye geri dusulur, YOLO hicbir zaman
                    # tamamen korletilmez.
                    roi_ofset_x, roi_ofset_y = 0, 0
                    gonderilen_kare = frame
                    if self._opencv_on_tespit_aktif:
                        # Her karede DEGIL, roi_yeniden_tespit_kare_araligi
                        # karede bir tam tarama yapilir - aradaki karelerde
                        # son bulunan ROI CPU harcamadan tekrar kullanilir
                        # (sahada tespitin her karede calismasi
                        # gozle gorulur kasmaya sebep oluyordu). Hedef henuz
                        # HIC bulunamadiysa (son_roi=None) her karede aranmaya
                        # devam edilir - aksi halde hedef goruntuye girdiginde
                        # gecikmeli fark edilir.
                        if self._son_roi is not None and self._roi_atlama_sayaci > 0:
                            roi = self._son_roi
                            self._roi_atlama_sayaci -= 1
                        else:
                            roi = self._hedef_roi_bul(frame)
                            self._son_roi = roi
                            self._roi_atlama_sayaci = self._roi_yeniden_tespit_araligi if roi is not None else 0
                        if roi is not None:
                            rx0, ry0, rx1, ry1 = roi
                            gonderilen_kare = frame[ry0:ry1, rx0:rx1]
                            roi_ofset_x, roi_ofset_y = rx0, ry0
                            cv2.rectangle(frame, (rx0, ry0), (rx1, ry1), (255, 128, 0), 1)
                            cv2.putText(frame, 'ROI', (rx0 + 4, ry0 + 14),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 128, 0), 1)

                    # ByteTrack: duz tespit (model()) YERINE model.track()
                    # kullanilir - dahili Kalman filtresi ile, silah HAREKET
                    # HALINDEYKEN (motion blur) hedef bir-iki karede
                    # tespit edilemese bile, TAHMIN EDILEN konumla takibi
                    # KOPRULER (sahada gozlemlendi: hareket sirasinda hedef
                    # "sürekli bozuluyor" - duz tespitte bu, o karede
                    # hedef_bulundu=False'a dusup takibi tamamen kesiyordu).
                    # persist=True SART: ayni video akisinin ardisik
                    # kareleri olarak islenmesi icin (Jetson kamera dongusu
                    # her karede ayri ayri track() cagiriyor).
                    results = self._model.track(
                        gonderilen_kare, imgsz=self._model_imgsz, device=0, conf=self._conf,
                        persist=True, tracker='bytetrack.yaml', verbose=False
                    )

                    hedef_bulundu = False
                    gx, gy = 0, 0

                    for result in results:
                        if len(result.boxes) > 0:
                            hedef_bulundu = True
                            self._hedef_son_gorulme_zamani = su_an
                            # ROI'ye gore donen koordinatlar - cizim ve
                            # nisan hesabinin TAM karede calismasi icin
                            # ROI'nin sol-ust kosesi geri eklenir.
                            box = result.boxes[0].xyxy[0].cpu().numpy()
                            box[0] += roi_ofset_x
                            box[2] += roi_ofset_x
                            box[1] += roi_ofset_y
                            box[3] += roi_ofset_y
                            # Hedefin konumu: model SEGMENTASYON (poligon/maske)
                            # ile egitildigi icin, kutunun geometrik merkezi
                            # YERINE maskenin GERCEK agirlik merkezi (centroid)
                            # kullanilir - hedef egik/duzensiz sekilliyse ya
                            # da kismen kutunun bir kosesine yakinsa bu, kutu
                            # merkezinden daha dogru bir "hedefin GERCEKTEN
                            # oldugu nokta" verir. Maske yoksa (orn. duz
                            # tespit modeli kullanilirsa) kutu merkezine
                            # GERI DUSULUR.
                            tcx, tcy = None, None
                            if result.masks is not None and len(result.masks.xy) > 0:
                                poligon = result.masks.xy[0].astype(np.int32)
                                poligon[:, 0] += roi_ofset_x
                                poligon[:, 1] += roi_ofset_y
                                M = cv2.moments(poligon)
                                if M['m00'] != 0:
                                    tcx, tcy = int(M['m10'] / M['m00']), int(M['m01'] / M['m00'])
                            if tcx is None:
                                tcx, tcy = int((box[0] + box[2]) / 2), int((box[1] + box[3]) / 2)

                            cv2.rectangle(frame, (int(box[0]), int(box[1])), (int(box[2]), int(box[3])), (0, 255, 0), 2)

                            # Hedefin ham konumu: YOLO tespiti (maske centroid'i).
                            cv2.circle(frame, (tcx, tcy), 4, (0, 255, 0), -1)
                            cv2.putText(frame, 'YOLO', (tcx + 8, tcy - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)

                            # YOLO'nun kare-kare kucuk titreyen ham konumunu
                            # PID'e vermeden once ustel yumusatma (EMA) ile
                            # filtrele - aksi halde PID bu gurultuyu kovalar.
                            if self._duz_ocx is None:
                                self._duz_ocx, self._duz_ocy = float(tcx), float(tcy)
                            else:
                                a = self._konum_alpha
                                self._duz_ocx = a * tcx + (1 - a) * self._duz_ocx
                                self._duz_ocy = a * tcy + (1 - a) * self._duz_ocy
                            tcx_f, tcy_f = self._duz_ocx, self._duz_ocy

                            if self._mod == 'OTONOM':
                                # PARALAKS DUZELTME: lazerin hedefe tam isabet
                                # etmesi icin nisan alinmasi gereken nokta,
                                # (taze lidar mesafesi varsa onunla, yoksa
                                # sabit varsayimla) trigonometriyle + sahada
                                # olculmus bias trim'iyle hesaplanir (bkz.
                                # _paralaks_ofseti_hesapla).
                                dx, dy = self._paralaks_ofseti_hesapla()
                                referans_x, referans_y = aim_center_x + dx, aim_center_y + dy
                                cv2.drawMarker(frame, (int(referans_x), int(referans_y)), (0, 0, 255), cv2.MARKER_CROSS, 20, 2)
                                cv2.putText(frame, 'NISAN NOKTASI', (int(referans_x) + 8, int(referans_y) + 14),
                                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 0, 255), 1)

                                if su_an - self._son_lidar_debug_log >= 1.0:
                                    self.get_logger().info(
                                        f'[PARALAKS DEBUG] tcx={tcx_f:.0f} tcy={tcy_f:.0f} '
                                        f'referans=({referans_x:.0f},{referans_y:.0f}) dx={dx:.1f} dy={dy:.1f} '
                                        f'lidar_mesafe_cm={("%.0f" % self._son_mesafe_cm) if self._son_mesafe_cm else None}'
                                    )
                                    self._son_lidar_debug_log = su_an

                                # Hata: hedefi nisan noktasina tasimak
                                # istiyoruz, hedefi goruntu merkezine degil.
                                ex, ey = tcx_f - referans_x, tcy_f - referans_y
                                # HASSAS MOD karari (KABA<->HASSAS gecisi):
                                # hata kucukse (ince_ayar_* HISTEREZISLI esik
                                # ciftiyle) HASSAS TEK-ADIM moduna gecilir.
                                # Histerezis SART: tek sabit esikte hata tam
                                # sinirda salinip surekli mod gecisine sebep olur.
                                hassas_esik = self._ince_ayar_kayip_tol if self._hassas_mod else self._ince_ayar_tol
                                self._hassas_mod = abs(ex) <= hassas_esik and abs(ey) <= hassas_esik
                                # LAZER/KILIT karari: SADECE hassas moddayken
                                # VE hata gercekten cok kucukken (artik hic
                                # duzeltmeye gerek yok) kilitlenir - kilide
                                # GIRMEK icin hassas_epsilon_px (siki), kilidi
                                # KAYBETMEK icin daha genis kilit_kaybi_esigi_px
                                # (histerezis, flicker onleme).
                                kilit_esigi = self._kilit_kaybi_esigi if self._kilitli_mi else self._hassas_epsilon
                                self._pozisyon_kilitli = self._hassas_mod and abs(ex) <= kilit_esigi and abs(ey) <= kilit_esigi
                                gx, gy = ex, ey
                        break

                    if self._mod == 'OTONOM':
                        if self._x_ters:
                            gx = -gx
                        if self._y_ters:
                            gy = -gy
                        if self._eksen_yer_degistir:
                            gx, gy = gy, gx

                        if not hedef_bulundu:
                            gx, gy = 0, 0
                            self._hassas_mod = False
                            # Tek karelik kacirmaya tolerans: hedef sadece
                            # SURDURULEBILIR sekilde (esik suresinden uzun)
                            # gorulmediyse konum/mesafe yumusatmasini sifirla.
                            if not self._canli_mi(self._hedef_son_gorulme_zamani, self._hedef_kayip_esigi):
                                self._hedef_takibi_sifirla()

                        # YOLO tek bir karede hedefi kacirabilir (hedef hala
                        # oradayken bile) - bu TEK KARELIK kacirma yuzunden
                        # kilidi/lazeri hemen dusurmuyoruz (aksi halde her
                        # kacirmada lazer bir kare sonup bir kare yanar, hizli
                        # flicker olur). hedef_surdurulebilir: hedef bu
                        # kareden GORULMESE bile hedef_kayip_esigi_s icinde
                        # gorulduyse True - ayni tolerans, konum/mesafe
                        # yumusatmasi sifirlamada kullanilanla tutarli.
                        hedef_surdurulebilir = self._canli_mi(self._hedef_son_gorulme_zamani, self._hedef_kayip_esigi)
                        temel_kilit = hedef_surdurulebilir and self._pozisyon_kilitli

                        if temel_kilit and self._kilit_baslangic_zamani is None:
                            # Yeni bir kilit BASLADI - bekleme sayacini baslat.
                            self._kilit_baslangic_zamani = su_an

                        # KILIT BEKLEME (kilit_bekleme_s): hedef vurulup
                        # kilit saglandiktan sonra, bu sure KOSULSUZ dolar -
                        # hedefin bu sure icinde GECICI olarak (ör. lazer
                        # noktasinin kameraya parlayip YOLO'yu bir sureligine
                        # sasirtmasi - sahada gozlemlendi: kilit/kayip
                        # dongusu HER SEFERINDE tam hedef_kayip_esigi_s kadar
                        # surdugu icin bu supheleniliyor) TAMAMEN
                        # kaybolmasi bile bekleme'yi ERKEN IPTAL ETMEZ -
                        # kullanicinin acik talebi: "lazer acikken KESINLIKLE
                        # oynamayacak ve 5sn bekleyecek". ESKIDEN
                        # hedef_surdurulebilir=False oldugunda sayac hemen
                        # sifirlaniyordu, bu YUZUNDEN bekleme neredeyse hic
                        # calismiyordu (laser-kaynakli tespit kaybi TAM
                        # kayip sayiliyordu).
                        kilit_bekleme_aktif = (
                            self._kilit_baslangic_zamani is not None
                            and (su_an - self._kilit_baslangic_zamani) < self._kilit_bekleme_s
                        )
                        onceki_kilitli_mi = self._kilitli_mi
                        self._kilitli_mi = temel_kilit or kilit_bekleme_aktif
                        if not kilit_bekleme_aktif and not temel_kilit:
                            # Bekleme suresi bitti VE artik gercekten kilitli
                            # degil - sayaci sifirla ki bir sonraki GERCEK
                            # kilitte taze baslasin.
                            self._kilit_baslangic_zamani = None
                        if onceki_kilitli_mi and not self._kilitli_mi:
                            # Kilit YENI bitti (5sn bekleme doldu) - sahada
                            # gozlemlendi: kilit bitince o anki (henuz
                            # durulmamis/gecici) hataya gore hemen bir
                            # duzeltme atip kucuk bir "sicrama" yapabiliyordu.
                            # kilit_sonrasi_bekleme_s kadar HAREKETI (lazeri
                            # DEGIL - o zaten kilit bitince hemen soner) ayrica
                            # bastirarak olcumun/hedefin durulmasi icin pay
                            # birakiyoruz.
                            self._kilit_bitis_zamani = su_an

                            # HEDEF VURULDU SAYACI (bkz. __init__ yorumu):
                            # tam burasi TAM OLARAK "bir kilit+5sn-bekleme
                            # dongusu TAMAMLANDI" ani - kullanicinin istegi
                            # "1 kere degil 3 kere kilitlenip 5er sn
                            # bekledikten sonra devam etsin" burada sayilir.
                            self._basarili_kilit_sayaci += 1
                            self.get_logger().info(
                                f'🎯 Basarili kilit+bekleme dongusu: '
                                f'{self._basarili_kilit_sayaci}/{self._HEDEF_VURULDU_ESIGI}')
                            self._hedef_vuruldu_pub.publish(
                                Int32(data=self._basarili_kilit_sayaci))
                            if self._basarili_kilit_sayaci >= self._HEDEF_VURULDU_ESIGI:
                                self.get_logger().warn(
                                    f'✅ HEDEF VURULDU ({self._HEDEF_VURULDU_ESIGI} basarili '
                                    'kilit+bekleme dongusu tamamlandi).')
                                self._basarili_kilit_sayaci = 0

                        hareket_bastirilsin = self._kilitli_mi or (
                            self._kilit_bitis_zamani is not None
                            and (su_an - self._kilit_bitis_zamani) < self._kilit_sonrasi_bekleme_s
                        )

                        if hareket_bastirilsin:
                            # KILITLI (lazer ACIK) YA DA kilit YENI bitmis
                            # (kilit_sonrasi_bekleme_s icinde) - turret
                            # HICBIR hareket komutu (ne kaba ne hassas)
                            # GONDERMEZ - tam durgunluk. Sahada gozlemlendi:
                            # lazer acikken hassas modun kucuk mikro-
                            # adimlara devam etmesi istenmiyor, lazer SADECE
                            # turret gercekten durmusken yanmali; kilit
                            # BITTIGINDE de hemen eski (durulmamis) hataya
                            # gore tepki vermek kucuk bir "sicrama" yapiyordu.
                            if son_komut != 'x':
                                self._komut_gonder('x')
                                son_komut = 'x'
                            self._pid_sifirla()
                            # Kontrol yeniden basladiginda sicrama-bastirma
                            # durumu kilit-oncesi ESKI bir yonden degil,
                            # TEMIZ baslasin.
                            self._hassas_son_yon = None
                        elif self._hassas_mod and hedef_bulundu:
                            # HASSAS SURUNME: surekli heartbeat YERINE Arduino'ya
                            # hatayla ORANTILI SAYIDA step-pulse'u TEK seferde
                            # gonderilir (bkz. _coklu_adim_gonder, firmware'de
                            # cokluAdimAt). ESKIDEN kare basina EN FAZLA 1
                            # fiziksel adimla siniriydi - bu, 12800 pulses/rev
                            # mikro-adimla (cok kucuk fiziksel adim) buyukce
                            # bir hatanin duzelmesini onlarca kareye yayip
                            # "kilitlenmesi cok uzun suruyor" sikayetine sebep
                            # oluyordu (sahada gozlemlendi). adim_sayisi,
                            # hassas_adim_kazanci (px basina adim) ile hesaplanir,
                            # asiri gitmeyi onlemek icin hassas_maks_adim_burst
                            # ile sinirlanir.
                            if abs(gx) > self._hassas_epsilon or abs(gy) > self._hassas_epsilon:
                                adim_yonu = self._yon_komutunu_hesapla(gx, gy)
                                if adim_yonu == _TERS_YON.get(self._hassas_son_yon):
                                    # SICRAMA BASTIRMA: bu adim bir onceki
                                    # adimin tam tersi - bir onceki adim
                                    # hedefi hafifce gecmis demektir, bu
                                    # adimi ALMAK sadece ters yonde YENI bir
                                    # gecis baslatir (sonsuz zip-zap). Al-
                                    # miyoruz; bir sonraki GERCEK (ayni
                                    # yonde tekrar gereken) duzeltmede
                                    # sicim sifirdan baslasin diye durumu
                                    # temizliyoruz.
                                    self._hassas_son_yon = None
                                else:
                                    hata_px = max(abs(gx), abs(gy))
                                    adim_sayisi = max(1, min(
                                        self._hassas_maks_adim_burst,
                                        round(hata_px * self._hassas_adim_kazanci)
                                    ))
                                    self._coklu_adim_gonder(adim_yonu, adim_sayisi)
                                    self._hassas_son_yon = adim_yonu
                            else:
                                self._hassas_son_yon = None
                            # Kaba moddan gecerken kalan surekli hareketi
                            # hemen durdur (firmware'in kendi 250ms zaman
                            # asimini beklemeye gerek yok).
                            if son_komut != 'x':
                                self._komut_gonder('x')
                                son_komut = 'x'
                            self._pid_sifirla()
                        else:
                            # KABA MOD: mevcut surekli-heartbeat + PID hiz.
                            komut = self._yon_komutunu_hesapla(gx, gy)
                            # PID: aktif eksenin hatasindan orantili hiz uret.
                            # Hedef/hata yoksa PID durumunu sifirla ki bir
                            # sonraki kilitlenmede eski integral/turev deger
                            # yanlis baslangic yapmasin.
                            if komut in ('a', 'd'):
                                hiz = self._pid_hiz_hesapla(gx, 'pan')
                            elif komut in ('w', 's'):
                                hiz = self._pid_hiz_hesapla(gy, 'tilt')
                            else:
                                hiz = 0
                                self._pid_sifirla()

                            # 'x' (dur): sadece harekete gecerken bir kez gonder.
                            # Hareket komutlari: yon degisince hemen, ayni yondeyse
                            # heartbeat'i canli tutmak icin nudge_araligi'ni asmadan tekrar gonder.
                            komut_degisti = komut != son_komut
                            zaman_doldu = (su_an - son_nudge_zamani) >= self._nudge_araligi
                            if (komut == 'x' and komut_degisti) or (komut != 'x' and (komut_degisti or zaman_doldu)):
                                self._komut_gonder(komut, hiz)
                                son_komut = komut
                                son_nudge_zamani = su_an

                        # Lazer SADECE nisan noktasina TAM KILITLENDIGINDE
                        # (_kilitli_mi, histerezisli esik) acilir - paralaks
                        # duzeltmesi hesapla tabanli oldugu icin (goruntude
                        # dogrudan tespite dayanmadigi icin) gorsel geri
                        # besleme geregi yok, kilit bekleyerek daha guvenli.
                        if self._kilitli_mi:
                            self._lazer_gonder('l')
                            if not getattr(self, '_tam_kilit_log', False):
                                self.get_logger().info('🎯 Hedefe TAM KILITLENDI, lazer ACIK')
                                self._tam_kilit_log = True
                        else:
                            self._lazer_gonder('k')
                            if getattr(self, '_tam_kilit_log', False):
                                self.get_logger().info('Kilit kayboldu, lazer KAPALI')
                                self._tam_kilit_log = False
                    else:
                        self._lazer_gonder('k')
                        # MANUEL: gercek komut gonderimi _manuel_heartbeat
                        # zamanlayicisinda (kamera dongusunden bagimsiz,
                        # ~20Hz) yapiliyor. Burada sadece video overlay icin
                        # aktif olup olmadigina bakiyoruz.
                        if self._canli_mi(self._manuel_zaman) and (self._manuel_x != 0 or self._manuel_y != 0):
                            cv2.putText(frame, 'MANUEL NISAN AKTIF', (50, 50), 1, 2, (255, 165, 0), 2)

                    # Kameranin optik merkezi (goruntunun tam ortasi).
                    cv2.drawMarker(frame, (aim_center_x, aim_center_y), (0, 255, 255), 1, 40, 2)
                    cv2.putText(frame, 'KAMERA MERKEZ', (aim_center_x + 8, aim_center_y - 24),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)

                    # Sag ust kose: lidar'in olctugu (yumusatilmis) mesafe.
                    mesafe_yazi = (f'{self._son_mesafe_cm:.0f} cm'
                                   if self._son_mesafe_cm is not None else '-- cm')
                    cv2.putText(frame, f'LIDAR: {mesafe_yazi}', (frame.shape[1] - 170, 25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

                    ret_enc, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 40])
                    if ret_enc:
                        data = buffer.tobytes()
                        if len(data) < 65000:
                            try:
                                self._sock_video.sendto(data, (self._video_target_ip, self._video_target_port))
                            except Exception:
                                pass

                except Exception as e:
                    self.get_logger().error(f'Kare isleme hatasi, atlaniyor: {e}')
                    self._komut_gonder('x')
                    self._lazer_gonder('k')
                    time.sleep(0.2)

        finally:
            self._komut_gonder('x')
            self._lazer_gonder('k')
            cap.release()

    def destroy_node(self):
        self._calisiyor = False
        self._komut_gonder('x')
        self._lazer_gonder('k')
        time.sleep(0.1)
        if self._arduino and self._arduino.is_open:
            self._arduino.close()
        self._sock_video.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TurretNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
