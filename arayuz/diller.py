CEVIRILER = {
    "Türkçe": {
        # --- AYARLAR SAYFASI BAŞLIKLARI ---
        "ana_baslik": "SİSTEM AYARLARI",
        "genel_ayarlar": "GENEL AYARLAR",
        "sistem_dili": "Sistem Dili:",
        "tema": "Tema:",
        "bildirimler": "Bildirimler:",
        "kontrol_ayarlari": "KONTROL AYARLARI",
        "klavye_kontrol": "Klavye Kontrol",
        "guvenlik_kontrolleri": "GÜVENLİK KONTROLLERİ",
        "kullanici_adi": "Kullanıcı Adı",
        "sifre": "Şifre:",
        "telefon": "PWM Hız Sınırı:",
        "canli_log": "CANLI SİSTEM LOGLARI :",
        "goruntu_ayarlari": "GÖRÜNTÜ AYARLARI",
        "yayin_kalitesi": "Yayın Kalitesi",
        "fps_siniri": "FPS Sınırı:",
        "veri_yedekleme": "VERİ YEDEKLEME",
        "hedef_klasor": "Hedef Klasör:",
        "sifreleme": "Şifreleme:",
        
        # --- BUTONLAR VE DURUMLAR (Çökmeyi engelleyen kısım) ---
        "btn_acik": "Açık",
        "btn_kapali": "Kapalı",
        "btn_aktif": "Aktif",
        "btn_pasif": "Pasif",
        "btn_bagli": "Bağlı",
        "btn_baglantiyok": "Bağlantı Yok",
        "btn_kaydet": "Veri Kaydet",
        "btn_kaydi_durdur": "Kaydı Durdur",
        "btn_yedekle": "Şimdi Yedekle",
        "btn_yedeklemeyi_durdur": "Yedeklemeyi Durdur",
        "arac_baslik": "Araç ⚡",
        "yer_baslik": "Yer ⚡",
        "etap": "E T A P",


        "tema_koyu": "Koyu Mod",
        "tema_acik": "Açık Mod",
        
        "kalite_1080": "1080p - Yüksek",
        "kalite_720": "720p - Orta",
        "kalite_480": "480p - Düşük",
        
        "fps_60": "60 Kare/sn",
        "fps_30": "30 Kare/sn",

        "mod_manuel": "M A N U E L",
        "mod_otonom": "O T O N O M",

        "kamera_on": "ÖN",
        "kamera_arka": "ARKA",
        "kamera_silah": "SİLAH",

        "m_aktif": "AKTİF", "m_pasif": "PASİF", "m_motor": "MOTORLAR",
        "g_normal": "NORMAL", "g_kritik": "KRİTİK", "g_sistem": "GÜÇ SİSTEMİ",
        "gps_etkin": "ETKİN", "gps_pasif": "PASİF",
        "cam_hazir": "HAZIR", "cam_hata": "BAĞLANTI KOPTU",
        "sis_hazir": "SİSTEM : HAZIR", "sis_hata": "SİSTEM : HATA",
        "veri_kaydet": "Veri Kaydet", "lidar_baslik": "LİDAR",
        "lidar_taraniyor": "TARANIYOR...",



        "nav_baslik": "NAVİGASYON VE GÖREV PLANLAMA",

        "hakkinda_baslik": "SİSTEM BİLGİSİ VE LİSANS",
        
        # ÜÇ TIRNAK KULLANIYORUZ Kİ HTML KODLARI RAHATÇA ALTA GEÇSİN
        "hakkinda_metin": """<!DOCTYPE HTML PUBLIC "-//W3C//DTD HTML 4.0//EN" "http://www.w3.org/TR/REC-html40/strict.dtd">
<html><head><meta name="qrichtext" content="1" /><style type="text/css">
p, li { white-space: pre-wrap; }
</style></head><body style=" font-family:'Ubuntu'; font-size:16px; font-weight:400; font-style:normal;">
<p style=" margin-top:12px; margin-bottom:12px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;"><span style=" font-family:'Courier New','monospace'; font-size:16px; font-weight:600; color:#00ff00;">&gt; ANA BEYİN (EDGE AI):</span><span style=" font-family:'Courier New','monospace'; font-size:16px; color:#a0c0d0;"> NVIDIA Jetson (Otonom Sürüş &amp; Karar Mekanizması) </span></p>
<p style=" margin-top:12px; margin-bottom:12px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;"><span style=" font-family:'Courier New','monospace'; font-size:16px; font-weight:600; color:#00ff00;">&gt; ALT SİSTEM KONTROL:</span><span style=" font-family:'Courier New','monospace'; font-size:16px; color:#a0c0d0;"> Arduino Mega 2560 (Gerçek Zamanlı I/O ve PWM Kontrolü) </span></p>
<p style=" margin-top:12px; margin-bottom:12px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;"><span style=" font-family:'Courier New','monospace'; font-size:16px; font-weight:600; color:#00ff00;">&gt; HABERLEŞME PROTOKOLÜ:</span><span style=" font-family:'Courier New','monospace'; font-size:16px; color:#a0c0d0;"> </span><span style=" font-family:'Courier New','monospace'; font-size:16px; font-weight:600; color:#a0c0d0;">ROS 2 Humble</span><span style=" font-family:'Courier New','monospace'; font-size:16px; color:#a0c0d0;"> (DDS Middleware, Pub/Sub Mimarisi) &amp; Kriptolu 2.4 GHz Telemetri </span></p>
<p style=" margin-top:12px; margin-bottom:12px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;"><span style=" font-family:'Courier New','monospace'; font-size:16px; font-weight:600; color:#00ff00;">&gt; GCS ALTYAPISI:</span><span style=" font-family:'Courier New','monospace'; font-size:16px; color:#a0c0d0;"> Python 3.12 | PyQt5 | Asenkron QThread Motoru </span></p>
<hr />
<h3 style=" margin-top:14px; margin-bottom:12px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;"><span style=" font-family:'Courier New','monospace'; font-size:large; font-weight:600; color:#00ffff;">[ SENSÖR FÜZYONU VE DURUM YÖNETİMİ ]</span><span style=" font-family:'Courier New','monospace'; font-size:large; font-weight:600; color:#a0c0d0;"> </span></h3>
<ul type="square" style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; -qt-list-indent: 1;"><li style=" font-family:'Courier New','monospace'; color:#00ff00;" style=" margin-top:12px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;"><span style=" font-size:16px; font-weight:600; color:#e0e0e0;">LİDAR ALTYAPISI:</span><span style=" font-size:16px; color:#e0e0e0;"> 360 Derece Çevresel Tarama, Haritalama (SLAM) ve Engel Algılama</span><span style=" font-size:16px;"> </span></li>
<li style=" font-family:'Courier New','monospace'; color:#00ff00;" style=" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;"><span style=" font-size:16px; font-weight:600; color:#e0e0e0;">GÖRÜNTÜ İŞLEME:</span><span style=" font-size:16px; color:#e0e0e0;"> Düşük Gecikmeli (Low-Latency) Kamera Akışı ve Nesne Tespiti</span><span style=" font-size:16px;"> </span></li>
<li style=" font-family:'Courier New','monospace'; color:#00ff00;" style=" margin-top:0px; margin-bottom:12px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;"><span style=" font-size:16px; font-weight:600; color:#e0e0e0;">TELEMETRİ:</span><span style=" font-size:16px; color:#e0e0e0;"> 50ms Gecikme ile Canlı Durum Taraması ve Güvenlik Döngüsü (Fail-Safe)</span><span style=" font-size:16px;"> </span></li></ul>
<hr />
<h3 style=" margin-top:14px; margin-bottom:12px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;"><span style=" font-family:'Courier New','monospace'; font-size:large; font-weight:600; color:#00ffff;">[ TAKIM BİLGİSİ VE VİZYON ]</span><span style=" font-family:'Courier New','monospace'; font-size:large; font-weight:600; color:#a0c0d0;"> </span></h3>
<p style=" margin-top:12px; margin-bottom:12px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;"><span style=" font-family:'Courier New','monospace'; font-size:16px; font-weight:600; color:#00ff00;">&gt; GÖREV GÜCÜ:</span><span style=" font-family:'Courier New','monospace'; font-size:16px; color:#a0c0d0;"> [Takım Adını Buraya Yazın] </span></p>
<p style=" margin-top:12px; margin-bottom:12px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; line-height:160%;"><span style=" font-family:'Courier New','monospace'; font-size:16px; color:#e0e0e0;">Teknofest 2026 İnsansız Kara Araçları (İKA) kategorisi için bir araya gelen ekibimiz; yazılım, elektronik ve mekanik disiplinlerini kusursuz bir uyumla birleştirmektedir. Amacımız, sadece yarışma parkurunu tamamlamak değil; </span><span style=" font-family:'Courier New','monospace'; font-size:16px; font-weight:600; color:#e0e0e0;">&quot;Milli Teknoloji Hamlesi&quot;</span><span style=" font-family:'Courier New','monospace'; font-size:16px; color:#e0e0e0;"> vizyonu doğrultusunda, asimetrik harp koşullarında ve zorlu arazi şartlarında görev yapabilecek, yerli ve milli otonom sistemler için ölçeklenebilir bir Ar-Ge altyapısı sunmaktır. TUFAN, bu vizyonun sahadaki çelik yansımasıdır ve ROS 2 mimarisi ile gelecekteki sürü (swarm) operasyonlarına dahi bugünden hazırdır.</span><span style=" font-family:'Courier New','monospace'; font-size:16px; color:#a0c0d0;"> <br /></span></p>
<p align="right" style=" margin-top:12px; margin-bottom:12px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;"><span style=" font-family:'Courier New','monospace'; font-size:14px; font-weight:600; font-style:italic; color:#ff3333;">&quot;Geleceğin harekat sahasında, kontrol daima güvendedir.&quot;</span></p></body></html>"""
    },
    
    "English": {
        # --- SETTINGS PAGE HEADERS ---
        "ana_baslik": "SYSTEM SETTINGS",
        "genel_ayarlar": "GENERAL SETTINGS",
        "sistem_dili": "System Language:",
        "tema": "Theme:",
        "bildirimler": "Notifications:",
        "kontrol_ayarlari": "CONTROL SETTINGS",
        "klavye_kontrol": "Keyboard Control",
        "guvenlik_kontrolleri": "SECURITY CONTROLS",
        "kullanici_adi": "Username:",
        "sifre": "Password:",
        "telefon": "PWM Speed Limit:",
        "canli_log": "LIVE SYSTEM LOGS :",
        "goruntu_ayarlari": "DISPLAY SETTINGS",
        "yayin_kalitesi": "Stream Quality:",
        "fps_siniri": "FPS Limit:",
        "veri_yedekleme": "DATA BACKUP",
        "hedef_klasor": "Target Folder:",
        "sifreleme": "Encryption:",
        
        # --- BUTTONS AND STATUSES (Çökmeyi engelleyen kısım) ---
        "btn_acik": "On",
        "btn_kapali": "Off",
        "btn_aktif": "Active",
        "btn_pasif": "Inactive",
        "btn_bagli": "Connected",
        "btn_baglantiyok": "Offline",
        "btn_kaydet": "Save Data",
        "btn_kaydi_durdur": "Stop Recording",
        "btn_yedekle": "Backup Now",
        "btn_yedeklemeyi_durdur": "Stop Backup",
        "arac_baslik": "Vehicle ⚡",
        "yer_baslik": "Ground ⚡",
        "etap": "S T A G E",

        "tema_koyu": "Dark Mode",
        "tema_acik": "Light Mode",
        
        "kalite_1080": "1080p - High",
        "kalite_720": "720p - Medium",
        "kalite_480": "480p - Low",
        
        "fps_60": "60 Frames/sec",
        "fps_30": "30 Frames/sec",

        "mod_manuel": "M A N U A L",
        "mod_otonom": "A U T O",

        "kamera_on": "FRONT",
        "kamera_arka": "REAR",
        "kamera_silah": "WEAPON",

        "m_aktif": "ACTIVE", "m_pasif": "PASSIVE", "m_motor": "MOTORS",
        "g_normal": "NORMAL", "g_kritik": "CRITICAL", "g_sistem": "POWER SYSTEM",
        "gps_etkin": "ACTIVE", "gps_pasif": "PASSIVE",
        "cam_hazir": "READY", "cam_hata": "DISCONNECTED",
        "sis_hazir": "SYSTEM : READY", "sis_hata": "SYSTEM : ERROR",
        "veri_kaydet": "Save Data", "lidar_baslik": "LIDAR",
        "lidar_taraniyor": "SCANNING...",

        "nav_baslik": "NAVIGATION & MISSION PLANNING",

        "hakkinda_baslik": "SYSTEM INFORMATION & LICENSE",
        
        "hakkinda_metin": """<!DOCTYPE HTML PUBLIC "-//W3C//DTD HTML 4.0//EN" "http://www.w3.org/TR/REC-html40/strict.dtd">
<html><head><meta name="qrichtext" content="1" /><style type="text/css">
p, li { white-space: pre-wrap; }
</style></head><body style=" font-family:'Ubuntu'; font-size:16px; font-weight:400; font-style:normal;">
<p style=" margin-top:12px; margin-bottom:12px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;"><span style=" font-family:'Courier New','monospace'; font-size:16px; font-weight:600; color:#00ff00;">&gt; MAIN BRAIN (EDGE AI):</span><span style=" font-family:'Courier New','monospace'; font-size:16px; color:#a0c0d0;"> NVIDIA Jetson (Autonomous Driving &amp; Decision Mechanism) </span></p>
<p style=" margin-top:12px; margin-bottom:12px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;"><span style=" font-family:'Courier New','monospace'; font-size:16px; font-weight:600; color:#00ff00;">&gt; SUBSYSTEM CONTROL:</span><span style=" font-family:'Courier New','monospace'; font-size:16px; color:#a0c0d0;"> Arduino Mega 2560 (Real-Time I/O and PWM Control) </span></p>
<p style=" margin-top:12px; margin-bottom:12px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;"><span style=" font-family:'Courier New','monospace'; font-size:16px; font-weight:600; color:#00ff00;">&gt; COMMUNICATION PROTOCOL:</span><span style=" font-family:'Courier New','monospace'; font-size:16px; color:#a0c0d0;"> </span><span style=" font-family:'Courier New','monospace'; font-size:16px; font-weight:600; color:#a0c0d0;">ROS 2 Humble</span><span style=" font-family:'Courier New','monospace'; font-size:16px; color:#a0c0d0;"> (DDS Middleware, Pub/Sub Architecture) &amp; Encrypted 2.4 GHz Telemetry </span></p>
<p style=" margin-top:12px; margin-bottom:12px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;"><span style=" font-family:'Courier New','monospace'; font-size:16px; font-weight:600; color:#00ff00;">&gt; GCS INFRASTRUCTURE:</span><span style=" font-family:'Courier New','monospace'; font-size:16px; color:#a0c0d0;"> Python 3.12 | PyQt5 | Asynchronous QThread Engine </span></p>
<hr />
<h3 style=" margin-top:14px; margin-bottom:12px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;"><span style=" font-family:'Courier New','monospace'; font-size:large; font-weight:600; color:#00ffff;">[ SENSOR FUSION AND STATE MANAGEMENT ]</span><span style=" font-family:'Courier New','monospace'; font-size:large; font-weight:600; color:#a0c0d0;"> </span></h3>
<ul type="square" style="margin-top: 0px; margin-bottom: 0px; margin-left: 0px; margin-right: 0px; -qt-list-indent: 1;"><li style=" font-family:'Courier New','monospace'; color:#00ff00;" style=" margin-top:12px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;"><span style=" font-size:16px; font-weight:600; color:#e0e0e0;">LIDAR INFRASTRUCTURE:</span><span style=" font-size:16px; color:#e0e0e0;"> 360 Degree Environmental Scanning, Mapping (SLAM) and Obstacle Detection</span><span style=" font-size:16px;"> </span></li>
<li style=" font-family:'Courier New','monospace'; color:#00ff00;" style=" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;"><span style=" font-size:16px; font-weight:600; color:#e0e0e0;">IMAGE PROCESSING:</span><span style=" font-size:16px; color:#e0e0e0;"> Low-Latency Camera Stream and Object Detection</span><span style=" font-size:16px;"> </span></li>
<li style=" font-family:'Courier New','monospace'; color:#00ff00;" style=" margin-top:0px; margin-bottom:12px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;"><span style=" font-size:16px; font-weight:600; color:#e0e0e0;">TELEMETRY:</span><span style=" font-size:16px; color:#e0e0e0;"> Live Status Scanning with 50ms Latency and Security Loop (Fail-Safe)</span><span style=" font-size:16px;"> </span></li></ul>
<hr />
<h3 style=" margin-top:14px; margin-bottom:12px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;"><span style=" font-family:'Courier New','monospace'; font-size:large; font-weight:600; color:#00ffff;">[ TEAM INFORMATION AND VISION ]</span><span style=" font-family:'Courier New','monospace'; font-size:large; font-weight:600; color:#a0c0d0;"> </span></h3>
<p style=" margin-top:12px; margin-bottom:12px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;"><span style=" font-family:'Courier New','monospace'; font-size:16px; font-weight:600; color:#00ff00;">&gt; TASK FORCE:</span><span style=" font-family:'Courier New','monospace'; font-size:16px; color:#a0c0d0;"> [Insert Team Name Here] </span></p>
<p style=" margin-top:12px; margin-bottom:12px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px; line-height:160%;"><span style=" font-family:'Courier New','monospace'; font-size:16px; color:#e0e0e0;">Our team, coming together for the Teknofest 2026 Unmanned Ground Vehicles (UGV) category, seamlessly integrates software, electronics, and mechanical disciplines. Our goal is not just to complete the competition track; in line with the </span><span style=" font-family:'Courier New','monospace'; font-size:16px; font-weight:600; color:#e0e0e0;">&quot;National Technology Initiative&quot;</span><span style=" font-family:'Courier New','monospace'; font-size:16px; color:#e0e0e0;"> vision, it is to provide a scalable R&amp;D infrastructure for domestic and national autonomous systems capable of operating in asymmetrical warfare conditions and challenging terrain. TUFAN is the steel reflection of this vision in the field and is already prepared for future swarm operations with its ROS 2 architecture.</span><span style=" font-family:'Courier New','monospace'; font-size:16px; color:#a0c0d0;"> <br /></span></p>
<p align="right" style=" margin-top:12px; margin-bottom:12px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;"><span style=" font-family:'Courier New','monospace'; font-size:14px; font-weight:600; font-style:italic; color:#ff3333;">&quot;In the battlefield of the future, control is always secure.&quot;</span></p></body></html>"""
    }
}