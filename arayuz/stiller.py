# ==========================================
# TUFAN GCS - ARAYÜZ TASARIM STİLLERİ (CSS)
# ==========================================

# 1. AYARLAR SAYFASI BUTONLARI İÇİN ORTAK STİLLER
AYAR_AKTIF = """
    QPushButton { background-color: #00bfff; color: white; border-radius: 20px; font-weight: bold; font-size: 16px; }
    QPushButton:hover { background-color: #008cba; border: 2px solid white; }
"""
AYAR_PASIF = """
    QPushButton { background-color: transparent; color: #00bfff; border: 2px solid #00bfff; border-radius: 20px; font-weight: bold; font-size: 16px; }
    QPushButton:hover { background-color: rgba(0, 191, 255, 40); color: white; border: 2px solid white; }
"""

# 2. MANUEL / OTONOM GEÇİŞ BUTONLARI İÇİN
MOD_AKTIF = """
    QPushButton { background-color: #0055ff; color: white; border-radius: 16px; font-weight: bold; font-size: 28px; }
    QPushButton:hover { background-color: #3377ff; }
"""
MOD_PASIF = """
    QPushButton { background-color: transparent; color: #555555; border: 2px solid #555555; border-radius: 16px; font-weight: bold; font-size: 28px; }
    QPushButton:hover { background-color: rgba(85, 85, 85, 50); border: 2px solid #777777; color: #ffffff; }
"""

# 3. VERİ KAYIT VE YEDEKLEME İÇİN
KAYIT_AKTIF = """
    QPushButton { background-color: darkred; color: white; border-radius: 24px; font-weight: bold; font-size: 24px; }
    QPushButton:hover { background-color: #ff0000; border: 2px solid white; }
"""
KAYIT_PASIF = """
    QPushButton { background-color: transparent; color: white; border: 2px solid #0055ff; border-radius: 24px; font-weight: bold; font-size: 24px; }
    QPushButton:hover { background-color: rgba(0, 85, 255, 50); border: 2px solid #00aaff; }
"""

YEDEK_AKTIF = """
    QPushButton { background-color: #0F4C81; color: white; border-radius: 16px; font-weight: bold; font-size: 26px; letter-spacing: 5px; }
    QPushButton:hover { background-color: #0F4C81; border: 2px solid white; }
"""
YEDEK_PASIF = """
    QPushButton { background-color: transparent; color: #0F4C81; border: 2px solid #0F4C81; border-radius: 16px; font-weight: bold; font-size: 26px; letter-spacing: 10px; }
    QPushButton:hover { background-color: rgba(0, 85, 255, 40); color: white; border: 2px solid white; }
"""

# ... (Üstteki AYAR_AKTIF, MOD_AKTIF kısımları aynen kalıyor) ...

# ... (AYAR_AKTIF, MOD_AKTIF vb. kısımlar aynen kalıyor) ...

# stiller.py dosyasının içindeki TEMA_ACIK değişkeni:

TEMA_ACIK = """
/* ==========================================
   1. GENEL ARKA PLANLAR (Açık Mavi / Krem Tonu)
   ========================================== */
QMainWindow, #centralwidget, #icerik_frame, #sol_menu_frame, #alt_bar, QStackedWidget > QWidget {
    background-color: #E8F0F4; /* İstediğin ferah, açık buz mavisi zemin */
}

/* ==========================================
   2. YAZI RENKLERİ (Okunabilirlik için Koyu Lacivert/Siyah)
   ========================================== */
QWidget {
    color: #1A1C20;
}

/* Ayarlar sayfasındaki başlık yazıları (Eski gri yazılar şimdi koyu lacivert) */
#label_sistemDili, #label_tema, #label_bildirimler, #label_VPN, #label_kullaniciAdi, #label_sifre, #label_telefon, #label_canliSistemLog, #label_joystick, #label_klavyeKontrol, #label_yayinKalite, #label_FPS, #label_hedefKlasor, #label_sifreleme {
    color: #334155; 
}

/* Genel Label Ayarları */
QLabel {
    background-color: transparent;
    border: none;
    font-weight: bold;
}

#label_sistemYazi {
    font-size: 32px; /* İstediğin büyüklüğe göre artırabilirsin */
    font-family: "Arial";
}

/* Sağ alttaki şarj yüzdeleri (Objelerin isimlerini Qt Designer'dan kontrol edip buraya yaz) */
#label_aracSarj, #label_yerSarj{
    font-size: 26px; 
    font-family: "Arial";
}

/* ==========================================
   3. ANA SAYFA KUTULARI (Kavis: 24px)
   ========================================== */
#frame_motorlar, #frame_Guc, #frame_lidar, #frame_wifi, #frame_GPS, #frame_kamera {
    background-color: #FFFFFF; /* Kutuların içi bembeyaz temiz dursun */
    border: 2px solid #B0C4DE; /* Yumuşak bir açık mavi-gri çerçeve */
    border-radius: 24px;       /* Orijinal tasarımındaki kavis */
}

/* ==========================================
   4. ALT BAR VE AYAR KUTULARI (Kavis: 16px)
   ========================================== */
#frame_hiz, #frame_etap, #frame_aracBatarya, #frame_yerBatarya,
#frame_genelAyarlar, #frame_kontrolAyarlari, #frame_guvenlikKontrolleri, #frame_goruntuAyarlari, #frame_veriYedekleme {
    background-color: #FFFFFF;
    border: 2px solid #B0C4DE; 
    border-radius: 16px;       /* Orijinal tasarımındaki kavis */
}

/* ==========================================
   5. GİRDİ KUTULARI VE COMBOBOX'LAR
   ========================================== */
QLineEdit, QComboBox {
    background-color: #F8FAFC; /* Çok çok uçuk mavi/beyaz iç zemin */
    color: #000000;
    border: 2px solid #B0C4DE;
    border-radius: 12px;       /* Orijinal kavis */
    padding: 5px 10px;
    font-size: 22px;
}
QLineEdit:focus, QComboBox:hover {
    border: 2px solid #00AAFF; /* Tıklayınca orijinalindeki gibi mavi parlasın */
    background-color: #FFFFFF;
}
QComboBox QAbstractItemView {
    background-color: #FFFFFF;
    color: #000000;
    border: 2px solid #00AAFF;
    border-radius: 12px;
}
QComboBox QAbstractItemView::item {
    color: #000000;
}
QComboBox QAbstractItemView::item:selected {
    background-color: #00AAFF;
    color: #FFFFFF;
}

/* ==========================================
   6. İSTİSNALAR (Koyu ve Askeri Kalması Gereken Yerler)
   ========================================== */
/* Kamera ekranları, Log terminali ve Bilgi Kartı açık modda dahi KOYU kalmalı */
#frame_anaKamera, #frame_yanKamera1, #frame_yanKamera2, #frame_yanKamera3, 
#log_ekran, #frame_BilgiKart, #frame_navigasyonAlan {
    background-color: #05050A; /* Zifiri Karanlık */
}

/* Log Terminali Özel Renkleri (Matrix Yeşili) */
#textEdit_canliSistemLog {
    background-color: #05050A;
    color: #00FF00;
    border: 2px solid #00FF41;
    border-radius: 48px; /* Orijinal kavis */
}

/* Hakkında Sayfası TextBrowser */
#textBrowser_Bilgi {
    background-color: transparent;
    color: #FFFFFF;
    border: none;
}
"""

