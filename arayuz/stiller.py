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

# 4. PWM HIZ SINIRI KUTUSU (eski "Telefon" kutusu) - diger tum ayarlar
# kutularindan (kullanici adi, sifre - hepsi notr gri/beyaz) BILEREK farkli
# bir kimlikte (kehribar/turuncu, hem kutu hem yazi rengi) - her zaman,
# deger girilmemisken bile, goze carpsin diye. Aktif (bir sinir onaylanmis)
# durumda renk daha da doygunlasiyor.
PWM_LIMIT_VARSAYILAN = """
    QLineEdit {
        background-color: #2A1D0E;
        color: #FFB454;
        border: 2px solid #B5750F;
        border-radius: 12px;
        padding: 4px 9px;
        font-size: 26px;
        font-weight: bold;
        font-family: "Roboto Mono";
    }
    QLineEdit::placeholder { color: #8A6A3A; }
    QLineEdit:focus { border: 2px solid #FFB454; background-color: #331F0A; }
"""
PWM_LIMIT_AKTIF = """
    QLineEdit {
        background-color: #3A2400;
        color: #FFD27A;
        border: 2px solid #FF9F1C;
        border-radius: 12px;
        padding: 4px 9px;
        font-size: 26px;
        font-weight: bold;
        font-family: "Roboto Mono";
    }
    QLineEdit:focus { border: 2px solid #FFC266; background-color: #402A00; }
"""

# ... (Üstteki AYAR_AKTIF, MOD_AKTIF kısımları aynen kalıyor) ...

# ... (AYAR_AKTIF, MOD_AKTIF vb. kısımlar aynen kalıyor) ...

# stiller.py dosyasının içindeki TEMA_ACIK değişkeni:

TEMA_ACIK = """
/* ==========================================
   1. GENEL ARKA PLANLAR (Açık Mavi / Krem Tonu)
   ========================================== */
/* 2026-10-28 PALET REVIZYONU (kullanici: "hepsini tek renk yapmaktansa
   acik tonlarda baska renkler de kullanilabilir, daha sik olur").
   Artik her sey #FFFFFF degil - renk ROLE gore ayrisiyor:
     sayfa zemini  #EDF2F7  (en acik, notr)
     yan menu/alt bar #E986... hayir: #E981 degil -> #E3EBF2 (hafif koyu,
                      icerikten AYRISIR, cerceve cizmeye gerek kalmaz)
     kart/kutu     #FFFFFF  (en one cikan katman)
     grup basligi  #DCE9F5  (bant hissi - ayarlar sayfasindaki bolum
                             basliklarini icerikten ayirir)
     alt bar kutusu #F3F8FC (karttan bir ton acik, bar icinde yumusak)
     girdi kutusu  #F8FAFC
   Boylece katman hiyerarsisi RENKLE kuruluyor, her yer beyaz olmuyor. */
QMainWindow, #centralwidget, #icerik_frame, QStackedWidget > QWidget {
    background-color: #EDF2F7;
}
#sol_menu_frame, #alt_bar {
    background-color: #E3EBF2;
}

/* ==========================================
   2. YAZI RENKLERİ (Okunabilirlik için Koyu Lacivert/Siyah)
   ========================================== */
QWidget {
    color: #1A1C20;
}

/* Ayarlar sayfasındaki başlık yazıları (Eski gri yazılar şimdi koyu lacivert) */
#label_sistemDili, #label_tema, #label_bildirimler, #label_telefon,
#label_canliSistemLog, #label_joystick, #label_klavyeKontrol, #label_sifreleme {
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
    background-color: #FFFFFF;
    border: 2px solid #AFC3D9;
    border-radius: 24px;
}

/* ==========================================
   4. ALT BAR VE AYAR KUTULARI (Kavis: 16px)
   ========================================== */
/* Alt bardaki gosterge kutulari - bar zemininden (#E3EBF2) bir ton ACIK */
#frame_hiz, #frame_etap, #frame_aracBatarya, #frame_yerBatarya {
    background-color: #F3F8FC;
    border: 2px solid #AFC3D9;
    border-radius: 16px;
}

/* Ayarlar sayfasi BOLUM BASLIKLARI - bant hissi veren tonlu zemin */
#frame_genelAyarlar, #frame_kontrolAyarlari,
#frame_guvenlikKontrolleri, #frame_veriYedekleme {
    background-color: #DCE9F5;
    border: 2px solid #9FB8D0;
    border-radius: 16px;
}
#label_genelAyarlarYazi, #label_kontrolAyarlariYazi,
#label_guvenlikKontrolleriYazi, #label_veriYedeklemeYazi {
    color: #16415F;   /* bant uzerinde kontrastli koyu lacivert */
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
/* SECICI OZGULLUGU (2026-10-28): widget'larin KENDI stili
   "QFrame#log_ekran { ... }" seklinde yazilmis. Buradaki kural sadece
   "#log_ekran" olunca OZGULLUGU DUSUK kaliyor ve zemin uygulanmiyordu -
   log terminali acik temada BEYAZ cikiyordu (render ile goruldu).
   Sinif adi eklenerek ozgulluk esitlendi. */
QFrame#frame_anaKamera, QFrame#frame_yanKamera1, QFrame#frame_yanKamera2,
QFrame#frame_yanKamera3, QFrame#log_ekran, QFrame#frame_BilgiKart,
QFrame#frame_navigasyonAlan {
    background-color: #05050A; /* Zifiri Karanlık */
}

/* Log Terminali Özel Renkleri (Matrix Yeşili) */
#textEdit_canliSistemLog {
    background-color: #05050A;
    color: #00FF00;
    border: 2px solid #00FF41;
    border-radius: 48px; /* Orijinal kavis */
}

/* ==========================================
   7. 2026-10-28 EKLENDI - RENK ARTIK TEK KAYNAKTAN
   ==========================================
   Tema gecisi artik widget stillerini SILMIYOR, sadece RENK bildirimlerini
   soyuyor (bkz. main.py::_stil_renklerini_soy). Yani border-radius/padding
   widget'in kendinde kalir, RENK buradan gelir. Asagidakiler eskiden
   TEMA_ACIK'ta hic gecmedigi icin acik temada renksiz kaliyordu. */

/* Ortadaki seffaf konteyner - acik temada da seffaf kalmali */
#frame_merkezAlan {
    background-color: transparent;
}

/* Ana menudeki dekoratif halka (175px kavis).
   2026-10-28 (kullanici: "acik temada ana ekranda orta halkanin
   gozukmesi gibi"): koyu temada rgba(0,0,0,50) koyu zemine karisip hic
   belli olmuyor. Acik zeminde ise EN KUCUK dolgu bile goze batiyor -
   olculdu: alfa 10 ile zemin #EDF2F7 -> disk #E7EDF3, yani ekranin
   ortasinda 6 ton koyu GRI BIR DAIRE. Bu yuzden acik temada cerceve
   TAMAMEN SAYDAM; radar halkasini zaten ustundeki label_radar_cember
   kesikli #00bfff elips olarak ciziyor.
   SECICI OZGULLUGU: widget kendi stilinde "QFrame#frame_halka" yaziyor;
   sadece "#frame_halka" demek OZGULLUK olarak YETMEZ (log_ekran'da
   yasanan AYNI tuzak) - sinif adi bu yuzden yazili. Kesin cozum icin
   ayrica main.py::ACIK_TEMA_OZEL widget'in KENDI stilini de saydam
   yapiyor; ikisi birbiriyle tutarli olmali. */
QFrame#frame_halka {
    background-color: transparent;
    border: none;
}

/* Butonlar: ayarlar ekranindaki ac/kapa butonlari koyu temada koyu zeminli.
   Renkleri soyuldugu icin burada acik tema karsiligi veriliyor; kavis ve
   dolgu widget'in kendi stilinden geliyor. */
QPushButton {
    background-color: #FFFFFF;
    color: #1A1C20;
    border-color: #B0C4DE;
}
QPushButton:hover {
    background-color: #F1F5F9;
    border-color: #00AAFF;
}
QPushButton:pressed {
    background-color: #E2E8F0;
}

/* Kamera/log/navigasyon KOYU kalan alanlarin UZERINDEKI yazilar beyaz
   kalmali - QWidget kuralindaki koyu renk oralarda okunmuyordu. */
#frame_anaKamera QLabel, #frame_yanKamera1 QLabel, #frame_yanKamera2 QLabel,
#frame_yanKamera3 QLabel, #frame_BilgiKart QLabel, #frame_navigasyonAlan QLabel {
    color: #FFFFFF;
}

/* SOL MENU IKON BUTONLARI: kendi koyu zeminleri soyulunca acik temada
   serit serit gorunuyordu (render ile goruldu). Menu zeminiyle ayni
   olsunlar, sadece uzerine gelince/secilince belli olsunlar. */
#home_button, #kamera_button, #navigasyon_button, #hakkinda_button,
#ayarlar_button, #power_button, #pushButton_acilKapat {
    background-color: transparent;
    border: none;
    border-radius: 16px;
}
#home_button:hover, #kamera_button:hover, #navigasyon_button:hover,
#hakkinda_button:hover, #ayarlar_button:hover, #power_button:hover,
#pushButton_acilKapat:hover {
    background-color: #D2E0EC;
}

/* KAYDIRMA CUBUGU (2026-10-28, kullanici: "canli log ekraninin asagi
   kaydirma alanini biraz kisalt, curve oldugu icin az miktarda tasiyor").
   Log kutusunun kavisi 48px; cubuk tam yuksekligi kaplayinca kose
   egrisinin DISINA tasiyordu. Ust/alt bosluk birakilarak kisaltildi ve
   tutamak yuvarlatildi. Ayni kural koyu temada da gecerli olsun diye
   tasarim.ui'deki stile de eklendi. */
QTextEdit QScrollBar:vertical, QPlainTextEdit QScrollBar:vertical {
    background: transparent;
    width: 10px;
    margin: 26px 4px 26px 0;
    border: none;
}
QTextEdit QScrollBar::handle:vertical, QPlainTextEdit QScrollBar::handle:vertical {
    background: #9FB8D0;
    border-radius: 5px;
    min-height: 28px;
}
QTextEdit QScrollBar::add-line:vertical, QTextEdit QScrollBar::sub-line:vertical,
QPlainTextEdit QScrollBar::add-line:vertical, QPlainTextEdit QScrollBar::sub-line:vertical {
    height: 0px;
}

/* Hakkında Sayfası TextBrowser */
#textBrowser_Bilgi {
    background-color: transparent;
    color: #FFFFFF;
    border: none;
}
"""

