#include <Arduino.h>

// TF02-Pro mesafe sensoru: Mega'nin donanimsal Serial3'u (pin 15=RX3,
// pin 14=TX3) kullanilir - SoftwareSerial ile test edildiginde surekli
// checksum hatasi aliniyordu (0 gecerli paket), Serial3'e gecince %100
// gecerli okuma alindi (bit-bang zamanlama sorunu). Olculen mesafe,
// paralaks duzeltmesi icin USB (Serial) uzerinden Jetson'a "D:<cm>\n"
// seklinde raporlanir.
#define lidarSerial Serial3
byte lidarBuf[9];
byte lidarIdx = 0;
int sonMesafeCm = -1; // -1 = henuz gecerli okuma yok
unsigned long sonMesafeRaporMs = 0;
const unsigned long MESAFE_RAPOR_ARALIGI_MS = 100;

// PAN (Sağ/Sol) Motoru Pinleri (kablolama capraz oldugu icin eskiden 11-12
// olan PAN, 5-6 olarak degistirildi)
const int PUL_PAN = 5;
const int DIR_PAN = 6;

// TILT (Yukarı/Aşağı) Motoru Pinleri (eskiden 5-6 olan TILT, 11-12 olarak
// degistirildi)
const int PUL_TILT = 11;
const int DIR_TILT = 12;

// LAZER: pin 9 bir role (relay) modulunun kontrol girisine bagli, lazerin
// kendisi ayri/role uzerinden besleniyor. Role AKTIF-DUSUK calisiyor (LOW
// = lazer ACIK, HIGH = lazer KAPALI) - bu yuzden duz digitalWrite kullanip
// mantigi tersine ceviriyoruz. (PWM/analogWrite KULLANILMIYOR: role dijital
// bir anahtar, "ortalama gerilim" kavrami role icin anlamsiz.)
const int LASER_PIN = 9;
const int LASER_ON_LEVEL = LOW;
const int LASER_OFF_LEVEL = HIGH;

// --- HIZ PROTOKOLU ---
// Hareket komutlari artik 2 bayt: [yon karakteri]['w','a','s','d'] + [hiz 0-255]
// 'x' (dur) tek bayt kalir, hiz baytina ihtiyaci yok.
// hiz=255 -> en hizli (PAN/TILT_DELAY_MIN_US), hiz=0 -> en yavas (..._MAX_US).
// Bu sayede ROS tarafinda gercek bir PID, hataya orantili degisken hiz
// gonderebiliyor (onceden hep sabit tam hizdi, bu da merkeze yakinken
// asiri duzeltip titremeye (oscillation) sebep oluyordu).
// DM556 mikro-adim DIP ayari 6400 -> 12800 pulses/rev'e cekildi (2 kat
// ince cozunurluk, "tam kilitlenememe" sorununu cozmek icin). Ayni ACISAL
// hizi korumak icin (adim basina yaricap hareket YARIYA dustugu icin)
// asagidaki TUM gecikme sabitleri 2'ye BOLUNDU - DIP ayari degisirse
// (baska bir pulses/rev secilirse) buradaki 4 deger de AYNI oranda
// yeniden olceklenmeli.
// GUNCELLEME: PAN ekseninde fiziksel bir redüktör (disli kutusu) oldugu
// icin ayni pulse hizinda TILT'e gore cok daha yavas donuyor (sahada
// gozlemlendi, "yatay eksen cok yavas") - PAN_DELAY_MIN/MAX_US bu yuzden
// TILT'ten BAGIMSIZ ayrica yariya dusuruldu (2 kat hizlandirildi). Hala
// yetersizse ayni oranda daha da kucultulebilir; asiri hizlanip
// overshoot/osilasyon baslarsa buyutulmeli.
const unsigned long PAN_DELAY_MIN_US = 50;
const unsigned long PAN_DELAY_MAX_US = 2000;
const unsigned long TILT_DELAY_MIN_US = 2000;
const unsigned long TILT_DELAY_MAX_US = 5000;

// Yeni komut/heartbeat bu sureden (ms) uzun sure gelmezse motor/lazer
// otomatik durur/soner.
const unsigned long GUVENLIK_ZAMAN_ASIMI_MS = 250;
// Yon baytindan sonra hiz bayti bu sureden (ms) uzun gelmezse bekleme iptal
// edilir (framing/senkron guvenligi - normalde <1ms icinde gelir).
const unsigned long CERCEVE_ZAMAN_ASIMI_MS = 50;

char aktifYon = 'x';              // 'w','a','s','d','x'
unsigned long aktifHizDelayUs = PAN_DELAY_MAX_US; // aktif eksene gore yorumlanir
unsigned long sonKomutZamaniMs = 0;

bool hizBekleniyor = false;
char bekleyenYon = 'x';
unsigned long yonAlinmaZamaniMs = 0;

// TEK ADIM PROTOKOLU: 2 bayt ['p']['w'/'a'/'s'/'d']. Surekli-heartbeat
// modundan (yukaridaki 'w'/'a'/'s'/'d'+hiz) TAMAMEN farkli: bu komut TEK
// bir step pulse'u uretir ve HEMEN durur - heartbeat/guvenlik zaman asimi
// GEREKMEZ (kendi kendine biter). Jetson tarafinda hedefe cok yaklasilinca
// (kucuk piksel hatasi) surekli-heartbeat modu her zaman en yavas hizda
// bile bir kare arasinda cok fazla adim atip asiri gidebiliyordu (kare
// hizi ~10-15fps oldugu icin iki kare arasinda onlarca adim atilabiliyor) -
// tek adim modu, hatayi Jetson'un GORDUGU her karede en fazla BIR fiziksel
// adimla azaltarak kare hizina baglirak hassas bir "sürünme" saglar.
bool adimBekleniyor = false;
// COKLU ADIM PROTOKOLU icin 2 asamali bekleme (once yon, sonra sayi bayti).
bool cokluAdimYonBekleniyor = false;
bool cokluAdimSayiBekleniyor = false;
char cokluAdimYonu = 'x';
// Tek adim pulse genislikleri: surucu datasheet'i icin guvenli pay,
// surekli modun en yavas hizindan (PAN/TILT_DELAY_MAX_US) COK daha kisa -
// tek atim oldugu icin loop()'u onemli olcude bloklamiyor.
const unsigned long ADIM_DIR_SETUP_US = 20;
const unsigned long ADIM_PULSE_US = 10;

bool panPulseYuksek = false;
unsigned long sonPanPulseZamaniUs = 0;

bool tiltPulseYuksek = false;
unsigned long sonTiltPulseZamaniUs = 0;

// BACKLASH (MEKANIK BOSLUK) TELAFISI: PAN ekseninde redüktör (disli
// kutusu) oldugu icin yon degistiginde motor birkac adim "bosa" doner,
// yuk gercekten hareket etmeye ONDAN SONRA baslar - bu, ozellikle HASSAS
// tek-adim modunda kucuk duzeltmelerin bir kismini tamamen etkisiz
// birakip osilasyona katkida bulunabiliyor. Cozum: eksenin YONU
// DEGISTIGINDE (bir onceki GERCEK harekete gore), gercek adimdan ONCE
// bu kadar "bedava" pulse atilir (yuku degil, sadece boslugu tuketir).
// BASLANGIC TAHMINI - sahada kalibre edilmeli: PAN'i bir yone birkac
// saniye surekli dondurup DURDUR, sonra TERS yone TEK ADIM'larla
// (Jetson tarafinda hassas mod ya da elle 'p'+yon) yukun GORULEBILIR
// sekilde hareket etmeye basladigi adim sayisini say, o sayiyi buraya yaz.
// 12800 pulses/rev mikro-adimla calisildigi icin (adim basina cok kucuk
// acisal hareket) boskuk adim cinsinden BUYUK bir sayi olabilir.
const int PAN_BACKLASH_ADIM = 40;
// GUNCELLEME: TILT'te redüktör olmasa da kaplin/dislide elle hissedilir
// bir bosluk oldugu sahada dogrulandi ("bazen tam vuruyor, bazen yukari
// cikip vuruyor" - yon degisiminde TELAFISIZ bosluk tam bu belirtiyi
// verir). 15 YETERSIZ kaldi (sahada dogrulandi, sorun devam etti) -
// belirgin sekilde arttirildi. Hala yetersizse ayni sekilde
// buyutulmeye devam edilmeli; asiri buyutulup fazla telafi
// (over-compensation, ters yonde kucuk bir sicrama) olursa kucultulmeli.
// TARIHCE: 90 -> 40 -> 10 -> 60 -> 90 -> 60 (2026-09-07, kullanici
// istegi: "backlashi 60'a geri cek"). Ara denemeler (40/10) sirasinda
// olcumler GECERSIZDI - bkz. asagidaki not.
// *** TUM BU DENEMELER GECERSIZDI ***: o sirada /silah_modu MANUEL'de
// takiliydi (arayuz her turda araç modunu /silah_modu'na yaziyor ve
// etap yoneticisinin OTONOM'unu eziyordu), yani turret_node'un TUM
// otonom hareket blogu ('if mod == OTONOM') kapaliydi ve TARET HIC
// HAREKET ETMIYORDU. "Yukari vuruyor / salinim yapiyor" gozlemleri
// otonom kilitlenmenin degil elle nisanin sonucuydu. Kullanicinin
// tespiti: "silahin ilk kodlari atis icin daha iyiydi". Baslangic
// degerine (90) donuldu; yeniden ayar YAPILACAKSA taret GERCEKTEN
// hareket ederken olculmelidir.
// Kademeli dusuruldu, ama 10'da lazer SISTEMATIK olarak YUKARI vurmaya
// basladi. Kritik kanit: nisan_bias_y_px HER IKI yonde de (-17.3 ve
// -11.3) denendi, IKISINDE DE "daha yukari vurdu". SABIT bir nisan
// kaymasi iki zit yonde birden kotulesemez - demek ki hata sabit bir
// ofset DEGIL, YON DEGISIMINE bagli histerezis, yani BACKLASH'in ta
// kendisi. Yukaridaki nota gore 15 zaten yetersiz kalmisti; 10 daha da
// azdi. 60'a geri donuldu (bu deger sahada calisiyordu) ve nisan_bias_y_px
// kalibre degeri -14.3'e geri alindi.
// DERS: backlash yetersizken nisan trim'iyle duzeltmeye calisma - trim
// sabit, backlash yone bagli; biri digerini telafi EDEMEZ.
// NOT: kayitli kaynakta bir ara 60 yaziyorken karta YUKLU firmware'de 90
// oldugu bildirildi - kaynak ile kart AYRISMISTI (ayni durum surus
// Arduino'sunda da yasandi). Her yukleme ikisini tekrar esitliyor.
// UYARI: yukaridaki tarihce notunda 15 adimin YETERSIZ kaldigi ve
// degerin bu yuzden buyutuldugu yaziyor. 10, o denemenin de ALTINDA -
// "bazen tam vuruyor, bazen yukari cikip vuruyor" belirtisi geri
// donerse deger tekrar buyutulmelidir.
const int TILT_BACKLASH_ADIM = 60;

// Her eksende SON GERCEKTEN hareket ettirilen yon ('a'/'d'/'w'/'s', 0 =
// henuz bilinmiyor - ilk hareket icin telafi uygulanmaz).
char sonPanYonGercek = 0;
char sonTiltYonGercek = 0;

// Yon (pulPin/dirPin/dirYuksek zaten ayarlanmis) ile backlash pulse'lari
// uretir - tekAdimAt'taki tek pulse ile AYNI kisa/guvenli zamanlama,
// sadece DIR degistirmeden N kez tekrarlanir.
void backlashTelafiUygula(int pulPin, int adimSayisi) {
  for (int i = 0; i < adimSayisi; i++) {
    digitalWrite(pulPin, HIGH);
    delayMicroseconds(ADIM_PULSE_US);
    digitalWrite(pulPin, LOW);
    delayMicroseconds(ADIM_PULSE_US);
  }
}

// DIR pin(i) ZATEN yeni yone ayarlanmisken cagrilir - bu eksende SON
// GERCEKTEN hareket edilen yonden FARKLIYSA (ve daha once bilinen bir yon
// varsa), gercek pulse'tan ONCE backlash telafisi uygulanir. Hem
// uygulaYon (KABA/surekli mod) hem tekAdimAt (HASSAS/tek-adim mod)
// tarafindan cagrilir - boylece boslugu tuketme mantigi IKI moddede de
// TUTARLI ve PAYLASIMLI (bir moddan digerine gecince de dogru calisir).
void backlashKontrolVeUygula(char yon, int pulPin) {
  char* sonYon = (yon == 'a' || yon == 'd') ? &sonPanYonGercek : &sonTiltYonGercek;
  int backlashAdim = (yon == 'a' || yon == 'd') ? PAN_BACKLASH_ADIM : TILT_BACKLASH_ADIM;
  if (*sonYon != 0 && *sonYon != yon && backlashAdim > 0) {
    backlashTelafiUygula(pulPin, backlashAdim);
  }
  *sonYon = yon;
}

// Lazer, hareketten TAMAMEN bagimsiz kendi komutu ('l' ac, 'k' kapat) ve
// kendi guvenlik zaman asimiyla kontrol edilir.
bool lazerAcikMi = false;
unsigned long sonLazerKomutZamaniMs = 0;

void hepsiniDurdur() {
  digitalWrite(PUL_PAN, LOW);
  digitalWrite(PUL_TILT, LOW);
  panPulseYuksek = false;
  tiltPulseYuksek = false;
}

unsigned long hizByteToDelay(byte hiz, char yon) {
  unsigned long dMin = (yon == 'a' || yon == 'd') ? PAN_DELAY_MIN_US : TILT_DELAY_MIN_US;
  unsigned long dMax = (yon == 'a' || yon == 'd') ? PAN_DELAY_MAX_US : TILT_DELAY_MAX_US;
  return dMax - ((unsigned long)hiz * (dMax - dMin)) / 255UL;
}

void uygulaYon(char cmd, byte hiz) {
  unsigned long simdiUs = micros();
  if (cmd != aktifYon) {
    // Eksen/yon degisti: temiz baslangic icin her iki pulse pinini sifirla
    hepsiniDurdur();

    if (cmd == 'w') { digitalWrite(DIR_TILT, HIGH); backlashKontrolVeUygula('w', PUL_TILT); }
    else if (cmd == 's') { digitalWrite(DIR_TILT, LOW); backlashKontrolVeUygula('s', PUL_TILT); }
    else if (cmd == 'a') { digitalWrite(DIR_PAN, HIGH); backlashKontrolVeUygula('a', PUL_PAN); }
    else if (cmd == 'd') { digitalWrite(DIR_PAN, LOW); backlashKontrolVeUygula('d', PUL_PAN); }

    sonPanPulseZamaniUs = simdiUs;
    sonTiltPulseZamaniUs = simdiUs;
  }
  aktifYon = cmd;
  if (cmd != 'x') {
    aktifHizDelayUs = hizByteToDelay(hiz, cmd);
  }
  sonKomutZamaniMs = millis();
}

void tekAdimAt(char yon) {
  // Guvenlik: surekli hareket aktifse once temiz durdur ki pulse pinleri
  // tutarsiz durumda kalmasin.
  if (aktifYon != 'x') {
    aktifYon = 'x';
    hepsiniDurdur();
  }

  int pulPin, dirPin;
  bool dirYuksek;
  if (yon == 'a')      { pulPin = PUL_PAN;  dirPin = DIR_PAN;  dirYuksek = true;  }
  else if (yon == 'd') { pulPin = PUL_PAN;  dirPin = DIR_PAN;  dirYuksek = false; }
  else if (yon == 'w') { pulPin = PUL_TILT; dirPin = DIR_TILT; dirYuksek = true;  }
  else if (yon == 's') { pulPin = PUL_TILT; dirPin = DIR_TILT; dirYuksek = false; }
  else return; // gecersiz yon, yok say

  digitalWrite(dirPin, dirYuksek ? HIGH : LOW);
  delayMicroseconds(ADIM_DIR_SETUP_US);
  backlashKontrolVeUygula(yon, pulPin);
  digitalWrite(pulPin, HIGH);
  delayMicroseconds(ADIM_PULSE_US);
  digitalWrite(pulPin, LOW);
}

// COKLU ADIM PROTOKOLU: 3 bayt ['b']['w'/'a'/'s'/'d'][adim_sayisi 0-255].
// tekAdimAt'in "kare basina 1 adim" siniri, hata buyukken kilitlenmeyi
// gereksiz yavaslatiyordu (sahada gozlemlendi: "kilitlenmesi cok uzun
// suruyor") - Jetson tarafi artik hatayi DOGRUDAN gereken adim sayisina
// cevirip TEK seferde gonderebiliyor (bkz. hassas_adim_kazanci). Backlash
// telafisi (varsa) SADECE bu N adimin BASINDA bir kez uygulanir, sonra N
// adim ARDI ARDINA (DIR degismeden) atilir - tekAdimAt ile AYNI guvenli
// pulse zamanlamasi kullanilir.
void cokluAdimAt(char yon, byte adimSayisi) {
  if (aktifYon != 'x') {
    aktifYon = 'x';
    hepsiniDurdur();
  }

  int pulPin, dirPin;
  bool dirYuksek;
  if (yon == 'a')      { pulPin = PUL_PAN;  dirPin = DIR_PAN;  dirYuksek = true;  }
  else if (yon == 'd') { pulPin = PUL_PAN;  dirPin = DIR_PAN;  dirYuksek = false; }
  else if (yon == 'w') { pulPin = PUL_TILT; dirPin = DIR_TILT; dirYuksek = true;  }
  else if (yon == 's') { pulPin = PUL_TILT; dirPin = DIR_TILT; dirYuksek = false; }
  else return;

  digitalWrite(dirPin, dirYuksek ? HIGH : LOW);
  delayMicroseconds(ADIM_DIR_SETUP_US);
  backlashKontrolVeUygula(yon, pulPin);
  for (byte i = 0; i < adimSayisi; i++) {
    digitalWrite(pulPin, HIGH);
    delayMicroseconds(ADIM_PULSE_US);
    digitalWrite(pulPin, LOW);
    delayMicroseconds(ADIM_PULSE_US);
  }
}

void lidarOku() {
  // Non-blocking: sadece hazir olan baytlari isler, loop() bloklanmaz.
  while (lidarSerial.available() > 0) {
    byte b = lidarSerial.read();
    if (lidarIdx < 2) {
      if (b == 0x59) {
        lidarBuf[lidarIdx++] = b;
      } else {
        lidarIdx = 0;
      }
    } else {
      lidarBuf[lidarIdx++] = b;
      if (lidarIdx == 9) {
        byte checksum = 0;
        for (byte i = 0; i < 8; i++) checksum += lidarBuf[i];
        if (checksum == lidarBuf[8]) {
          int mesafe = lidarBuf[2] | (lidarBuf[3] << 8);
          int sinyal = lidarBuf[4] | (lidarBuf[5] << 8);
          if (sinyal >= 100 && sinyal != 65535) {
            sonMesafeCm = mesafe;
          }
        }
        lidarIdx = 0;
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  lidarSerial.begin(115200);

  pinMode(PUL_PAN, OUTPUT);
  pinMode(DIR_PAN, OUTPUT);
  pinMode(PUL_TILT, OUTPUT);
  pinMode(DIR_TILT, OUTPUT);
  pinMode(LASER_PIN, OUTPUT);

  digitalWrite(PUL_PAN, LOW);
  digitalWrite(DIR_PAN, LOW);
  digitalWrite(PUL_TILT, LOW);
  digitalWrite(DIR_TILT, LOW);
  digitalWrite(LASER_PIN, LASER_OFF_LEVEL);
}

void loop() {
  unsigned long simdiMs = millis();
  unsigned long simdiUs = micros();

  lidarOku();
  if (simdiMs - sonMesafeRaporMs >= MESAFE_RAPOR_ARALIGI_MS) {
    Serial.print("D:");
    Serial.println(sonMesafeCm);
    sonMesafeRaporMs = simdiMs;
  }

  // 0) Ikinci/ucuncu bayt bekleniyor ama cok uzun suredir gelmediyse iptal (framing guvenligi)
  if ((hizBekleniyor || adimBekleniyor || cokluAdimYonBekleniyor || cokluAdimSayiBekleniyor)
      && (simdiMs - yonAlinmaZamaniMs > CERCEVE_ZAMAN_ASIMI_MS)) {
    hizBekleniyor = false;
    adimBekleniyor = false;
    cokluAdimYonBekleniyor = false;
    cokluAdimSayiBekleniyor = false;
  }

  // 1) Yeni seri bayt var mi?
  if (Serial.available() > 0) {
    if (!hizBekleniyor && !adimBekleniyor && !cokluAdimYonBekleniyor && !cokluAdimSayiBekleniyor) {
      char cmd = Serial.read();
      if (cmd == ' ') cmd = 'x';

      if (cmd == 'x') {
        uygulaYon('x', 0); // 'x' hiz baytina ihtiyac duymaz, hemen uygula
      } else if (cmd == 'w' || cmd == 'a' || cmd == 's' || cmd == 'd') {
        bekleyenYon = cmd;
        hizBekleniyor = true;
        yonAlinmaZamaniMs = simdiMs;
      } else if (cmd == 'l' || cmd == 'k') {
        // Lazer ac/kapat - hareket durumundan bagimsiz kendi heartbeat'i.
        lazerAcikMi = (cmd == 'l');
        sonLazerKomutZamaniMs = simdiMs;
      } else if (cmd == 'p') {
        // Tek adim: bir sonraki bayt yon karakteri ('w'/'a'/'s'/'d').
        adimBekleniyor = true;
        yonAlinmaZamaniMs = simdiMs;
      } else if (cmd == 'b') {
        // Coklu adim: sonraki 2 bayt yon + adim_sayisi.
        cokluAdimYonBekleniyor = true;
        yonAlinmaZamaniMs = simdiMs;
      }
      // Taninmayan karakterleri yok say.
    } else if (hizBekleniyor) {
      byte hiz = (byte)Serial.read();
      uygulaYon(bekleyenYon, hiz);
      hizBekleniyor = false;
    } else if (adimBekleniyor) {
      char yon = Serial.read();
      tekAdimAt(yon);
      adimBekleniyor = false;
    } else if (cokluAdimYonBekleniyor) {
      cokluAdimYonu = Serial.read();
      cokluAdimYonBekleniyor = false;
      cokluAdimSayiBekleniyor = true;
      yonAlinmaZamaniMs = simdiMs;
    } else { // cokluAdimSayiBekleniyor
      byte sayi = (byte)Serial.read();
      cokluAdimAt(cokluAdimYonu, sayi);
      cokluAdimSayiBekleniyor = false;
    }
  }

  // 2) Guvenlik zaman asimi: heartbeat kesilirse motor/lazer otomatik durur.
  if (aktifYon != 'x' && (simdiMs - sonKomutZamaniMs > GUVENLIK_ZAMAN_ASIMI_MS)) {
    aktifYon = 'x';
    hepsiniDurdur();
  }
  if (lazerAcikMi && (simdiMs - sonLazerKomutZamaniMs > GUVENLIK_ZAMAN_ASIMI_MS)) {
    lazerAcikMi = false;
  }

  // 3) Non-blocking adim uretimi: sadece aktif eksende, delayMicroseconds
  //    YOK - loop() serial okumaya/timeout kontrolune devam edebiliyor.
  if (aktifYon == 'a' || aktifYon == 'd') {
    if (simdiUs - sonPanPulseZamaniUs >= aktifHizDelayUs) {
      panPulseYuksek = !panPulseYuksek;
      digitalWrite(PUL_PAN, panPulseYuksek ? HIGH : LOW);
      sonPanPulseZamaniUs = simdiUs;
    }
  } else if (aktifYon == 'w' || aktifYon == 's') {
    if (simdiUs - sonTiltPulseZamaniUs >= aktifHizDelayUs) {
      tiltPulseYuksek = !tiltPulseYuksek;
      digitalWrite(PUL_TILT, tiltPulseYuksek ? HIGH : LOW);
      sonTiltPulseZamaniUs = simdiUs;
    }
  }

  digitalWrite(LASER_PIN, lazerAcikMi ? LASER_ON_LEVEL : LASER_OFF_LEVEL);
}
