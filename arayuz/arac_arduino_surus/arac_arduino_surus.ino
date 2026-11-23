#include <Servo.h>

// MOTOR SİNYAL PİNLERİ
#define SOL_MOTOR_PIN 5
#define SAG_MOTOR_PIN 6

// FAR RÖLE PİNLERİ (2026-09-01, kullanıcı isteği) - AKTİF-DÜŞÜK: pin LOW
// olunca farlar AÇILIR, HIGH olunca KAPANIR. Pin 2/3 daha önce hiç
// kullanılmıyordu (motor pinleri 5/6), çakışma yok.
#define FAR_PIN_1 2
#define FAR_PIN_2 3

// FREN MOTORU - L298N (2026-09-06, kullanici istegi: "arduino unoya
// l298n motor surucu bagladim, 11 ve 10. pinlere bagli; yer
// istasyonundaki fiziksel manuel/otonom butonu fren olsun, bastigimizda
// freni acsin tekrar bastigimizda kapatsin, 1 kere basinca 3 sn motoru
// calistirsin").
//
// PIN CAKISMASI YOK - DOGRULANDI: bu sketch motorlar icin 5/6, farlar
// icin 2/3 kullaniyor; 10 ve 11 bosta.
//
// KABLOLAMA VARSAYIMI: 11 -> L298N IN1, 10 -> L298N IN2, ENA jumper ile
// 5V'ta (yani hiz sabit, sadece yon kontrolu var). Fren ters yone
// calisiyorsa TEK SATIR duzeltme: asagidaki FREN_AC_IN1 / FREN_AC_IN2
// degerlerini birbiriyle degistirin.
#define FREN_IN1 11
#define FREN_IN2 10
const uint8_t FREN_AC_IN1  = HIGH;  // "fren AC" yonu
const uint8_t FREN_AC_IN2  = LOW;
const unsigned long FREN_CALISMA_MS = 3000;  // her komutta motor 3 sn doner

Servo solMotor;
Servo sagMotor;

// PPM için Merkez (Durma), Maksimum ve Minimum sınır değerleri (Mikrosaniye cinsinden)
const int PPM_DUR = 1500;
const int PPM_MIN = 1000;
const int PPM_MAX = 2000;

// --- FREN DURUMU ---
// frenIstenen : Jetson'dan gelen HEDEF durum (0 = kapali, 1 = acik)
// frenUygulanan : en son FIILEN uygulanmis durum
// Motor SADECE bu ikisi farklilastiginda (kenar tetikleme) 3 sn calisir;
// Jetson ayni degeri saniyede 10 kez gonderse bile TEKRAR tetiklenmez.
// Arduino resetlenirse ikisi de 0 baslar ve motor KENDILIGINDEN DONMEZ -
// fiziksel fren neredeyse orada kalir (acilista hareket YOK).
int frenIstenen = 0;
int frenUygulanan = 0;
bool frenCalisiyor = false;
unsigned long frenBaslamaMs = 0;

// İletişim koparsa aracı durdurmak için zaman aşımı (Watchdog) sayacı
unsigned long sonKomutZamani = 0;
const unsigned long ZAMAN_ASIMI_MS = 500; // 0.5 saniye sinyal gelmezse motorları durdur

void setup() {
  solMotor.attach(SOL_MOTOR_PIN);
  sagMotor.attach(SAG_MOTOR_PIN);

  pinMode(FAR_PIN_1, OUTPUT);
  pinMode(FAR_PIN_2, OUTPUT);
  digitalWrite(FAR_PIN_1, HIGH);  // baslangicta KAPALI (aktif-dusuk)
  digitalWrite(FAR_PIN_2, HIGH);

  // Fren motoru acilista SERBEST (iki giris de LOW = L298N cikisi bos).
  pinMode(FREN_IN1, OUTPUT);
  pinMode(FREN_IN2, OUTPUT);
  digitalWrite(FREN_IN1, LOW);
  digitalWrite(FREN_IN2, LOW);

  // Başlangıçta ESC'leri aktif edebilmek (arming) ve kalibre etmek için 1500ms (Dur) sinyali gönder
  solMotor.writeMicroseconds(PPM_DUR);
  sagMotor.writeMicroseconds(PPM_DUR);

  Serial.begin(115200);
  delay(1000);
  sonKomutZamani = millis();
}

void loop() {
  // 1. PYTHON'DAN GELEN VERIYI OKU. Eski formatlar HALA DESTEKLENIR
  //    (alanlar opsiyonel, geriye donuk uyumluluk icin):
  //      "SOL_PPM,SAG_PPM\n"
  //      "SOL_PPM,SAG_PPM,FAR\n"            FAR : 0=kapali, 1=acik
  //      "SOL_PPM,SAG_PPM,FAR,FREN\n"       FREN: 0=kapali, 1=acik
  if (Serial.available()) {
    String veri = Serial.readStringUntil('\n');
    veri.trim();

    // Virgullerin yerini bul ve veriyi parcala
    int virgul1 = veri.indexOf(',');
    if (virgul1 > 0) {
      int virgul2 = veri.indexOf(',', virgul1 + 1);
      String solStr = veri.substring(0, virgul1);
      String sagStr = (virgul2 > 0) ? veri.substring(virgul1 + 1, virgul2)
                                     : veri.substring(virgul1 + 1);

      int solPPM = solStr.toInt();
      int sagPPM = sagStr.toInt();

      // Güvenlik sınırlandırması (1000 - 2000 dışına çıkılmasını donanımsal engelle)
      solPPM = constrain(solPPM, PPM_MIN, PPM_MAX);
      sagPPM = constrain(sagPPM, PPM_MIN, PPM_MAX);

      // Motorlara hassas PPM sinyallerini yaz (Farklı palet hızları burada gerçekleşir!)
      solMotor.writeMicroseconds(solPPM);
      sagMotor.writeMicroseconds(sagPPM);

      // Far durumu (3. alan varsa) - aktif-dusuk: 1 -> LOW (acik), 0 -> HIGH (kapali)
      int virgul3 = (virgul2 > 0) ? veri.indexOf(',', virgul2 + 1) : -1;
      if (virgul2 > 0) {
        String farStr = (virgul3 > 0) ? veri.substring(virgul2 + 1, virgul3)
                                      : veri.substring(virgul2 + 1);
        int farDurumu = farStr.toInt();
        digitalWrite(FAR_PIN_1, farDurumu ? LOW : HIGH);
        digitalWrite(FAR_PIN_2, farDurumu ? LOW : HIGH);
      }

      // Fren hedef durumu (4. alan varsa). Burada SADECE hedef guncellenir;
      // motoru calistirma karari asagidaki kenar-tetikleme blogunda verilir.
      if (virgul3 > 0) {
        frenIstenen = veri.substring(virgul3 + 1).toInt() ? 1 : 0;
      }

      // Watchdog sayacını sıfırla
      sonKomutZamani = millis();
    }
  }

  // 2. DONANIM KORUMASI (Watchdog - Eğer Jetson/PC bağlantısı koparsa aracı anında durdur)
  if (millis() - sonKomutZamani > ZAMAN_ASIMI_MS) {
    solMotor.writeMicroseconds(PPM_DUR);
    sagMotor.writeMicroseconds(PPM_DUR);
  }

  // 3. FREN MOTORU - KENAR TETIKLEMELI, BLOKLAMAYAN 3 SANIYE
  //    *** delay(3000) KULLANILMIYOR - KRITIK ***: bloklayan bir bekleme
  //    bu sure boyunca seri okumayi ve YUKARIDAKI SURUS WATCHDOG'UNU
  //    durdururdu; arac 3 saniye boyunca son hiziyla kontrolsuz devam
  //    ederdi. millis() ile sayilan bu yapida surus dongusu hic aksamaz.
  if (!frenCalisiyor && frenIstenen != frenUygulanan) {
    frenUygulanan = frenIstenen;
    frenCalisiyor = true;
    frenBaslamaMs = millis();
    if (frenIstenen == 1) {           // FREN AC
      digitalWrite(FREN_IN1, FREN_AC_IN1);
      digitalWrite(FREN_IN2, FREN_AC_IN2);
    } else {                          // FREN KAPAT (ters yon)
      digitalWrite(FREN_IN1, FREN_AC_IN2);
      digitalWrite(FREN_IN2, FREN_AC_IN1);
    }
  }
  if (frenCalisiyor && (millis() - frenBaslamaMs >= FREN_CALISMA_MS)) {
    frenCalisiyor = false;
    digitalWrite(FREN_IN1, LOW);      // motoru SERBEST birak (bos)
    digitalWrite(FREN_IN2, LOW);
    // *** IKINCI SAVUNMA HATTI (2026-09-06, canli testte bulundu) ***
    // Hareket SIRASINDA gelen durum degisiklikleri BIRIKMEZ - hedef,
    // fiilen uygulanan duruma esitlenir. Yoksa (butonun kontak sicramasi
    // yuzunden yasandi) 3 saniye biter bitmez motor hemen TERS yone
    // kalkiyor ve lineer aktuator ileri-geri gidip hicbir yere varmiyordu.
    // Asil koruma yer istasyonunda (kontrol_paneli_node::FREN_KILIT_S);
    // bu, komut baska bir kaynaktan gelse bile gecerli olan yedegidir.
    frenIstenen = frenUygulanan;
  }
}
