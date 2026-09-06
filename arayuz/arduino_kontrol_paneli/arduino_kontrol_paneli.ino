/*
 * TUFAN - Yer Istasyonu Fiziksel Kontrol Paneli Okuyucu
 * ----------------------------------------------------------
 * Arayuz monitoru (dokunmatik ekran) yer istasyonuna monte edildi;
 * yanina 2 joystick + 2 buton + 2 toggle switch + 1 potansiyometre
 * bagli. HEPSI BU TEK ARDUINO'ya bagli. Bu Arduino ham degerleri
 * arayuz Jetson'una seri port uzerinden gonderir.
 *
 * KART: Arduino UNO (orijinal). Kullanilan pinler A0,A1,A3,A4,A5 +
 * D4,D5,D9,D10 - hepsi UNO'da var (eski Mega sketch'i A6/A7 kullaniyordu,
 * bu kullanmiyor).
 *
 * BU ARDUINO MOTORLARA / SILAHA DOGRUDAN BAGLI DEGILDIR - sadece
 * paneli okuyup veri gonderir. PWM donusumu, tank karisimi, guvenlik
 * watchdog'u, /palet_hizlari + /turret_manuel_cmd + /silah_ates_manuel
 * + /farlar + /surus_modu + /arac_komut yayinlari arayuz Jetson'undaki
 * kontrol_paneli_sistemi.py + telemetri_sistemi.py icinde yapilir.
 *
 * KABLOLAMA:
 *   Sol joystick   -> A3, A4   (araci sur)
 *   Sag joystick   -> A0, A1   (silah/turret pan-tilt)
 *   Potansiyometre -> A5       (PWM ust siniri)
 *   Yer bataryasi  -> A2       (gerilim bolucu uzerinden, asagida)
 *   Sol toggle sw  -> D4       (GND'ye alininca ACIL STOP)
 *   Sag toggle sw  -> D5       (GND'ye alininca SILAH ATES)
 *   Sol buton      -> D9       (Manuel <-> Otonom)
 *   Sag buton      -> D10      (Farlar ac/kapa)
 *   Toggle/buton ortak ucu GND'ye; INPUT_PULLUP kullanildigi icin
 *   basili/GND'de = LOW(0), serbest = HIGH(1). Anlamlandirmayi
 *   (hangi eksen X, yon ters mi, kenar tespiti) Python tarafi yapar.
 *
 * YER BATARYASI OLCUMU (2026-09-05, kullanici istegi: "yer istasyonunu
 * besleyen 22.2V (6S) LiPo'nun sarjini yer sarj gostergesine cekelim"):
 * UNO'nun analog pinleri SADECE 0-5V okur, 6S LiPo dolu gerilimi (25.2V)
 * DOGRUDAN baglanirsa Arduino'yu YAKAR. GERILIM BOLUCU (iki direnc)
 * SARTTIR - once bolucu KURULMADAN bu pine hicbir sey baglama:
 *   Batarya (+) --[R1=100k]-- A2 --[R2=22k]-- GND
 *   Batarya (-) -> Arduino GND (ORTAK TOPRAK, cok onemli)
 * Bolme orani: R2/(R1+R2) = 22/122 = 0.1803 -> 25.2V*0.1803 = 4.54V
 * (5V sinirinin altinda, guvenlik payi var). Python tarafi (bkz.
 * kontrol_paneli_sistemi.py) AYNI R1/R2 degerleriyle gercek voltaja
 * geri cevirir - degerleri DEGISTIRIRSEN orayi da guncelle.
 * DIRENC DEGERLERI %1 toleranslı SECILIRSE dogruluk artar (adi %5
 * direnc de calisir, birkac yuzde hata olabilir - onemli degil, sarj
 * gostergesi zaten kaba bir tahmindir).
 *
 * PROTOKOL (satir basina, 115200 baud, ~50 Hz):
 *   "P,A3,A4,A0,A1,A5,A2,D4,D5,D9,D10\n"
 *   A*: 0-1023 ham ADC.  D*: 0/1 ham digitalRead.
 *   "P," oneki eski tek-joystick formatindan ("X,Y,BTN") ayirt icin.
 */

const int PIN_SOL_JOY_A = A3;
const int PIN_SOL_JOY_B = A4;
const int PIN_SAG_JOY_A = A0;
const int PIN_SAG_JOY_B = A1;
const int PIN_POT        = A5;
const int PIN_YER_BATARYA = A2;

const int PIN_SOL_TOGGLE = 4;
const int PIN_SAG_TOGGLE = 5;
const int PIN_SOL_BUTON   = 9;
const int PIN_SAG_BUTON   = 10;

const unsigned long GONDERIM_ARALIGI_MS = 20;  // ~50 Hz
unsigned long son_gonderim = 0;

void setup() {
  Serial.begin(115200);
  pinMode(PIN_SOL_TOGGLE, INPUT_PULLUP);
  pinMode(PIN_SAG_TOGGLE, INPUT_PULLUP);
  pinMode(PIN_SOL_BUTON, INPUT_PULLUP);
  pinMode(PIN_SAG_BUTON, INPUT_PULLUP);
}

void loop() {
  unsigned long simdi = millis();
  if (simdi - son_gonderim < GONDERIM_ARALIGI_MS) return;
  son_gonderim = simdi;

  int sol_a = analogRead(PIN_SOL_JOY_A);
  int sol_b = analogRead(PIN_SOL_JOY_B);
  int sag_a = analogRead(PIN_SAG_JOY_A);
  int sag_b = analogRead(PIN_SAG_JOY_B);
  int pot   = analogRead(PIN_POT);
  int yer_bat = analogRead(PIN_YER_BATARYA);

  int d_sol_toggle = digitalRead(PIN_SOL_TOGGLE);
  int d_sag_toggle = digitalRead(PIN_SAG_TOGGLE);
  int d_sol_buton  = digitalRead(PIN_SOL_BUTON);
  int d_sag_buton  = digitalRead(PIN_SAG_BUTON);

  Serial.print("P,");
  Serial.print(sol_a);   Serial.print(",");
  Serial.print(sol_b);   Serial.print(",");
  Serial.print(sag_a);   Serial.print(",");
  Serial.print(sag_b);   Serial.print(",");
  Serial.print(pot);     Serial.print(",");
  Serial.print(yer_bat); Serial.print(",");
  Serial.print(d_sol_toggle); Serial.print(",");
  Serial.print(d_sag_toggle); Serial.print(",");
  Serial.print(d_sol_buton);  Serial.print(",");
  Serial.println(d_sag_buton);
}
