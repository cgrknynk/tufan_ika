/*
 * TUFAN - Manuel Surus Joystick Okuyucu
 * ----------------------------------------------------------
 * Standart 2 eksenli analog joystick modulu (KY-023 tipi:
 * VRx, VRy, SW, +5V, GND) okuyup ham degerleri arayuz
 * Jetson'una seri port uzerinden gonderir.
 *
 * BU ARDUINO MOTORLARA DOGRUDAN BAGLI DEGILDIR - sadece
 * joystick okuyup veri gonderir. PWM donusumu, guvenlik
 * watchdog'u ve /palet_hizlari yayini arayuz Jetson'undaki
 * surus_joystick_sistemi.py + telemetri_sistemi.py icinde yapilir.
 *
 * Kablolama (gercek baglanti - Arduino Mega):
 *   VRx -> A6
 *   VRy -> A7
 *   SW  -> D2   (INPUT_PULLUP, basili iken LOW, bagli degilse zararsiz)
 *
 * Protokol (satir basina, 115200 baud):
 *   "X,Y,BTN\n"   X,Y: 0-1023 ham ADC degeri, BTN: 0/1
 */

const int PIN_VRX = A6;
const int PIN_VRY = A7;
const int PIN_SW = 2;

const unsigned long GONDERIM_ARALIGI_MS = 20;  // ~50 Hz
unsigned long son_gonderim = 0;

void setup() {
  Serial.begin(115200);
  pinMode(PIN_SW, INPUT_PULLUP);
}

void loop() {
  unsigned long simdi = millis();
  if (simdi - son_gonderim < GONDERIM_ARALIGI_MS) return;
  son_gonderim = simdi;

  int x = analogRead(PIN_VRX);
  int y = analogRead(PIN_VRY);
  int btn = (digitalRead(PIN_SW) == LOW) ? 1 : 0;

  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.print(",");
  Serial.println(btn);
}
