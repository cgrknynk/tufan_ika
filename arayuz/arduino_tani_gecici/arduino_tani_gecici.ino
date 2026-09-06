/* GECICI TANI - tum analog (A0-A5) + digital (D2-D13) pinleri basar.
   Kontrol paneli kablolamasini dogrulamak icin. Sonra asil sketch
   (arduino_kontrol_paneli) geri yuklenecek. */
void setup() {
  Serial.begin(115200);
  for (int p = 2; p <= 13; p++) pinMode(p, INPUT_PULLUP);
}
void loop() {
  Serial.print("A ");
  for (int a = 0; a < 6; a++) { Serial.print(analogRead(A0 + a)); Serial.print(a < 5 ? "," : "  "); }
  Serial.print("D ");
  for (int d = 2; d <= 13; d++) { Serial.print(digitalRead(d)); Serial.print(d < 13 ? "," : ""); }
  Serial.println();
  delay(40);
}
