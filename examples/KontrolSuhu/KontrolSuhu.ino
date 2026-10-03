// Menjaga suhu tetap dengan pemanas dan termistor NTC 10k.
// Cocok untuk inkubator, pengering, atau hotbed sederhana.
//
// Sambungan:
// - Pemanas lewat MOSFET logic-level (mis. IRLZ44N): gate ke pin 9 lewat
//   resistor 220 ohm, source ke GND, pemanas antara drain dan +12V.
// - NTC 10k dengan resistor 10k sebagai pembagi tegangan:
//   VCC --[ 10k ]--+-- A0
//                  |
//               [ NTC ]
//                  |
//                 GND
// - ESP32 DevKit: pemanas di GPIO 25, NTC di GPIO 34, VCC pembagi tegangan 3.3V.
//
// Suhu berubah lambat, jadi PID cukup dihitung tiap 0,5 detik.
#include <KontrolPID.h>

#ifdef ESP32
const uint8_t PIN_PEMANAS = 25, PIN_NTC = 34;
const float ADC_MAKS = 4095; // ADC 12 bit
#else
const uint8_t PIN_PEMANAS = 9, PIN_NTC = A0;
const float ADC_MAKS = 1023; // ADC 10 bit (Uno, Nano, STM32)
#endif
const float TARGET = 40.0; // derajat Celsius

KontrolPID pid(20, 0.5, 0);

float bacaSuhu() {
  // Rumus beta NTC: B = 3950, 10k pada 25 derajat.
  float adc = analogRead(PIN_NTC);
  adc = constrain(adc, 1.0f, ADC_MAKS - 1);
  float r = 10000.0 * adc / (ADC_MAKS - adc);
  return 1.0 / (log(r / 10000.0) / 3950.0 + 1.0 / 298.15) - 273.15;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_PEMANAS, OUTPUT);
  pid.aturBatas(0, 255); // pemanas hanya bisa memanaskan
}

void loop() {
  static uint32_t terakhir = 0;
  if (millis() - terakhir < 500) return;
  terakhir += 500;

  float suhu = bacaSuhu();
  // Pengaman: matikan pemanas jika sensor lepas atau suhu tidak masuk akal.
  if (suhu < -20 || suhu > 90) {
    analogWrite(PIN_PEMANAS, 0);
    pid.reset();
    Serial.println("Sensor bermasalah, pemanas dimatikan");
    return;
  }

  float pwm = pid.hitung(TARGET, suhu);
  analogWrite(PIN_PEMANAS, (int)pwm);

  Serial.print("target:");
  Serial.print(TARGET);
  Serial.print(",suhu:");
  Serial.print(suhu);
  Serial.print(",pwm/10:");
  Serial.println(pwm / 10); // dibagi 10 agar muat di grafik yang sama
}
