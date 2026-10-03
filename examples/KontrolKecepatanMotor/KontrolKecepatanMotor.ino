// Menjaga kecepatan motor DC (RPM) tetap walau beban berubah, dengan encoder.
//
// Sambungan (driver L298N / TB6612):
// - PWM driver (ENA) ke pin 5, arah IN1 ke pin 7, IN2 ke pin 8.
// - Encoder kanal A ke pin 2 (pin interrupt), kanal B ke pin 4.
// - ESP32 DevKit: lihat pin di bawah. ESP32-C3/S3: sesuaikan nomor GPIO.
// Isi PULSA_PER_PUTARAN dengan pulsa kanal A per satu putaran poros keluar
// (pulsa encoder x rasio gearbox), lihat datasheet motor.
//
// Buka Serial Plotter (115200) untuk melihat target dan kecepatan.
#include <KontrolPID.h>

#ifdef ESP32 // GPIO 6..11 dipakai flash, jangan disentuh
const uint8_t PIN_PWM = 25, PIN_IN1 = 26, PIN_IN2 = 27, PIN_ENC_A = 18, PIN_ENC_B = 19;
#else
const uint8_t PIN_PWM = 5, PIN_IN1 = 7, PIN_IN2 = 8, PIN_ENC_A = 2, PIN_ENC_B = 4;
#endif
const float PULSA_PER_PUTARAN = 374; // contoh: encoder 11 pulsa x gearbox 34
const float TARGET_RPM = 120;        // negatif = mundur

KontrolPID pid(1.5, 8, 0);
volatile long pulsa = 0;

void hitungPulsa() {
  // Kanal B menentukan arah putaran.
  if (digitalRead(PIN_ENC_B)) pulsa = pulsa + 1;
  else pulsa = pulsa - 1;
}

void jalankanMotor(int pwm) {
  digitalWrite(PIN_IN1, pwm >= 0);
  digitalWrite(PIN_IN2, pwm < 0);
  analogWrite(PIN_PWM, abs(pwm));
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_PWM, OUTPUT);
  pinMode(PIN_IN1, OUTPUT);
  pinMode(PIN_IN2, OUTPUT);
  pinMode(PIN_ENC_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), hitungPulsa, RISING);
  pid.aturBatas(-255, 255);
}

void loop() {
  static uint32_t terakhir = 0;
  static long pulsaLalu = 0;
  if (millis() - terakhir < 50) return; // hitung tiap 50 ms
  terakhir += 50;

  noInterrupts(); // long 4 byte: salin utuh tanpa disela interrupt
  long p = pulsa;
  interrupts();

  float rpm = (p - pulsaLalu) / PULSA_PER_PUTARAN * 60.0 / 0.05;
  pulsaLalu = p;

  float pwm = pid.hitung(TARGET_RPM, rpm);
  jalankanMotor((int)pwm);

  Serial.print("target:");
  Serial.print(TARGET_RPM);
  Serial.print(",rpm:");
  Serial.println(rpm);
}
