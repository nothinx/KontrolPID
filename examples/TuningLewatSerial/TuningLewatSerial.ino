// Tuning kp, ki, kd lewat Serial tanpa upload ulang, lalu lihat hasilnya
// di Serial Plotter (115200).
//
// Proses yang dikendalikan: rangkaian RC, proses orde-1 sungguhan yang murah.
//   pin 9 --[ 10k ]--+-- A0
//                    |
//                 [ 100uF ]
//                    |
//                   GND
// ESP32 DevKit: GPIO 25 ke resistor, GPIO 34 di titik tengah.
// Ganti pin dan pembacaan sesuai proses kamu (motor, suhu, dll).
//
// Perintah (ketik lalu Enter, satu per baris):
//   p 2.5   -> kp = 2.5
//   i 0.8   -> ki = 0.8
//   d 0.05  -> kd = 0.05
//   t 600   -> target = 600
//   r       -> reset integral
// Di Serial Plotter, ketik perintah di kotak kirim di bagian bawah.
#include <KontrolPID.h>

#ifdef ESP32 // ADC 12 bit: target bisa sampai 4095
const uint8_t PIN_KELUARAN = 25, PIN_NILAI = 34;
#else
const uint8_t PIN_KELUARAN = 9, PIN_NILAI = A0;
#endif

KontrolPID pid(1.0, 0.5, 0);
float target = 500;

void bacaPerintah() {
  if (!Serial.available()) return;
  char c = Serial.read();
  if (c == 'r') pid.reset();
  if (c != 'p' && c != 'i' && c != 'd' && c != 't') return;
  float v = Serial.parseFloat();
  if (c == 'p') pid.aturTuning(v, pid.ki(), pid.kd());
  if (c == 'i') pid.aturTuning(pid.kp(), v, pid.kd());
  if (c == 'd') pid.aturTuning(pid.kp(), pid.ki(), v);
  if (c == 't') target = v;
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(50); // parseFloat() tidak menunggu lama
  pinMode(PIN_KELUARAN, OUTPUT);
  pid.aturBatas(0, 255);
}

void loop() {
  bacaPerintah();

  static uint32_t terakhir = 0;
  if (millis() - terakhir < 20) return;
  terakhir += 20;

  float nilai = analogRead(PIN_NILAI);
  float keluaran = pid.hitung(target, nilai);
  analogWrite(PIN_KELUARAN, (int)keluaran);

  // Komponen P, I, D ikut dicetak agar terlihat bagian mana yang bekerja.
  Serial.print("target:");
  Serial.print(target);
  Serial.print(",nilai:");
  Serial.print(nilai);
  Serial.print(",keluaran:");
  Serial.print(keluaran);
  Serial.print(",P:");
  Serial.print(pid.p());
  Serial.print(",I:");
  Serial.print(pid.i());
  Serial.print(",D:");
  Serial.println(pid.d());
}
