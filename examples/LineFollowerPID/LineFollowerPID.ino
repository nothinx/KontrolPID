// Robot line follower dengan 5 sensor garis (TCRT5000 / modul 5 kanal) dan PID.
//
// Sambungan:
// - Sensor kiri ke kanan: A0, A1, A2, A3, A4 (keluaran analog).
// - Driver motor (L298N / TB6612): motor kiri PWM pin 5, arah pin 7 & 8;
//   motor kanan PWM pin 6, arah pin 9 & 10.
// - ESP32 DevKit: lihat pin di bawah (C3/S3: sesuaikan GPIO), dan naikkan AMBANG ke sekitar 2000 (ADC 12 bit).
// Garis hitam di lantai putih. Untuk garis putih, balik perbandingan AMBANG.
//
// Tuning: mulai dengan KI = KD = 0. Naikkan KP sampai robot bisa mengikuti
// garis sambil bergoyang, lalu naikkan KD sampai goyangan hilang.
// KI jarang perlu untuk line follower.
#include <KontrolPID.h>

#ifdef ESP32 // GPIO 6..11 dipakai flash, jangan disentuh
const uint8_t SENSOR[5] = {36, 39, 34, 35, 32};
const uint8_t KIRI_PWM = 25, KIRI_A = 26, KIRI_B = 27, KANAN_PWM = 14, KANAN_A = 12, KANAN_B = 13;
#else
const uint8_t SENSOR[5] = {A0, A1, A2, A3, A4};
const uint8_t KIRI_PWM = 5, KIRI_A = 7, KIRI_B = 8, KANAN_PWM = 6, KANAN_A = 9, KANAN_B = 10;
#endif
const int AMBANG = 500;          // di atas ini = melihat garis hitam
const int KECEPATAN_DASAR = 150; // PWM saat jalan lurus

KontrolPID pid(40, 0, 4);

// Posisi garis terhadap robot: -2 (paling kiri) .. 0 (tengah) .. 2 (paling kanan).
float posisiGaris() {
  static float terakhir = 0;
  int jumlah = 0;
  float total = 0;
  for (uint8_t i = 0; i < 5; i++) {
    if (analogRead(SENSOR[i]) > AMBANG) {
      total += i - 2;
      jumlah++;
    }
  }
  // Garis hilang: anggap garis masih di sisi tempat terakhir terlihat.
  if (jumlah == 0) return terakhir < 0 ? -2.5 : (terakhir > 0 ? 2.5 : 0);
  terakhir = total / jumlah;
  return terakhir;
}

void motor(uint8_t pinPwm, uint8_t pinA, uint8_t pinB, int kecepatan) {
  digitalWrite(pinA, kecepatan >= 0);
  digitalWrite(pinB, kecepatan < 0);
  analogWrite(pinPwm, min(abs(kecepatan), 255));
}

void setup() {
  const uint8_t pin[] = {KIRI_PWM, KIRI_A, KIRI_B, KANAN_PWM, KANAN_A, KANAN_B};
  for (uint8_t i = 0; i < 6; i++) pinMode(pin[i], OUTPUT);
  pid.aturBatas(-200, 200);
}

void loop() {
  static uint32_t terakhir = 0;
  if (millis() - terakhir < 5) return; // 200 kali per detik
  terakhir += 5;

  // Target: garis di tengah (posisi 0). Garis di kanan (posisi positif)
  // membuat koreksi negatif, sehingga motor kiri lebih cepat dan robot
  // berbelok ke kanan.
  float koreksi = pid.hitung(0, posisiGaris());
  motor(KIRI_PWM, KIRI_A, KIRI_B, KECEPATAN_DASAR - koreksi);
  motor(KANAN_PWM, KANAN_A, KANAN_B, KECEPATAN_DASAR + koreksi);
}
