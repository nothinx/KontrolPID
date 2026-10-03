// Belajar PID tanpa hardware: motor disimulasikan di dalam board.
// Buka Tools -> Serial Plotter (115200) untuk melihat target, kecepatan,
// dan PWM. Target berganti tiap 5 detik.
//
// Coba ubah KP, KI, KD di bawah lalu upload ulang:
// - KI = 0: kecepatan berhenti di bawah target (ada error tetap).
// - KP besar: cepat naik, tapi mulai berosilasi.
// - KD: meredam osilasi.
#include <KontrolPID.h>

const float KP = 0.8, KI = 2.0, KD = 0.01;
KontrolPID pid(KP, KI, KD);

// Motor tiruan: kecepatan (RPM) mendekati 1,2 x PWM dengan konstanta waktu
// 0,3 detik. Di bawah PWM 20 motor diam karena gesekan.
float kecepatan = 0;

void simulasiMotor(float pwm, float dt) {
  float tujuan = fabs(pwm) < 20 ? 0 : 1.2 * pwm;
  kecepatan += (tujuan - kecepatan) / 0.3 * dt;
}

void setup() {
  Serial.begin(115200);
  pid.aturBatas(-255, 255);
}

void loop() {
  static uint32_t terakhir = 0;
  if (millis() - terakhir < 20) return; // hitung tiap 20 ms (50 kali per detik)
  terakhir += 20;

  float target = (millis() / 5000) % 2 ? 200 : 100;
  float pwm = pid.hitung(target, kecepatan);
  simulasiMotor(pwm, 0.02);

  Serial.print("target:");
  Serial.print(target);
  Serial.print(",kecepatan:");
  Serial.print(kecepatan);
  Serial.print(",pwm:");
  Serial.println(pwm);
}
