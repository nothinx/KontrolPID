// Simulasi di PC yang menjalankan kode KontrolPID asli (../../src), untuk grafik README.
// Dipanggil oleh gambar.py. Keluaran: bagian "# nama" diikuti baris CSV.
#include <math.h>
#include <stdio.h>
#include "KontrolPID.h"
#include "pembanding.h"

uint32_t mikroPalsu = 0;

// 1. Pemanas, tiga algoritma, dt tetap 0.1 s (sama dengan extras/test/uji.cpp).
template <class C> static void pemanas(C &c) {
  Pemanas p;
  for (int k = 0; k < 600; k++) { // 60 detik
    float u = c.hitung(100, p.y, 0.1f);
    printf("%.1f,%.3f,%.2f\n", k * 0.1f, p.y, u);
    p.langkah(u, 0.1f);
  }
}

// Compute() PID_v1 1.2.1 (br3ttb, PID_v1.cpp) dengan P on error dan DIRECT:
// hanya menghitung jika sudah lewat SampleTime (default 100 ms), lalu memakai
// ki * SampleTime dan kd / SampleTime, bukan waktu yang benar-benar lewat.
struct PIDv1Asli {
  float kp, ki, kd;           // seperti SetTunings(): ki, kd sudah dikali/dibagi SampleTime
  unsigned long sampleTime;   // ms
  float s, lalu, keluaran;
  unsigned long waktuLalu;
  PIDv1Asli(float Kp, float Ki, float Kd, unsigned long st, unsigned long sekarang)
      : kp(Kp), ki(Ki * st / 1000.0f), kd(Kd / (st / 1000.0f)), sampleTime(st), s(0), lalu(0),
        keluaran(0), waktuLalu(sekarang - st) {}
  float hitung(float target, float nilai, unsigned long millis) {
    if (millis - waktuLalu >= sampleTime) {
      float e = target - nilai;
      float dInput = nilai - lalu;
      s += ki * e;
      s = s > 255 ? 255 : (s < 0 ? 0 : s);
      float u = kp * e + s - kd * dInput;
      keluaran = u > 255 ? 255 : (u < 0 ? 0 : u);
      lalu = nilai;
      waktuLalu = millis;
    }
    return keluaran;
  }
};

// 2. Loop tidak rata: hitung() dipanggil tiap 100..400 ms (acak, seed tetap).
// Pemanas disimulasikan per 1 ms, keluaran ditahan di antara panggilan.
// mode 0: KontrolPID tiap 100 ms tepat (acuan), 1: KontrolPID acak, 2: PID_v1 acak.
static void loopTidakRata(int mode) {
  uint32_t acak = 12345;
  KontrolPID pid(4, 2, 0);
  pid.aturBatas(0, 255);
  PIDv1Asli v1(4, 2, 0, 100, 0);
  Pemanas p;
  float u = 0;
  unsigned long berikut = 0;
  for (unsigned long ms = 0; ms <= 60000; ms++) {
    if (ms == berikut) {
      mikroPalsu = ms * 1000;
      u = mode == 2 ? v1.hitung(100, p.y, ms) : pid.hitung(100, p.y);
      if (mode == 0) berikut += 100;
      else {
        acak = acak * 1103515245u + 12345u; // LCG, sama di semua compiler
        berikut += 100 + (acak >> 16) % 301;
      }
    }
    if (ms % 100 == 0) printf("%.1f,%.3f\n", ms / 1000.0f, p.y);
    p.langkah(u, 0.001f);
  }
}

// 3. Derivative kick: kecepatan motor orde-1, target melompat 100 -> 140 rpm di t = 2 s.
// mode 0: KontrolPID (D dari perubahan nilai), 1: rumus buku teks (D dari error),
// selain D identik dengan KontrolPID (anti-windup sama).
static void kick(int mode) {
  const float kp = 1.5f, ki = 3, kd = 0.05f, dt = 0.01f;
  KontrolPID pid(kp, ki, kd);
  pid.aturBatas(-255, 255);
  float rpm = 0, i = 0, eLalu = 0;
  bool awal = true;
  for (int k = 0; k < 400; k++) {
    float t = k * dt, target = t < 2 ? 100 : 140, u, d;
    if (mode == 0) {
      u = pid.hitung(target, rpm, dt);
      d = pid.d();
    } else {
      float e = target - rpm;
      d = awal ? 0 : kd * (e - eLalu) / dt;
      eLalu = e;
      float tambah = awal ? 0 : ki * e * dt, uu = kp * e + i + d;
      awal = false;
      if (!((uu >= 255 && tambah > 0) || (uu <= -255 && tambah < 0))) i += tambah;
      u = kp * e + i + d;
      u = u > 255 ? 255 : (u < -255 ? -255 : u);
    }
    printf("%.2f,%.3f,%.3f,%.3f,%.1f\n", t, rpm, u, d, target);
    rpm += (1.0f * u - rpm) / 0.5f * dt; // motor: rpm menuju 1 x PWM, konstanta waktu 0.5 s
  }
}

int main() {
  KontrolPID kami(4, 2, 0);
  kami.aturBatas(0, 255);
  PIDv1 v1{4, 2, 0, 0, 0};
  PIDNaif naif{4, 2, 0, 0, 0};
  puts("# pemanas_kontrolpid");
  pemanas(kami);
  puts("# pemanas_pidv1");
  pemanas(v1);
  puts("# pemanas_naif");
  pemanas(naif);
  const char *nama[] = {"acuan", "kontrolpid", "pidv1"};
  for (int m = 0; m < 3; m++) {
    printf("# loop_%s\n", nama[m]);
    loopTidakRata(m);
  }
  puts("# kick_kontrolpid");
  kick(0);
  puts("# kick_bukuteks");
  kick(1);
  return 0;
}
