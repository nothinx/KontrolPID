// Uji logika KontrolPID di PC:
//   g++ -std=c++11 -Wall -Wextra -I. -I../../src uji.cpp ../../src/KontrolPID.cpp -o uji && ./uji
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "KontrolPID.h"
#include "pembanding.h"

uint32_t mikroPalsu = 1000;

static bool dekat(float a, float b, float tol = 1e-3f) { return fabsf(a - b) <= tol; }

// Jalankan pemanas 120 detik (dt 0.1 s), kembalikan overshoot (derajat di atas target).
template <class C> static float overshoot(C &c, float *akhir) {
  Pemanas p;
  float maks = 0;
  for (int k = 0; k < 1200; k++) {
    float u = c.hitung(100, p.y, 0.1f);
    assert(u >= 0 && u <= 255);
    p.langkah(u, 0.1f);
    if (p.y > maks) maks = p.y;
  }
  *akhir = p.y;
  return maks - 100;
}

int main() {
  int kasus = 0;
  { // pemanas: konvergen, keluaran tak pernah lewat batas, anti-windup vs tanpa
    KontrolPID pid(4, 2, 0);
    assert(pid.aturBatas(0, 255));
    float akhirKami, akhirNaif, akhirV1;
    float osKami = overshoot(pid, &akhirKami);
    PIDNaif naif{4, 2, 0, 0, 0};
    float osNaif = overshoot(naif, &akhirNaif);
    PIDv1 v1{4, 2, 0, 0, 0};
    float osV1 = overshoot(v1, &akhirV1);
    printf("Pemanas  : overshoot KontrolPID %.1f, PID_v1 %.1f, tanpa anti-windup %.1f derajat\n", osKami, osV1, osNaif);
    assert(dekat(akhirKami, 100, 0.1f));
    assert(osKami < 3.5f && osKami < osV1 && osV1 < osNaif);
    kasus++;
  }
  { // integrator (posisi motor): konvergen, batas ±100 dipatuhi, anti-windup vs tanpa
    float osPID = 0, osNaif = 0;
    for (int mode = 0; mode < 2; mode++) {
      KontrolPID pid(2, 0.5f, 0.3f);
      pid.aturBatas(-100, 100);
      float y = 0, maks = 0, s = 0;
      for (int k = 0; k < 3000; k++) {
        float u;
        if (mode == 0) u = pid.hitung(500, y, 0.01f);
        else { // tanpa anti-windup
          float e = 500 - y;
          s += 0.5f * e * 0.01f;
          u = 2 * e + s;
          u = u > 100 ? 100 : (u < -100 ? -100 : u);
        }
        assert(u >= -100 && u <= 100);
        y += u * 0.01f;
        if (y > maks) maks = y;
      }
      if (mode == 0) { osPID = maks - 500; assert(dekat(y, 500, 0.5f)); }
      else osNaif = maks - 500;
    }
    printf("Integrator: overshoot KontrolPID %.1f, tanpa anti-windup %.1f\n", osPID, osNaif);
    assert(osPID < osNaif);
    kasus++;
  }
  { // dt nol, negatif, NaN: integral & D tidak berubah, keluaran tetap terbatas
    KontrolPID pid(1, 1, 1);
    pid.aturBatas(-10, 10);
    pid.hitung(5, 0, 0.1f);
    pid.hitung(5, 1, 0.1f);
    float i = pid.i(), d = pid.d();
    pid.hitung(5, 3, 0);
    pid.hitung(5, 3, -1);
    pid.hitung(5, 3, NAN);
    assert(pid.i() == i && pid.d() == d);
    assert(dekat(pid.keluaran(), 2 + i + d));
    pid.hitung(1e30f, 0, 0.1f);
    assert(pid.keluaran() == 10);
    kasus++;
  }
  { // derivative kick: target melompat, nilai tetap -> D tetap nol
    KontrolPID pid(1, 0, 5);
    pid.hitung(0, 20, 0.01f);
    pid.hitung(100, 20, 0.01f);
    assert(pid.d() == 0 && dekat(pid.keluaran(), 80));
    pid.hitung(100, 21, 0.01f); // nilai naik 1 dalam 0.01 s -> D = -5 * 100
    assert(dekat(pid.d(), -500));
    kasus++;
  }
  { // ganti tuning tanpa lonjakan integral
    KontrolPID pid(2, 1, 0);
    for (int k = 0; k < 10; k++) pid.hitung(10, 4, 0.1f);
    float i = pid.i(), u = pid.keluaran();
    pid.aturTuning(2, 5, 0);
    assert(dekat(pid.hitung(10, 4, 0), u) && pid.i() == i);
    assert(pid.kp() == 2 && pid.ki() == 5 && pid.kd() == 0);
    kasus++;
  }
  { // reset(keluaran): perpindahan mulus dari manual
    KontrolPID pid(3, 1, 2);
    pid.aturBatas(0, 255);
    pid.reset(120);
    assert(pid.keluaran() == 120);
    assert(dekat(pid.hitung(50, 50), 120)); // error 0, tanpa D di panggilan pertama
    pid.reset(999);
    assert(pid.i() == 255);
    kasus++;
  }
  { // arah terbalik: nilai di atas target -> keluaran positif (pendingin)
    KontrolPID pid(2, 0, 1);
    pid.aturTerbalik(true);
    assert(pid.hitung(25, 30, 0.1f) == 10);
    pid.hitung(25, 31, 0.1f); // nilai naik -> D menambah keluaran
    assert(pid.d() > 0);
    kasus++;
  }
  { // filter D meredam lonjakan derau
    KontrolPID kasar(0, 0, 1), halus(0, 0, 1);
    halus.aturFilterD(0.1f);
    kasar.hitung(0, 0, 0.01f);
    halus.hitung(0, 0, 0.01f);
    kasar.hitung(0, 1, 0.01f);
    halus.hitung(0, 1, 0.01f);
    assert(dekat(kasar.d(), -100) && halus.d() > -10 && halus.d() < 0);
    kasus++;
  }
  { // dt dari micros(), termasuk saat micros() meluap
    mikroPalsu = 0xFFFFFFFFu - 50000;
    KontrolPID pid(0, 1, 0);
    pid.hitung(10, 0); // panggilan pertama: dt = 0
    assert(pid.i() == 0);
    mikroPalsu += 100000; // 0.1 s, melewati luapan
    pid.hitung(10, 0);
    assert(dekat(pid.i(), 1));
    kasus++;
  }
  { // batas tidak sah ditolak
    KontrolPID pid(1, 0, 0);
    assert(!pid.aturBatas(5, 5) && !pid.aturBatas(10, -10) && !pid.aturBatas(NAN, 1));
    assert(pid.hitung(1e6f, 0, 0.1f) == 1e6f); // default tanpa batas
    kasus++;
  }
  printf("Semua uji lolos (%d kasus, sizeof = %u byte)\n", kasus, (unsigned)sizeof(KontrolPID));
  return 0;
}
