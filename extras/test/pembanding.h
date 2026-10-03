// Plant & algoritma pembanding, dipakai uji.cpp dan ../simulasi/simulasi.cpp.
#pragma once

// Pemanas orde-1: suhu naik menuju 0.5 * PWM dengan konstanta waktu 5 detik.
// Batas PWM 0..255, jadi suhu maksimum 127.5. Target 100.
struct Pemanas {
  float y = 0;
  void langkah(float u, float dt) { y += (0.5f * u - y) / 5.0f * dt; }
};

// PID naif tanpa anti-windup: integral bebas, hanya keluaran yang dibatasi.
struct PIDNaif {
  float kp, ki, kd, s, lalu;
  float hitung(float t, float n, float dt) {
    float e = t - n;
    s += ki * e * dt;
    float u = kp * e + s - kd * (n - lalu) / dt;
    lalu = n;
    return u < 0 ? 0 : (u > 255 ? 255 : u);
  }
};

// Algoritma PID_v1 (br3ttb) 1.2.1: integral dibatasi ke batas keluaran.
struct PIDv1 {
  float kp, ki, kd, s, lalu;
  float hitung(float t, float n, float dt) {
    float e = t - n;
    s += ki * dt * e;
    s = s > 255 ? 255 : (s < 0 ? 0 : s);
    float u = kp * e + s - kd / dt * (n - lalu);
    lalu = n;
    return u < 0 ? 0 : (u > 255 ? 255 : u);
  }
};
