// KontrolPID - kontrol PID untuk motor, suhu, line follower, dan robot.
// Copyright (c) 2026 Amadeo Wisesa. Lisensi MIT.
//
// - dt nyata dari micros(): loop() yang tidak rata tetap dihitung benar.
// - Anti-windup: integral berhenti menumpuk saat keluaran mentok di batas.
// - D dihitung dari perubahan nilai (bukan error): ganti target tidak
//   membuat keluaran melonjak (derivative kick).
// - Ganti kp/ki/kd di tengah jalan tanpa lonjakan dari integral.
#pragma once
#include <Arduino.h>
#include <math.h>

class KontrolPID {
public:
  KontrolPID(float kp, float ki, float kd) : _kp(kp), _ki(ki), _kd(kd) {}

  // Panggil berkala di loop(). dt diukur sendiri dengan micros().
  // Panggilan pertama (dan pertama setelah reset()) hanya memakai P.
  float hitung(float target, float nilai);
  // Sama, dengan dt (detik) dari timer sendiri. dt <= 0: integral & D tetap.
  float hitung(float target, float nilai, float dt);

  // Batas keluaran, misalnya 0..255 (PWM) atau -255..255 (motor dua arah).
  // Default tanpa batas. false jika min >= maks.
  bool aturBatas(float min, float maks);
  void aturTuning(float kp, float ki, float kd);
  // Saring derau D dengan low-pass. Konstanta waktu dalam detik, 0 = mati (default).
  void aturFilterD(float detik) { _filter = detik; }
  // Untuk proses terbalik: keluaran naik membuat nilai turun (mis. pendingin).
  void aturTerbalik(bool terbalik) { _terbalik = terbalik; }
  // Hapus integral & riwayat D. Isi keluaran terakhir saat pindah dari
  // kendali manual agar perpindahannya mulus.
  void reset(float keluaran = 0);

  // --- Untuk debug & Serial Plotter ---
  float keluaran() const { return _keluaran; }
  float p() const { return _p; }
  float i() const { return _i; }
  float d() const { return _d; }
  float kp() const { return _kp; }
  float ki() const { return _ki; }
  float kd() const { return _kd; }

private:
  float _kp, _ki, _kd;
  float _min = -INFINITY, _maks = INFINITY;
  float _filter = 0;
  float _p = 0, _i = 0, _d = 0, _keluaran = 0, _nilaiLalu = 0;
  uint32_t _waktu = 0;   // micros() saat hitung() terakhir
  bool _awal = true;     // belum ada nilai sebelumnya untuk D
  bool _terbalik = false;
};
