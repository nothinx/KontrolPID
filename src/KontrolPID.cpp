#include "KontrolPID.h"

static float batasi(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

float KontrolPID::hitung(float target, float nilai) {
  uint32_t sekarang = micros();
  float dt = _awal ? 0 : (uint32_t)(sekarang - _waktu) * 1e-6f; // aman saat micros() meluap
  _waktu = sekarang;
  return hitung(target, nilai, dt);
}

float KontrolPID::hitung(float target, float nilai, float dt) {
  float e = target - nilai;
  if (_terbalik) e = -e;
  _p = _kp * e;
  if (_awal) {
    _nilaiLalu = nilai;
    _awal = false;
  }
  if (dt > 0) { // juga menolak NaN
    if (dt != _dt) { // dt tetap (timer sendiri): tanpa pembagian setelah panggilan pertama
      _dt = dt;
      _kiDt = _ki * dt;
      _kdPerDt = _kd / dt;
      _alfa = _filter > 0 ? dt / (_filter + dt) : 1;
    }
    // D dari perubahan nilai: sama dengan perubahan error selama target tetap.
    float turun = _nilaiLalu - nilai;
    if (_terbalik) turun = -turun;
    float dMentah = _kdPerDt * turun;
    _d = _filter > 0 ? _d + (dMentah - _d) * _alfa : dMentah;
    _nilaiLalu = nilai;
    // Anti-windup (conditional integration): integral tidak ditambah jika
    // keluaran sudah mentok dan tambahan itu mendorongnya makin jauh.
    float tambah = _kiDt * e;
    float u = _p + _d + _i;
    if (!((u >= _maks && tambah > 0) || (u <= _min && tambah < 0)))
      _i = batasi(_i + tambah, _min, _maks);
  }
  _keluaran = batasi(_p + _d + _i, _min, _maks);
  return _keluaran;
}

bool KontrolPID::aturBatas(float min, float maks) {
  if (!(min < maks)) return false;
  _min = min;
  _maks = maks;
  _i = batasi(_i, min, maks);
  _keluaran = batasi(_keluaran, min, maks);
  return true;
}

void KontrolPID::aturTuning(float kp, float ki, float kd) {
  // Integral disimpan sebagai nilai (bukan jumlah error), jadi ganti ki
  // tidak membuat keluaran melonjak.
  _kp = kp;
  _ki = ki;
  _kd = kd;
  _dt = 0;
}

void KontrolPID::reset(float keluaran) {
  _i = batasi(keluaran, _min, _maks);
  _p = _d = 0;
  _keluaran = _i;
  _awal = true;
}
