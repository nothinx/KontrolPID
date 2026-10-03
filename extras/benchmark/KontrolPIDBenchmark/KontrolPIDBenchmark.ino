// Benchmark KontrolPID di ATmega328P 16 MHz (simavr). Cara menjalankan dan
// angka hasilnya: README bagian "Kecepatan & memori".
// Siklus.h: Timer1 tanpa prescaler, UKUR(nama, ulang, kode) mencetak
// "BENCH nama siklus_per_panggilan".
#include <KontrolPID.h>
#include "Siklus.h"

// Nilai sensor berganti tiap panggilan (8 nilai dekat target) agar keluaran tidak mentok.
static const float NILAI[8] = {95.0f, 97.5f, 99.0f, 100.5f, 101.0f, 99.5f, 98.0f, 96.5f};
volatile float target = 100, keluaran;
KontrolPID pid(2, 0.5f, 0.1f), pidFilter(2, 0.5f, 0.1f);

void setup() {
  Serial.begin(115200);
  pid.aturBatas(0, 255);
  pidFilter.aturBatas(0, 255);
  pidFilter.aturFilterD(0.05f);
  Serial.print(F("BENCH sizeof "));
  Serial.println(sizeof(KontrolPID));
  UKUR("kosong", 1000, keluaran = NILAI[_i & 7]);
  UKUR("hitung_dt", 1000, keluaran = pid.hitung(target, NILAI[_i & 7], 0.01f));
  UKUR("hitung_dt_filterD", 1000, keluaran = pidFilter.hitung(target, NILAI[_i & 7], 0.01f));
  UKUR("hitung_micros", 1000, keluaran = pid.hitung(target, NILAI[_i & 7]));
  selesai();
}

void loop() {}
