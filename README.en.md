# KontrolPID (English)

[Bahasa Indonesia](README.md)

An Arduino **PID controller** for motor speed, temperature, line followers, and robot turning. One call, `pid.hitung(target, value)`, with no pointers and no `SetMode()`. The API and examples are in Indonesian. This page maps every function to English.

```cpp
#include <KontrolPID.h>

KontrolPID pid(2.0, 0.5, 0.1); // kp, ki, kd

void setup() {
  pid.aturBatas(0, 255);       // setLimits(): PWM range
}

void loop() {
  static uint32_t last = 0;
  if (millis() - last < 20) return;  // every 20 ms
  last += 20;

  float pwm = pid.hitung(500, analogRead(A0)); // compute(setpoint, input)
  analogWrite(9, (int)pwm);
}
```

## Why

Checked against the source of PID_v1 1.2.1 (br3ttb), QuickPID 3.1.9, and FastPID:

| | KontrolPID | PID_v1 | QuickPID | FastPID |
|---|---|---|---|---|
| Works right after construction | ✅ | needs `SetMode(AUTOMATIC)` | needs `SetMode()` | ✅ |
| Negative output by default | ✅ | default 0–255 | default 0–255 | default 0–32767 |
| I and D use the real elapsed time | ✅ | fixed sample time | fixed sample time | fixed rate |
| Derivative filter | ✅ | ❌ | ❌ | ❌ |
| Data type | `float` | `double` via pointers | `float` via pointers | `int16_t` |

- Real `dt` from `micros()`, or pass your own `dt` in seconds. Zero, negative, or NaN `dt` leaves the integral and D untouched.
- Anti-windup by conditional integration plus clamping. Measured overshoot on a first-order heater (PWM 0–255, kp = 4, ki = 2): **3.0** vs 5.8 for the PID_v1 algorithm and 22.5 with no anti-windup.
- Derivative on measurement: no kick when the setpoint jumps.
- Change tunings on the fly without an integral bump. Bumpless manual-to-auto with `reset(lastOutput)`.
- P, I, and D terms are readable for the Serial Plotter. 50 bytes of RAM per controller. Safe across the `micros()` overflow.

## Function reference

| Indonesian | English | Notes |
|---|---|---|
| `KontrolPID(kp, ki, kd)` | constructor | |
| `hitung(target, nilai)` | compute(setpoint, input) | dt measured with `micros()`; the first call after construction or `reset()` is P only |
| `hitung(target, nilai, dt)` | compute(setpoint, input, dt) | dt in seconds |
| `aturBatas(min, maks)` | setOutputLimits | default unlimited; `false` if min >= max |
| `aturTuning(kp, ki, kd)` | setTunings | |
| `aturFilterD(detik)` | setDerivativeFilter(seconds) | low-pass time constant, 0 = off (default) |
| `aturTerbalik(bool)` | setReverse | for reverse-acting processes (coolers) |
| `reset(keluaran = 0)` | reset(output = 0) | clears integral and D history |
| `keluaran()` | output | last output |
| `p()`, `i()`, `d()` | P, I, D terms | |
| `kp()`, `ki()`, `kd()` | current tunings | |

## Examples

`SimulasiTanpaHardware` (simulated motor, no hardware needed), `TuningLewatSerial` (tune from the Serial Monitor), `KontrolKecepatanMotor` (motor RPM with encoder), `KontrolSuhu` (heater with NTC thermistor), `LineFollowerPID` (5-sensor line follower).

## Status

Version 1.0.0 passes automated logic tests (including plant simulations) and compiles on Uno, Mega, ESP32, ESP32-C3, ESP32-S3, STM32 Blackpill F411, and Bluepill F103. It has **not yet been tested on real hardware**.

## License

MIT © 2026 Amadeo Wisesa.
