// Arduino.h tiruan untuk menguji logika KontrolPID di PC.
#pragma once
#include <stdint.h>
extern uint32_t mikroPalsu;
inline uint32_t micros() { return mikroPalsu; }
