// Заглушка драйвера: запоминает последние записанные настройки.
#pragma once
#include <cstdint>
class TMC2130Stepper {
 public:
  TMC2130Stepper(uint16_t, uint16_t, uint16_t, uint16_t) {}
  void begin() {}
  void microsteps(uint16_t m) { ms = m; }
  void interpolate(bool) {}
  void rms_current(uint16_t mA, float = 0.5f, float = 0.11f) { current = mA; }
  void stealthChop(bool v) { stealth = v; }
  uint16_t current = 0, ms = 0;
  bool stealth = false;
};
