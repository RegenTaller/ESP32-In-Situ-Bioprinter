#pragma once
#include "Arduino.h"
class BluetoothSerial : public Stream {
 public:
  bool begin(const char *) { return true; }
  bool hasClient() { return false; }
  int available() override { return 0; }
  int read() override { return -1; }
  size_t write(uint8_t) override { return 1; }
};
