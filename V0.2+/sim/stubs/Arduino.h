// Заглушка Arduino/ESP32 API для сборки прошивки на ПК (только для симуляции).
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "freertos/FreeRTOS.h"

#define IRAM_ATTR
#define HIGH 1
#define LOW 0
#define INPUT 0x01
#define OUTPUT 0x03
#define INPUT_PULLUP 0x05
#define INPUT_PULLDOWN 0x09
#define CHANGE 3
#define digitalPinToInterrupt(p) (p)

uint32_t millis();
uint32_t micros();
void delay(uint32_t ms);
void pinMode(uint8_t pin, uint8_t mode);
void digitalWrite(uint8_t pin, uint8_t val);
int digitalRead(uint8_t pin);
bool ledcAttach(uint8_t pin, uint32_t freq, uint8_t resolution);
bool ledcWrite(uint8_t pin, uint32_t duty);
void attachInterrupt(uint8_t pin, void (*fn)(void), int mode);

struct hw_timer_t;
hw_timer_t *timerBegin(uint32_t frequency);
void timerAttachInterrupt(hw_timer_t *timer, void (*fn)(void));
void timerAlarm(hw_timer_t *timer, uint64_t alarm, bool autoreload, uint64_t count);

class Print {
 public:
  virtual ~Print() {}
  virtual size_t write(uint8_t c) = 0;
  size_t print(const char *s) {
    size_t n = 0;
    while (*s) n += write((uint8_t)*s++);
    return n;
  }
};

class Stream : public Print {
 public:
  virtual int available() = 0;
  virtual int read() = 0;
};

class SimSerial : public Stream {
 public:
  std::string rx, tx;
  void begin(unsigned long) {}
  void flush() {}
  void updateBaudRate(unsigned long b) { baud = b; }
  int available() override { return (int)rx.size(); }
  int read() override {
    if (rx.empty()) return -1;
    const int c = (uint8_t)rx[0];
    rx.erase(0, 1);
    return c;
  }
  size_t write(uint8_t c) override {
    tx.push_back((char)c);
    return 1;
  }
  unsigned long baud = 115200;
};
extern SimSerial Serial;

struct EspClass {
  void restart() {}
};
extern EspClass ESP;
