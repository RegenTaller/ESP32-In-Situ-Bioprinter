// Dispenser.h — дозатор: два ДПТ с энкодерами на одном толкателе штока шприца.
#pragma once
#include <Arduino.h>
#include "Settings.h"

struct DispState {
  double ref;          // тик, опорная позиция (дробная — без систематического недолива)
  float vref;          // тик/с, опорная скорость планировщика
  int32_t target;      // тик, цель перемещения
  bool moving;         // планировщик ведёт ref к target
  bool relief;         // идёт отвод после заклинивания
  bool enabled;        // false — моторы на тормозе (stop / перекос / сторожевой таймер)
  bool lagActive;      // опора упирается в предел ошибки слежения
  bool holdErrActive;  // в покое мотор не может дойти до опоры
  int8_t ext;          // непрерывная экструзия: +1 / -1 / 0
  int8_t clampDir;
  uint32_t lagSinceMs, holdErrSinceMs;
  int32_t enc[2];      // тик
  float vel[2];        // тик/с, измеренная скорость
  float integ[2];      // интегральная составляющая, ШИМ
  int16_t u[2];        // выход, ШИМ со знаком (0 — тормоз)
};

extern DispState D;

void dispBegin(const SavedPositions &p);
void dispControlStep(uint32_t nowMs);  // каждые DISP_PERIOD_MS, под gLock
void dispOnFault(uint32_t f);
void dispFaultClear();

// Команды (под gLock). Возвращают nullptr или текст ошибки.
const char *dispCmdTarget(int64_t ticks);
const char *dispCmdExt(int dir);
const char *dispCmdZero();
void dispStopSmooth();

bool dispIsIdle();
int32_t dispLimitTicks(bool upper);
