// Control.h — задача управления, общая блокировка, аварии, снимок состояния.
#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

// Аварии (защёлкиваются до команды clear)
enum : uint32_t {
  F_SENSOR = 1u << 0,  // нет достоверной дистанции во время слежения
  F_JAM = 1u << 1,     // заклинивание / окклюзия дозатора
  F_SKEW = 1u << 2,    // перекос толкателя: моторы разошлись
  F_ESTOP = 1u << 3,   // команда stop
  F_WDT = 1u << 4,     // контур управления не выполнялся > 50 мс
};

// События для сообщений в порт (младшие биты), аварии — со сдвигом EV_FAULT_SHIFT
enum : uint32_t {
  EV_Z_DONE = 1u << 0,
  EV_D_DONE = 1u << 1,
  EV_HOME_DONE = 1u << 2,
  EV_REACQUIRE = 1u << 3,
  EV_Z_SATURATED = 1u << 4,
  EV_RETRACT_DONE = 1u << 5,
  EV_D_LIMIT = 1u << 6,
};
constexpr uint32_t EV_FAULT_SHIFT = 16;

extern SemaphoreHandle_t gLock;  // защищает всё состояние управления
extern uint32_t gFaults;         // под gLock
extern uint32_t gEvents;         // под gLock
extern volatile bool gFlashBusy; // запись во flash: сторожевой таймер в ISR приостановлен

// RAII-блокировка состояния. Внутри — только быстрые операции, без вывода в порт.
class Lock {
 public:
  Lock() { xSemaphoreTake(gLock, portMAX_DELAY); }
  ~Lock() { xSemaphoreGive(gLock); }
  Lock(const Lock &) = delete;
  Lock &operator=(const Lock &) = delete;
};

// Снимок состояния для телеметрии и status (обновляется контуром каждую 1 мс)
struct Snapshot {
  uint32_t ms;
  uint32_t faults;
  // ось Z
  uint8_t zMode;
  bool zHomed;
  float zPos, zTarget, zVel;
  float distRaw, distEst, distVel;
  bool distFresh;
  uint32_t usValid, usInvalid, usReject;
  // дозатор
  double dRef;
  int32_t dTarget, enc1, enc2;
  float vRef, v1, v2;
  int16_t u1, u2;
  float i1, i2;
  bool dMoving, dRelief, dEnabled;
  int8_t dExt;
};
extern Snapshot gSnap;

void controlInit();   // создать блокировку (до инициализации модулей)
void controlStart();  // запустить задачу управления
void persistencePoll();       // из loop(): запись позиций в NVS в покое
void faultRaise(uint32_t f);  // вызывать под gLock
void faultClear();            // вызывать под gLock
const char *faultNames(uint32_t f, char *buf, size_t n);
