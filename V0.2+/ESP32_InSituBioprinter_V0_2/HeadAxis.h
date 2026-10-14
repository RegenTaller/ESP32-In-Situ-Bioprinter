// HeadAxis.h — главный привод (ось Z головки) и ультразвуковой датчик дистанции.
// Позиция: 0 — верхнее (отведённое) положение, "+" — вниз, к поверхности.
#pragma once
#include <Arduino.h>
#include "Settings.h"

enum ZMode : uint8_t { Z_IDLE = 0, Z_TRACK, Z_MOVE, Z_RETRACT, Z_HOMING, Z_STOPPED };

struct HeadState {
  ZMode mode;
  bool homed;         // позиция известна (хоминг, zset или восстановлена из NVS)
  bool homedFromNvs;
  bool unhomedJog;    // ручное перемещение без известной позиции (пределы отключены)
  bool holding;       // в зоне нечувствительности
  bool satLow;        // слежение упёрлось в верхний предел (поверхность слишком близко)
  float moveTargetMm; // цель для zgo / zjog
  float targetMm;     // текущая цель контура
  float vCmd;         // мм/с, заданная скорость (с ограничением ускорения)

  // α-β фильтр дистанции
  float distRaw;      // мм, последний сырой отсчёт
  float xEst, vEst;   // мм, мм/с
  bool filtInit, hasPending;
  float pending;      // отсчёт после скачка, ждёт подтверждения
  int64_t tSampleUs;
  uint32_t lastValidMs, nValid, nInvalid, nReject;

  // хоминг об упор
  int32_t homeStartSteps, homeTravelSteps;
};

extern HeadState H;
extern volatile uint32_t zHeartbeat;
extern volatile bool zWdtArmed;
extern volatile bool zWdtTripped;

void headBegin(const SavedPositions &p);
void headControlStep(uint32_t nowMs);  // 1 кГц, под gLock
void headOnFault(uint32_t f);
void headFaultClear();

float headStepsPerMm();
int32_t headPosSteps();
float headPosMm();
float headDistEst();                   // фильтрованная дистанция с экстраполяцией
bool headSensorFresh(uint32_t nowMs);
bool headIsIdle();

void headApplyDriverSettings();        // ток и stealthChop в TMC2130 (SPI)
void headUpdateLimits();
void headUpdateDir();
void headSetTrigPeriod();

// Команды. Возвращают nullptr при успехе или текст ошибки. Вызывать под gLock.
const char *headCmdTrack(bool on, uint32_t nowMs);
const char *headCmdMove(float targetMm);
const char *headCmdJog(float deltaMm);
const char *headCmdRetract();
const char *headCmdHomeHard();
const char *headCmdSetPos(float mm);
const char *zModeName(uint8_t m);
