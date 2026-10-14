// Control.cpp — задача управления 1 кГц (ось Z) / 100 Гц (дозатор), аварии.
#include "Control.h"
#include "Config.h"
#include "HeadAxis.h"
#include "Dispenser.h"
#include <math.h>

static_assert(configTICK_RATE_HZ == 1000, "Задача управления рассчитана на тик FreeRTOS 1 мс");

SemaphoreHandle_t gLock = nullptr;
uint32_t gFaults = 0;
uint32_t gEvents = 0;
volatile bool gFlashBusy = false;
Snapshot gSnap;

static TaskHandle_t controlTask = nullptr;

void controlInit() {
  gLock = xSemaphoreCreateMutex();
  memset(&gSnap, 0, sizeof(gSnap));
}

static void snapshotUpdate(uint32_t now) {
  Snapshot &s = gSnap;
  s.ms = now;
  s.faults = gFaults;
  s.zMode = H.mode;
  s.zHomed = H.homed;
  s.zPos = headPosMm();
  s.zTarget = H.targetMm;
  s.zVel = H.vCmd;
  s.distRaw = H.distRaw;
  s.distEst = H.filtInit ? headDistEst() : NAN;
  s.distVel = H.vEst;
  s.distFresh = headSensorFresh(now);
  s.usValid = H.nValid;
  s.usInvalid = H.nInvalid;
  s.usReject = H.nReject;
  s.dRef = D.ref;
  s.dTarget = D.target;
  s.enc1 = D.enc[0];
  s.enc2 = D.enc[1];
  s.vRef = D.vref;
  s.v1 = D.vel[0];
  s.v2 = D.vel[1];
  s.u1 = D.u[0];
  s.u2 = D.u[1];
  s.i1 = D.integ[0];
  s.i2 = D.integ[1];
  s.dMoving = D.moving;
  s.dRelief = D.relief;
  s.dEnabled = D.enabled;
  s.dExt = D.ext;
}

// Контур управления работает в отдельной задаче с приоритетом выше loop():
// приём команд, Bluetooth и вывод в порт больше не останавливают регуляторы.
static void controlTaskFn(void *) {
  TickType_t last = xTaskGetTickCount();
  uint32_t dispDiv = 0;
  zWdtArmed = true;
  for (;;) {
    vTaskDelayUntil(&last, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
    const uint32_t now = millis();
    xSemaphoreTake(gLock, portMAX_DELAY);
    zHeartbeat = zHeartbeat + 1;
    if (zWdtTripped && !(gFaults & F_WDT)) faultRaise(F_WDT);
    headControlStep(now);
    if (++dispDiv >= DISP_PERIOD_MS) {
      dispDiv = 0;
      dispControlStep(now);
    }
    snapshotUpdate(now);
    xSemaphoreGive(gLock);
  }
}

void controlStart() {
  xTaskCreatePinnedToCore(controlTaskFn, "control", 4096, nullptr, CONTROL_TASK_PRIO, &controlTask, 1);
}

// Позиции пишутся в NVS только после PERSIST_IDLE_MS полного покоя обеих осей:
// запись во flash останавливает оба ядра на миллисекунды.
void persistencePoll() {
  static uint32_t lastCheck = 0, idleSince = 0;
  const uint32_t now = millis();
  if ((uint32_t)(now - lastCheck) < 100) return;
  lastCheck = now;
  bool idle, homed;
  int32_t zp, e1, e2;
  {
    Lock l;
    idle = headIsIdle() && H.mode != Z_TRACK && dispIsIdle();
    homed = H.homed;
    zp = headPosSteps();
    e1 = D.enc[0];
    e2 = D.enc[1];
  }
  if (!idle) {
    idleSince = now;
    return;
  }
  if ((uint32_t)(now - idleSince) < PERSIST_IDLE_MS) return;
  if (homed) positionsSaveZIfChanged(zp);
  positionsSaveDIfChanged(e1, e2);
}

void faultRaise(uint32_t f) {
  const uint32_t added = f & ~gFaults;
  if (!added) return;
  gFaults |= added;
  gEvents |= added << EV_FAULT_SHIFT;
  headOnFault(added);
  dispOnFault(added);
}

void faultClear() {
  gFaults = 0;
  zWdtTripped = false;
  headFaultClear();
  dispFaultClear();
}

const char *faultNames(uint32_t f, char *buf, size_t n) {
  if (!f) {
    snprintf(buf, n, "none");
    return buf;
  }
  snprintf(buf, n, "%s%s%s%s%s", (f & F_SENSOR) ? "SENSOR " : "", (f & F_JAM) ? "JAM " : "",
           (f & F_SKEW) ? "SKEW " : "", (f & F_ESTOP) ? "STOP " : "", (f & F_WDT) ? "WDT " : "");
  const size_t len = strlen(buf);
  if (len && buf[len - 1] == ' ') buf[len - 1] = 0;
  return buf;
}
