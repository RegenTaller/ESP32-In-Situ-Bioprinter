// HeadAxis.cpp — главный привод: генерация шагов, датчик HC-SR04, слежение за поверхностью.
#include "HeadAxis.h"
#include "Config.h"
#include "Control.h"
#include <SPI.h>
#include <TMC2130Stepper.h>
#include <math.h>
#include "esp_timer.h"
#include "soc/gpio_reg.h"
#include "soc/soc.h"

HeadState H;

// Оба TMC2130 сидят на общих EN/DIR/STEP/CS: запись регистров идёт в оба драйвера сразу.
static TMC2130Stepper driver(PIN_Z_EN, PIN_Z_DIR, PIN_Z_STEP, PIN_Z_CS);
static hw_timer_t *stepTimer = nullptr;
static portMUX_TYPE zMux = portMUX_INITIALIZER_UNLOCKED;

// ---- Общие с ISR переменные. В ISR только целочисленная арифметика (FPU в ISR недоступен) ----
static volatile int32_t isrPos = 0;         // позиция, микрошаги
static volatile uint32_t isrInc = 0;        // приращение фазового аккумулятора за тик
static volatile int8_t isrDir = 0;          // -1 / 0 / +1
static volatile int8_t isrCurDir = 0;       // направление, выставленное на пине DIR
static volatile bool isrDirPosHigh = true;  // уровень DIR для движения "+" (вниз)
static volatile bool isrLimOn = false;
static volatile int32_t isrLimMin = 0, isrLimMax = 0;
static volatile uint32_t isrTrigTicks = 500;

volatile uint32_t zHeartbeat = 0;
volatile bool zWdtArmed = false;
volatile bool zWdtTripped = false;

static volatile uint32_t echoRiseUs = 0, echoWidthUs = 0, echoSeq = 0;
static volatile bool echoHigh = false;
static uint32_t echoSeqSeen = 0;

#define GPIO_HI(p) REG_WRITE(GPIO_OUT_W1TS_REG, (1UL << (p)))
#define GPIO_LO(p) REG_WRITE(GPIO_OUT_W1TC_REG, (1UL << (p)))

static inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// Таймер 50 кГц: шаги (DDA с фазовым аккумулятором), импульс TRIG, сторожевой таймер контура.
static void IRAM_ATTR onStepTimer() {
  static uint32_t phase = 0, hbLast = 0, hbTicks = 0, trigCnt = 0;
  static bool stepHigh = false;

  if (stepHigh) {  // импульс STEP длится один тик (20 мкс)
    GPIO_LO(PIN_Z_STEP);
    stepHigh = false;
  }

  // Сторожевой таймер: контур управления обновляет zHeartbeat каждую 1 мс.
  if (zWdtArmed && !gFlashBusy) {
    const uint32_t hb = zHeartbeat;
    if (hb != hbLast) {
      hbLast = hb;
      hbTicks = 0;
    } else if (++hbTicks > CTRL_WDT_TICKS && !zWdtTripped) {
      zWdtTripped = true;
      // Тормоз обоих ДПТ дозатора: IN1 = IN2 = HIGH (short brake TB6612FNG)
      REG_WRITE(GPIO_OUT_W1TS_REG, (1UL << PIN_D1_IN1) | (1UL << PIN_D1_IN2) |
                                       (1UL << PIN_D2_IN1) | (1UL << PIN_D2_IN2));
    }
  } else {
    hbTicks = 0;
    hbLast = zHeartbeat;
  }

  const int8_t d = isrDir;
  const uint32_t inc = isrInc;
  if (!zWdtTripped && d != 0 && inc != 0) {
    if (d != isrCurDir) {  // смена направления: выставить DIR, шаги — со следующего тика
      if ((d > 0) == isrDirPosHigh) GPIO_HI(PIN_Z_DIR);
      else GPIO_LO(PIN_Z_DIR);
      isrCurDir = d;
    } else {
      const uint32_t prev = phase;
      phase += inc;
      if (phase < prev) {  // переполнение аккумулятора — очередной шаг
        const int32_t np = isrPos + d;
        if (!isrLimOn || (np >= isrLimMin && np <= isrLimMax)) {
          GPIO_HI(PIN_Z_STEP);
          stepHigh = true;
          isrPos = np;
        }
      }
    }
  }

  // TRIG HC-SR04: импульс 20 мкс раз в usper мс
  if (++trigCnt >= isrTrigTicks) {
    trigCnt = 0;
    GPIO_HI(PIN_US_TRIG);
  } else if (trigCnt == 1) {
    GPIO_LO(PIN_US_TRIG);
  }
}

// Длительность импульса ECHO по меткам esp_timer (разрешение 1 мкс ≈ 0,17 мм).
static void IRAM_ATTR onEcho() {
  const uint32_t now = (uint32_t)esp_timer_get_time();
  if (REG_READ(GPIO_IN_REG) & (1UL << PIN_US_ECHO)) {
    echoRiseUs = now;
    echoHigh = true;
  } else if (echoHigh) {
    echoWidthUs = now - echoRiseUs;
    echoHigh = false;
    echoSeq = echoSeq + 1;
  }
}

// ----------------------------------------------------------------------------
float headStepsPerMm() { return (float)(Z_FULL_STEPS * Z_MICROSTEPS) / S.zMmPerRev; }
int32_t headPosSteps() { return isrPos; }
float headPosMm() { return (float)isrPos / headStepsPerMm(); }

bool headIsIdle() {
  return (H.mode == Z_IDLE || H.mode == Z_STOPPED) && H.vCmd == 0.0f;
}

static void setRate(float stepsPerSec) {
  float a = fabsf(stepsPerSec);
  if (a < 1.0f) {
    isrDir = 0;
    isrInc = 0;
    return;
  }
  if (a > Z_MAX_STEP_RATE) a = Z_MAX_STEP_RATE;
  float inc = a * (4294967296.0f / (float)STEP_TICK_HZ);
  if (inc > 2147483647.0f) inc = 2147483647.0f;  // не чаще шага через тик
  isrInc = (uint32_t)inc;
  isrDir = (stepsPerSec > 0.0f) ? 1 : -1;
}

static void setPosSteps(int32_t steps) {
  setRate(0.0f);
  H.vCmd = 0.0f;
  portENTER_CRITICAL(&zMux);
  isrPos = steps;
  portEXIT_CRITICAL(&zMux);
}

void headApplyDriverSettings() {
  driver.rms_current(S.zCurrent, 0.5f, Z_RSENSE);
  driver.stealthChop(S.zStealth != 0);
}

void headUpdateLimits() {
  isrLimMin = 0;
  isrLimMax = (int32_t)lroundf(S.zMax * headStepsPerMm());
  isrLimOn = H.homed && H.mode != Z_HOMING && !H.unhomedJog;
}

void headUpdateDir() {
  isrDirPosHigh = (S.zDirInvert == 0);
  isrCurDir = 0;  // пин DIR будет выставлен заново при следующем движении
}

void headSetTrigPeriod() { isrTrigTicks = (uint32_t)S.usPeriodMs * 1000u / STEP_TICK_US; }

void headBegin(const SavedPositions &p) {
  memset(&H, 0, sizeof(H));
  H.mode = Z_IDLE;

  pinMode(PIN_US_TRIG, OUTPUT);
  digitalWrite(PIN_US_TRIG, LOW);
  pinMode(PIN_US_ECHO, INPUT_PULLDOWN);  // оборванный провод ECHO = нет фронтов = авария по таймауту

  driver.begin();  // SPI.begin(), EN = HIGH (выключен)
  driver.microsteps(Z_MICROSTEPS);
  driver.interpolate(true);
  headApplyDriverSettings();
  pinMode(PIN_Z_STEP, OUTPUT);
  digitalWrite(PIN_Z_STEP, LOW);
  pinMode(PIN_Z_DIR, OUTPUT);
  headUpdateDir();
  digitalWrite(PIN_Z_EN, LOW);  // драйверы включены всегда: удерживают головку от сползания

  isrDir = 0;
  isrInc = 0;
  isrPos = p.zValid ? p.zSteps : 0;
  H.homed = p.zValid;
  H.homedFromNvs = p.zValid;
  headUpdateLimits();
  headSetTrigPeriod();

  attachInterrupt(digitalPinToInterrupt(PIN_US_ECHO), onEcho, CHANGE);
  stepTimer = timerBegin(STEP_TIMER_HZ);
  timerAttachInterrupt(stepTimer, &onStepTimer);
  timerAlarm(stepTimer, STEP_TICK_US, true, 0);
}

// ------------------------------- датчик --------------------------------------
float headDistEst() {
  if (!H.filtInit) return NAN;
  float dt = (float)(esp_timer_get_time() - H.tSampleUs) * 1e-6f;
  dt = clampf(dt, 0.0f, 2.0f * S.usPeriodMs * 0.001f);  // экстраполяция не дальше 2 периодов
  return H.xEst + H.vEst * dt;
}

bool headSensorFresh(uint32_t nowMs) {
  return H.filtInit && (uint32_t)(nowMs - H.lastValidMs) <= S.usTimeoutMs;
}

static void filterReset(float z, int64_t tUs, uint32_t nowMs) {
  H.xEst = z;
  H.vEst = 0.0f;
  H.filtInit = true;
  H.hasPending = false;
  H.tSampleUs = tUs;
  H.lastValidMs = nowMs;
  H.nValid++;
}

// α-β фильтр вместо SimpleKalmanFilter: нулевая установившаяся ошибка на равномерном
// движении поверхности (у прежнего фильтра коэффициент усиления падал почти до нуля,
// отсюда запаздывание ~400 мс). Отсчёты вне [usmin, usmax] — недостоверны;
// одиночный скачок больше usjump отбрасывается, два согласованных подряд — принимаются.
static void processEcho(uint32_t widthUs, int64_t tUs, uint32_t nowMs) {
  const float d = (float)widthUs * S.usSoS * 0.0005f * S.usScale + S.usOffset;  // мкс → мм
  H.distRaw = d;
  if (d < S.usMin || d > S.usMax) {
    H.nInvalid++;
    return;
  }
  const float z = fminf(d, S.usTrackMax);
  if (!H.filtInit) {
    filterReset(z, tUs, nowMs);
    return;
  }
  const float dt = (float)(tUs - H.tSampleUs) * 1e-6f;
  if (dt < 0.001f || dt > 0.5f) {  // дубликат или давно не было отсчётов
    if (dt > 0.5f) filterReset(z, tUs, nowMs);
    return;
  }
  const float xp = H.xEst + H.vEst * dt;
  const float r = z - xp;
  if (fabsf(r) > S.usJump) {
    if (H.hasPending && fabsf(z - H.pending) <= S.usJump) {
      filterReset(z, tUs, nowMs);  // скачок подтверждён вторым отсчётом
      gEvents |= EV_REACQUIRE;
    } else {
      H.pending = z;
      H.hasPending = true;
      H.nReject++;
    }
    return;
  }
  H.xEst = xp + S.fAlpha * r;
  H.vEst = clampf(H.vEst + (S.fBeta / dt) * r, -Z_SURFACE_SPEED_MAX, Z_SURFACE_SPEED_MAX);
  H.hasPending = false;
  H.tSampleUs = tUs;
  H.lastValidMs = nowMs;
  H.nValid++;
}

// ------------------------------- контур --------------------------------------
void headControlStep(uint32_t nowMs) {
  // 1. Новый отсчёт дальномера (повторное чтение — защита от гонки с ISR)
  uint32_t seq = echoSeq;
  if (seq != echoSeqSeen) {
    uint32_t w = echoWidthUs;
    if (echoSeq != seq) {
      seq = echoSeq;
      w = echoWidthUs;
    }
    echoSeqSeen = seq;
    processEcho(w, esp_timer_get_time(), nowMs);
  }

  // 2. Потеря датчика во время слежения — авария, отвод головки
  if (H.mode == Z_TRACK && !headSensorFresh(nowMs)) faultRaise(F_SENSOR);

  const float spm = headStepsPerMm();
  const float pos = (float)isrPos / spm;
  const float dvMax = S.zAccel * 0.001f;
  float vmax = S.zSpeed;
  float target = pos, vff = 0.0f;
  bool active = true;

  switch (H.mode) {
    case Z_STOPPED:
      H.vCmd = 0.0f;
      setRate(0.0f);
      H.targetMm = pos;
      return;
    case Z_IDLE:
      active = false;
      break;
    case Z_TRACK: {
      const float raw = fminf(headDistEst(), S.usTrackMax) - S.zGap;
      const bool sat = raw < 0.0f;
      if (sat && !H.satLow) gEvents |= EV_Z_SATURATED;
      H.satLow = sat;
      target = raw;
      vff = sat ? 0.0f : S.zFF * H.vEst;
      break;
    }
    case Z_MOVE:
      target = H.moveTargetMm;
      if (H.unhomedJog) vmax = fminf(vmax, Z_UNHOMED_JOG_SPEED);
      break;
    case Z_RETRACT:
      target = 0.0f;
      break;
    case Z_HOMING: {
      const int32_t travelled = H.homeStartSteps - isrPos;
      if (travelled >= H.homeTravelSteps) {
        setPosSteps(0);
        driver.rms_current(S.zCurrent, 0.5f, Z_RSENSE);
        H.homed = true;
        H.homedFromNvs = false;
        H.mode = Z_IDLE;
        headUpdateLimits();
        gEvents |= EV_HOME_DONE;
        return;
      }
      H.vCmd += clampf(-Z_HOME_SPEED - H.vCmd, -dvMax, dvMax);
      setRate(H.vCmd * spm);
      H.targetMm = pos;
      return;
    }
  }

  if (H.homed && !H.unhomedJog) target = clampf(target, 0.0f, S.zMax);
  H.targetMm = target;

  float vdes = 0.0f;
  if (active) {
    const float e = target - pos;
    const float ae = fabsf(e);
    if (H.holding) {
      if (ae > 2.0f * S.zDeadband || fabsf(vff) > Z_FF_HOLD_EXIT) H.holding = false;
    } else if (ae < S.zDeadband && fabsf(vff) < Z_FF_HOLD_EXIT) {
      H.holding = true;
    }
    if (!H.holding) {
      // P-регулятор с ограничением по тормозному пути: без перерегулирования при za
      const float vfb = fminf(S.zKp * ae, sqrtf(2.0f * S.zAccel * ae));
      vdes = vff + copysignf(vfb, e);
    }
  }
  vdes = clampf(vdes, -vmax, vmax);
  H.vCmd += clampf(vdes - H.vCmd, -dvMax, dvMax);
  setRate(H.vCmd * spm);

  // 3. Завершение перемещений
  if ((H.mode == Z_MOVE || H.mode == Z_RETRACT) && H.holding && fabsf(H.vCmd) < 0.01f) {
    gEvents |= (H.mode == Z_MOVE) ? EV_Z_DONE : EV_RETRACT_DONE;
    H.mode = Z_IDLE;
    H.holding = false;
    H.unhomedJog = false;
    headUpdateLimits();
  }
}

void headOnFault(uint32_t f) {
  if (f & (F_ESTOP | F_WDT)) {
    if (H.mode == Z_HOMING) driver.rms_current(S.zCurrent, 0.5f, Z_RSENSE);
    H.mode = Z_STOPPED;
    H.unhomedJog = false;
    setRate(0.0f);
    H.vCmd = 0.0f;
    headUpdateLimits();
  } else if ((f & F_SENSOR) && H.mode == Z_TRACK) {
    H.mode = Z_RETRACT;  // отвести сопло от пациента
    H.holding = false;
  }
}

void headFaultClear() {
  if (H.mode == Z_STOPPED) H.mode = Z_IDLE;
  H.holding = false;
  headUpdateLimits();
}

// ------------------------------- команды -------------------------------------
const char *headCmdTrack(bool on, uint32_t nowMs) {
  if (!on) {
    if (H.mode == Z_TRACK) H.mode = Z_IDLE;
    return nullptr;
  }
  if (H.mode == Z_TRACK) return nullptr;
  if (!H.homed) return "Z not homed (zhome / zset / zhomehard)";
  if (H.mode != Z_IDLE) return "Z busy";
  if (!headSensorFresh(nowMs)) return "no valid distance from sensor";
  H.mode = Z_TRACK;
  H.holding = false;
  H.satLow = false;
  return nullptr;
}

const char *headCmdMove(float targetMm) {
  if (!H.homed) return "Z not homed (zhome / zset / zhomehard)";
  if (H.mode != Z_IDLE && H.mode != Z_MOVE) return "Z busy (track0 first)";
  if (targetMm < 0.0f || targetMm > S.zMax) return "target outside 0..zmax";
  H.moveTargetMm = targetMm;
  H.mode = Z_MOVE;
  H.holding = false;
  return nullptr;
}

const char *headCmdJog(float deltaMm) {
  if (H.mode != Z_IDLE && H.mode != Z_MOVE) return "Z busy (track0 first)";
  const float base = (H.mode == Z_MOVE) ? H.moveTargetMm : headPosMm();
  if (!H.homed) {
    H.unhomedJog = true;  // пределы отключены, скорость ограничена Z_UNHOMED_JOG_SPEED
    H.moveTargetMm = base + deltaMm;
  } else {
    H.moveTargetMm = clampf(base + deltaMm, 0.0f, S.zMax);
  }
  H.mode = Z_MOVE;
  H.holding = false;
  headUpdateLimits();
  return nullptr;
}

const char *headCmdRetract() {
  if (!H.homed) return "Z not homed";
  if (H.mode == Z_HOMING || H.mode == Z_STOPPED) return "Z busy";
  H.mode = Z_RETRACT;
  H.holding = false;
  return nullptr;
}

const char *headCmdHomeHard() {
  if (!headIsIdle() || H.mode == Z_STOPPED) return "Z busy";
  H.homeStartSteps = isrPos;
  H.homeTravelSteps = (int32_t)lroundf((S.zMax + Z_HOME_EXTRA_MM) * headStepsPerMm());
  H.homed = false;
  H.unhomedJog = false;
  H.mode = Z_HOMING;
  headUpdateLimits();
  const float cur = fmaxf((float)S.zCurrent * Z_HOME_CURRENT_K, 50.0f);
  driver.rms_current((uint16_t)cur, 0.5f, Z_RSENSE);
  return nullptr;
}

const char *headCmdSetPos(float mm) {
  if (!headIsIdle() || H.mode == Z_STOPPED) return "stop Z first";
  setPosSteps((int32_t)lroundf(mm * headStepsPerMm()));
  H.homed = true;
  H.homedFromNvs = false;
  H.unhomedJog = false;
  headUpdateLimits();
  return nullptr;
}

const char *zModeName(uint8_t m) {
  switch (m) {
    case Z_IDLE: return "IDLE";
    case Z_TRACK: return "TRACK";
    case Z_MOVE: return "MOVE";
    case Z_RETRACT: return "RETRACT";
    case Z_HOMING: return "HOMING";
    case Z_STOPPED: return "STOPPED";
  }
  return "?";
}
