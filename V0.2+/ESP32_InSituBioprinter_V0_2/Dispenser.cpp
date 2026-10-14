// Dispenser.cpp — регулятор толкателя штока шприца.
//
// Что изменено относительно v0.1:
//  - опорная позиция дробная (double): шаг 262 тик/с × 0,02 с больше не усекается до целого;
//  - D-составляющая с правильным знаком: kd·(vref − v), а не kd·(e[k-1] − e[k]);
//  - антивиндап: ограничение интеграла и запрет накопления в насыщении;
//  - компенсация трения по направлению траектории, а не по знаку выхода ПИД
//    (убирает предельный цикл "стоит — рывок — толчок назад" на малых скоростях);
//  - ограничитель болюса: опора не уходит дальше emax от отстающего мотора;
//  - детектор заклинивания с отводом толкателя и детектор перекоса моторов;
//  - скорость по интервалу между фронтами энкодера (M/T-метод), а не по числу тиков за 20 мс;
//  - методы "редукции ШИМ" rk / rconst удалены: их заменяют emax + jamt.
#include "Dispenser.h"
#include "Config.h"
#include "Control.h"
#include "HeadAxis.h"
#include <math.h>
#include "esp_timer.h"
#include "soc/gpio_reg.h"
#include "soc/soc.h"

DispState D;

struct EncIsrState {
  volatile int32_t count;
  volatile uint32_t lastEdgeUs;
};
static EncIsrState encIsr[2];
static portMUX_TYPE encMux = portMUX_INITIALIZER_UNLOCKED;
static int32_t velPrevCount[2];
static uint32_t velPrevEdgeUs[2];

static const uint8_t PWM_PIN[2] = {PIN_D1_PWM, PIN_D2_PWM};
static const uint8_t IN1_PIN[2] = {PIN_D1_IN1, PIN_D2_IN1};
static const uint8_t IN2_PIN[2] = {PIN_D1_IN2, PIN_D2_IN2};

static inline float clampf(float v, float lo, float hi) {
  return v < lo ? lo : (v > hi ? hi : v);
}

// ------------------------------- энкодеры ------------------------------------
static inline __attribute__((always_inline)) uint32_t pinLevel(uint32_t pin) {
  return (pin < 32) ? ((REG_READ(GPIO_IN_REG) >> pin) & 1u) : ((REG_READ(GPIO_IN1_REG) >> (pin - 32)) & 1u);
}

static inline __attribute__((always_inline)) void encAdd(int idx, int32_t delta) {
  const uint32_t now = (uint32_t)esp_timer_get_time();
  portENTER_CRITICAL_ISR(&encMux);
  encIsr[idx].count = encIsr[idx].count + delta;
  encIsr[idx].lastEdgeUs = now;
  portEXIT_CRITICAL_ISR(&encMux);
}

// Декодирование x4 — то же, что в pin_A1/B1/A2/B2_ISR v0.1.
static void IRAM_ATTR isrEnc1A() { encAdd(0, ENC1_SIGN * ((pinLevel(PIN_D1_ENCA) == pinLevel(PIN_D1_ENCB)) ? -1 : 1)); }
static void IRAM_ATTR isrEnc1B() { encAdd(0, ENC1_SIGN * ((pinLevel(PIN_D1_ENCA) == pinLevel(PIN_D1_ENCB)) ? 1 : -1)); }
static void IRAM_ATTR isrEnc2A() { encAdd(1, ENC2_SIGN * ((pinLevel(PIN_D2_ENCA) == pinLevel(PIN_D2_ENCB)) ? -1 : 1)); }
static void IRAM_ATTR isrEnc2B() { encAdd(1, ENC2_SIGN * ((pinLevel(PIN_D2_ENCA) == pinLevel(PIN_D2_ENCB)) ? 1 : -1)); }

static void encSet(int32_t e1, int32_t e2) {
  const uint32_t now = (uint32_t)esp_timer_get_time();
  portENTER_CRITICAL(&encMux);
  encIsr[0].count = e1;
  encIsr[1].count = e2;
  encIsr[0].lastEdgeUs = now;
  encIsr[1].lastEdgeUs = now;
  portEXIT_CRITICAL(&encMux);
  velPrevCount[0] = e1;
  velPrevCount[1] = e2;
  velPrevEdgeUs[0] = velPrevEdgeUs[1] = now;
  D.enc[0] = e1;
  D.enc[1] = e2;
  D.vel[0] = D.vel[1] = 0.0f;
}

// ------------------------------- выход ---------------------------------------
// u > 0: IN1 = HIGH, IN2 = LOW (как level = 1 в v0.1); u = 0: короткое торможение.
static void motorOut(int i, int u) {
  if (zWdtTripped) return;  // ISR сторожевого таймера уже затормозил моторы
  if (u == 0) {
    digitalWrite(IN1_PIN[i], HIGH);
    digitalWrite(IN2_PIN[i], HIGH);
    ledcWrite(PWM_PIN[i], DISP_PWM_MAX);
  } else if (u > 0) {
    digitalWrite(IN1_PIN[i], HIGH);
    digitalWrite(IN2_PIN[i], LOW);
    ledcWrite(PWM_PIN[i], (uint32_t)u);
  } else {
    digitalWrite(IN1_PIN[i], LOW);
    digitalWrite(IN2_PIN[i], HIGH);
    ledcWrite(PWM_PIN[i], (uint32_t)(-u));
  }
}

static void holdHere() {
  D.ref = 0.5 * ((double)D.enc[0] + (double)D.enc[1]);
  D.target = (int32_t)llround(D.ref);
  D.vref = 0.0f;
  D.moving = false;
  D.relief = false;
  D.ext = 0;
  D.lagActive = false;
  D.holdErrActive = false;
  D.integ[0] = D.integ[1] = 0.0f;
}

void dispBegin(const SavedPositions &p) {
  memset(&D, 0, sizeof(D));
  for (int i = 0; i < 2; i++) {
    pinMode(IN1_PIN[i], OUTPUT);
    pinMode(IN2_PIN[i], OUTPUT);
    ledcAttach(PWM_PIN[i], DISP_PWM_FREQ, DISP_PWM_BITS);
  }
  pinMode(PIN_D1_ENCA, INPUT_PULLDOWN);
  pinMode(PIN_D1_ENCB, INPUT_PULLDOWN);
  pinMode(PIN_D2_ENCA, INPUT);  // у GPIO34/35 нет внутренних подтяжек
  pinMode(PIN_D2_ENCB, INPUT);

  encSet(p.dValid ? p.enc1 : 0, p.dValid ? p.enc2 : 0);
  holdHere();
  D.enabled = true;
  motorOut(0, 0);
  motorOut(1, 0);

  attachInterrupt(digitalPinToInterrupt(PIN_D1_ENCA), isrEnc1A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_D1_ENCB), isrEnc1B, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_D2_ENCA), isrEnc2A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_D2_ENCB), isrEnc2B, CHANGE);
}

int32_t dispLimitTicks(bool upper) {
  return (int32_t)lroundf((upper ? S.dMaxMm : S.dMinMm) * dispTicksPerMm(S));
}

bool dispIsIdle() {
  return !D.moving && !D.relief && D.u[0] == 0 && D.u[1] == 0;
}

// ------------------------------- контур --------------------------------------
void dispControlStep(uint32_t nowMs) {
  const float dt = DISP_PERIOD_MS * 0.001f;
  int32_t c[2];
  uint32_t te[2];
  portENTER_CRITICAL(&encMux);
  for (int i = 0; i < 2; i++) {
    c[i] = encIsr[i].count;
    te[i] = encIsr[i].lastEdgeUs;
  }
  portEXIT_CRITICAL(&encMux);
  const uint32_t nowUs = (uint32_t)esp_timer_get_time();

  // 1. Скорость: число фронтов / время между последними фронтами окон (M/T-метод)
  for (int i = 0; i < 2; i++) {
    if (c[i] != velPrevCount[i]) {
      const uint32_t dtu = te[i] - velPrevEdgeUs[i];
      const float dn = (float)(c[i] - velPrevCount[i]);
      D.vel[i] = (dtu > 0 && dtu < 1000000u) ? dn * 1e6f / (float)dtu : dn / dt;
      velPrevCount[i] = c[i];
      velPrevEdgeUs[i] = te[i];
    } else {
      const uint32_t age = nowUs - velPrevEdgeUs[i];
      if (age > 300000u) {
        D.vel[i] = 0.0f;
      } else {
        const float bound = 1e6f / (float)(age ? age : 1u);  // фронта нет — скорость не выше 1/age
        if (fabsf(D.vel[i]) > bound) D.vel[i] = copysignf(bound, D.vel[i]);
      }
    }
    D.enc[i] = c[i];
  }

  // 2. Перекос: два мотора на одном жёстком толкателе не должны расходиться
  const int32_t skew = c[0] - c[1];
  if (D.enabled && (skew > S.dSkewMax || -skew > S.dSkewMax)) faultRaise(F_SKEW);

  // 3. Планировщик: трапеция скорости к цели (как comp_cur_pos v0.1, но без усечения)
  const bool wasMoving = D.moving;
  if (D.moving) {
    const float vmax = D.relief ? DISP_RELIEF_SPEED : S.dVel;
    const double e = (double)D.target - D.ref;
    const float ae = (float)fabs(e);
    float vdes;
    if (S.dAcc > 0.0f) {
      vdes = copysignf(fminf(vmax, sqrtf(2.0f * S.dAcc * ae)), (float)e);
      const float dv = S.dAcc * dt;
      D.vref += clampf(vdes - D.vref, -dv, dv);
    } else {
      D.vref = copysignf(vmax, (float)e);
    }
    const double step = (double)D.vref * dt;
    if (ae < 0.5f || (e >= 0.0 && step >= e) || (e <= 0.0 && step <= e)) {
      D.ref = D.target;
      D.vref = 0.0f;
      D.moving = false;
    } else {
      D.ref += step;
    }
  }

  // 4. Ограничитель болюса: опора не дальше emax от отстающего мотора.
  //    При заклинивании дефицит не копится, после освобождения нет выброса чернил.
  const double lo = (double)(c[0] > c[1] ? c[0] : c[1]) - S.dEmax;
  const double hi = (double)(c[0] < c[1] ? c[0] : c[1]) + S.dEmax;
  D.clampDir = 0;
  if (lo <= hi) {
    if (D.ref > hi) {
      D.ref = hi;
      D.clampDir = 1;
    } else if (D.ref < lo) {
      D.ref = lo;
      D.clampDir = -1;
    }
  }

  // 5. Заклинивание / окклюзия: опора упирается в emax дольше jamt
  const bool lagging = D.moving && !D.relief &&
                       ((D.clampDir > 0 && D.vref > 0.0f) || (D.clampDir < 0 && D.vref < 0.0f));
  if (lagging) {
    if (!D.lagActive) {
      D.lagActive = true;
      D.lagSinceMs = nowMs;
    } else if ((uint32_t)(nowMs - D.lagSinceMs) >= S.dJamMs) {
      faultRaise(F_JAM);
    }
  } else {
    D.lagActive = false;
  }

  // 5а. Удержание: если в покое мотор дольше jamt не может дойти до опоры (трение,
  //     упор), текущая позиция принимается как цель — без бесконечного давления на мотор.
  const bool holdErr = !D.moving && D.enabled &&
                       (fabs(D.ref - c[0]) > S.dDeadband || fabs(D.ref - c[1]) > S.dDeadband);
  if (!holdErr) {
    D.holdErrActive = false;
  } else if (!D.holdErrActive) {
    D.holdErrActive = true;
    D.holdErrSinceMs = nowMs;
  } else if ((uint32_t)(nowMs - D.holdErrSinceMs) >= S.dJamMs) {
    holdHere();
  }

  if (wasMoving && !D.moving) {
    if (D.relief) {
      D.relief = false;
    } else {
      gEvents |= (D.ext != 0) ? EV_D_LIMIT : EV_D_DONE;
      D.ext = 0;
    }
  }

  // 6. ПИД каждого мотора с упреждением по скорости и компенсацией трения
  for (int i = 0; i < 2; i++) {
    int16_t u = 0;
    if (D.enabled) {
      const float e = (float)(D.ref - (double)c[i]);
      const bool hold = !D.moving && D.vref == 0.0f && fabsf(e) <= (float)S.dDeadband;
      if (hold) {
        D.integ[i] = 0.0f;
      } else {
        float ff = S.dKv * D.vref;
        if (D.vref > 0.5f) ff += S.dMinDuty;
        else if (D.vref < -0.5f) ff -= S.dMinDuty;
        else if (e > (float)S.dDeadband) ff += S.dMinDuty;  // доводка в покое
        else if (e < -(float)S.dDeadband) ff -= S.dMinDuty;
        const float p = S.dKp * e;
        const float dterm = S.dKd * (D.vref - D.vel[i]);
        const float maxU = (float)S.dMaxDuty;
        const float uRaw = p + D.integ[i] + dterm + ff;
        const bool satHi = (uRaw > maxU || D.clampDir > 0) && e > 0.0f;
        const bool satLo = (uRaw < -maxU || D.clampDir < 0) && e < 0.0f;
        // Антивиндап: не копить интеграл в насыщении ШИМ и пока опора упирается в emax
        // (иначе после заклинивания накопленный интеграл выдавливает лишние чернила).
        if (!satHi && !satLo) {
          D.integ[i] = clampf(D.integ[i] + S.dKi * e * dt, -S.dIMax, S.dIMax);
        }
        u = (int16_t)lroundf(clampf(p + D.integ[i] + dterm + ff, -maxU, maxU));
      }
    }
    D.u[i] = u;
    motorOut(i, u);
  }
}

// ------------------------------- аварии --------------------------------------
void dispStopSmooth() {
  D.ext = 0;
  if (!D.moving) return;
  if (S.dAcc > 0.0f) {
    const double dist = (double)D.vref * D.vref / (2.0 * S.dAcc);
    D.target = (int32_t)llround(D.ref + (D.vref >= 0.0f ? dist : -dist));
  } else {
    D.target = (int32_t)llround(D.ref);
    D.ref = D.target;
    D.vref = 0.0f;
    D.moving = false;
  }
}

void dispOnFault(uint32_t f) {
  if (f & (F_ESTOP | F_SKEW | F_WDT)) {
    holdHere();
    D.enabled = false;
    motorOut(0, 0);
    motorOut(1, 0);
    D.u[0] = D.u[1] = 0;
  } else if (f & F_JAM) {
    // Сброс давления: отвести толкатель на relief тиков против направления подачи
    const int dir = (D.vref >= 0.0f) ? 1 : -1;
    holdHere();
    D.target = (int32_t)llround(D.ref) - dir * S.dRelief;
    D.moving = S.dRelief > 0;
    D.relief = D.moving;
  } else if (f & F_SENSOR) {
    dispStopSmooth();
  }
}

void dispFaultClear() {
  holdHere();
  D.enabled = true;
}

// ------------------------------- команды -------------------------------------
const char *dispCmdTarget(int64_t ticks) {
  if (!D.enabled) return "dispenser disabled (fault)";
  if (S.dVel <= 0.0f) return "vel = 0";
  if (ticks < dispLimitTicks(false) || ticks > dispLimitTicks(true)) return "target outside dmin..dmax";
  D.target = (int32_t)ticks;
  D.moving = true;
  D.relief = false;
  D.ext = 0;
  return nullptr;
}

const char *dispCmdExt(int dir) {
  if (dir == 0) {
    dispStopSmooth();
    return nullptr;
  }
  const char *err = dispCmdTarget(dispLimitTicks(dir > 0));
  if (!err) D.ext = (int8_t)(dir > 0 ? 1 : -1);
  return err;
}

const char *dispCmdZero() {
  if (D.moving || D.relief) return "dispenser moving";
  encSet(0, 0);
  holdHere();
  return nullptr;
}
