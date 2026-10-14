// sim.cpp — проверка логики прошивки v0.2 на ПК, без железа.
//
// Код прошивки (Settings, Control, HeadAxis, Dispenser, Commands) компилируется как есть,
// ESP32 API заменён заглушками (stubs/). Симулятор моделирует:
//  - шаговый привод: считает фронты STEP с учётом уровня DIR;
//  - HC-SR04: эхо по TRIG, шум, перескоки на период несущей (~4,3 мм), пропуски эха;
//  - два ДПТ N20 298:1 с сухим трением на одном толкателе, квадратурные энкодеры;
//  - заклинивание толкателя и остановку контура управления.
// Модель моторов — грубая (параметры подобраны "по порядку величины"), поэтому
// цифры здесь — проверка логики и сравнение вариантов, а не предсказание стенда.
#include <functional>
#include <random>
#include <vector>

#include "Arduino.h"
#include "Preferences.h"
#include "esp_timer.h"
#include "soc/gpio_reg.h"

#include "../ESP32_InSituBioprinter_V0_2/Commands.h"
#include "../ESP32_InSituBioprinter_V0_2/Config.h"
#include "../ESP32_InSituBioprinter_V0_2/Control.h"
#include "../ESP32_InSituBioprinter_V0_2/Dispenser.h"
#include "../ESP32_InSituBioprinter_V0_2/HeadAxis.h"
#include "../ESP32_InSituBioprinter_V0_2/Settings.h"

// ============================== HAL-заглушки =================================
static int64_t tUs = 0;
uint32_t millis() { return (uint32_t)(tUs / 1000); }
uint32_t micros() { return (uint32_t)tUs; }
int64_t esp_timer_get_time() { return tUs; }
void delay(uint32_t) {}
SimSerial Serial;
EspClass ESP;
std::map<std::string, std::vector<uint8_t>> simNvs;
int simNvsWrites = 0;

static uint64_t outBits = 0, inBits = 0;
static uint32_t pwmDuty[64];
static void (*pinIsr[64])(void);
static void (*timerIsr)(void);
struct hw_timer_t {
  int unused;
};
static hw_timer_t theTimer;

static inline uint64_t bit(int p) { return 1ull << p; }

// ---- модель шагового привода ----
static int64_t motorSteps = 0;  // реально сделанные шаги (по фронтам STEP)

// ---- модель HC-SR04 ----
static std::mt19937 rng(12345);
static std::function<double(double)> surface = [](double) { return 50.0; };  // мм от датчика
static bool sensorDead = false;
static double noiseSigma = 0.0, pCycleJump = 0.0, pMiss = 0.0;
static int64_t echoRise = -1, echoFall = -1;

static void sensorTrigger() {
  if (echoRise >= 0 || (inBits & bit(PIN_US_ECHO)) || sensorDead) return;
  std::uniform_real_distribution<double> u(0, 1);
  std::normal_distribution<double> n(0, 1);
  double widthUs;
  if (u(rng) < pMiss) {
    widthUs = 38000;  // нет эха — модуль выдаёт импульс-таймаут
  } else {
    double d = surface(tUs * 1e-6) + noiseSigma * n(rng);
    if (u(rng) < pCycleJump) d += 343.0 / 40000.0 / 2.0 * 1000.0;  // пропуск периода 40 кГц
    widthUs = d / 0.1715;                                            // 343 м/с туда-обратно
  }
  echoRise = tUs + 450;
  echoFall = echoRise + (int64_t)llround(widthUs);
}

static void onOutputs(uint64_t before, uint64_t after) {
  if (!(before & bit(PIN_Z_STEP)) && (after & bit(PIN_Z_STEP))) motorSteps += (after & bit(PIN_Z_DIR)) ? 1 : -1;
  if (!(before & bit(PIN_US_TRIG)) && (after & bit(PIN_US_TRIG))) sensorTrigger();
}

void simRegWrite(uint32_t reg, uint32_t v) {
  const uint64_t b = outBits;
  if (reg == GPIO_OUT_W1TS_REG) outBits |= v;
  if (reg == GPIO_OUT_W1TC_REG) outBits &= ~(uint64_t)v;
  onOutputs(b, outBits);
}
uint32_t simRegRead(uint32_t reg) {
  if (reg == GPIO_IN_REG) return (uint32_t)inBits;
  if (reg == GPIO_IN1_REG) return (uint32_t)(inBits >> 32);
  return 0;
}
void pinMode(uint8_t, uint8_t) {}
void digitalWrite(uint8_t p, uint8_t v) {
  const uint64_t b = outBits;
  if (v) outBits |= bit(p);
  else outBits &= ~bit(p);
  onOutputs(b, outBits);
}
int digitalRead(uint8_t p) { return (int)((inBits >> p) & 1); }
bool ledcAttach(uint8_t, uint32_t, uint8_t) { return true; }
bool ledcWrite(uint8_t p, uint32_t d) {
  pwmDuty[p] = d;
  return true;
}
void attachInterrupt(uint8_t p, void (*fn)(void), int) { pinIsr[p] = fn; }
hw_timer_t *timerBegin(uint32_t) { return &theTimer; }
void timerAttachInterrupt(hw_timer_t *, void (*fn)(void)) { timerIsr = fn; }
void timerAlarm(hw_timer_t *, uint64_t, bool, uint64_t) {}

static void setInput(int p, bool level) {
  if (((inBits >> p) & 1) == (uint64_t)level) return;
  if (level) inBits |= bit(p);
  else inBits &= ~bit(p);
  if (pinIsr[p]) pinIsr[p]();
}

// ---- модель дозатора: два ДПТ на одном толкателе ----
struct Motor {
  double x = 0, v = 0;  // тик, тик/с
  int64_t tick = 0;
  int q = 0;            // фаза квадратуры 0..3
};
static Motor mot[2];
static double VNL = 13900;   // тик/с холостого хода при ШИМ 1023 (≈100 об/мин)
static double TAU = 0.02;    // с, механическая постоянная
static double STICTION = 230, KINETIC = 180;  // ШИМ, трение покоя / скольжения
static double KC = 5.0;      // ШИМ на тик рассогласования — жёсткость толкателя
static bool jam[2] = {false, false};
static const int ENC_A[2] = {PIN_D1_ENCA, PIN_D2_ENCA}, ENC_B[2] = {PIN_D1_ENCB, PIN_D2_ENCB};
static const int IN1[2] = {PIN_D1_IN1, PIN_D2_IN1}, IN2[2] = {PIN_D1_IN2, PIN_D2_IN2}, PWM[2] = {PIN_D1_PWM, PIN_D2_PWM};
// Прямой ход: мотор 1 — B опережает A, мотор 2 — A опережает B (так считают ISR v0.1/v0.2)
static const int SEQ[2][4][2] = {{{0, 0}, {0, 1}, {1, 1}, {1, 0}}, {{0, 0}, {1, 0}, {1, 1}, {0, 1}}};

static void encoderUpdate(int i) {
  const int64_t want = (int64_t)floor(mot[i].x);
  while (mot[i].tick != want) {
    const int dir = want > mot[i].tick ? 1 : -1;
    mot[i].q = (mot[i].q + dir + 4) % 4;
    mot[i].tick += dir;
    const int a = SEQ[i][mot[i].q][0], b = SEQ[i][mot[i].q][1];
    setInput(ENC_A[i], a);
    setInput(ENC_B[i], b);
  }
}

static void motorsStep(double dt) {
  for (int i = 0; i < 2; i++) {
    Motor &m = mot[i];
    const bool in1 = outBits & bit(IN1[i]), in2 = outBits & bit(IN2[i]);
    const double duty = pwmDuty[PWM[i]];
    double d = (in1 && !in2) ? duty : ((!in1 && in2) ? -duty : 0.0);
    const bool brake = in1 && in2;
    const double deff = d + KC * (mot[1 - i].x - m.x);
    if (jam[i]) {
      m.v = 0;
    } else if (brake) {
      m.v += (-m.v / 0.005) * dt;
      if (fabs(m.v) < 1) m.v = 0;
    } else if (m.v == 0.0) {
      if (fabs(deff) > STICTION) m.v += VNL / 1023.0 * (deff - copysign(KINETIC, deff)) / TAU * dt;
    } else {
      const double vt = VNL / 1023.0 * (deff - copysign(KINETIC, m.v));
      const double nv = m.v + (vt - m.v) / TAU * dt;
      m.v = (nv * m.v < 0) ? 0.0 : nv;  // смена знака — через остановку (трение покоя)
    }
    m.x += m.v * dt;
    encoderUpdate(i);
  }
}

// ============================== цикл симуляции ===============================
static bool controlStalled = false;
static uint32_t dispDiv = 0;
static std::string captured;  // вывод прошивки с момента последней команды

static void flushSerial() {
  if (Serial.tx.empty()) return;
  printf("%s", Serial.tx.c_str());
  captured += Serial.tx;
  Serial.tx.clear();
}

static void controlTick() {  // тело controlTaskFn без FreeRTOS
  zHeartbeat = zHeartbeat + 1;
  if (zWdtTripped && !(gFaults & F_WDT)) faultRaise(F_WDT);
  headControlStep(millis());
  if (++dispDiv >= DISP_PERIOD_MS) {
    dispDiv = 0;
    dispControlStep(millis());
  }
}

static void run(double seconds, const std::function<void()> &everyMs = nullptr) {
  const int64_t end = tUs + (int64_t)llround(seconds * 1e6);
  while (tUs < end) {
    tUs++;
    if (echoRise >= 0 && tUs == echoRise) setInput(PIN_US_ECHO, true);
    if (echoFall >= 0 && tUs == echoFall) {
      setInput(PIN_US_ECHO, false);
      echoRise = echoFall = -1;
    }
    if (tUs % 10 == 0) motorsStep(10e-6);
    if (tUs % STEP_TICK_US == 0 && timerIsr) timerIsr();
    if (tUs % 1000 == 0) {
      if (!controlStalled) controlTick();
      commandsPoll();
      eventsPoll();
      persistencePoll();
      flushSerial();
      if (everyMs) everyMs();
    }
  }
}

static std::string cmd(const std::string &line, double settle = 0.02) {
  captured.clear();
  Serial.rx += line + "\n";
  run(settle > 0.002 ? settle : 0.002);
  return captured;
}

static int failures = 0;
static void check(bool ok, const char *what) {
  printf("  [%s] %s\n", ok ? "PASS" : "FAIL", what);
  if (!ok) failures++;
}
static bool has(const std::string &s, const char *sub) { return s.find(sub) != std::string::npos; }

static void boot(bool keepNvs = false) {
  if (!keepNvs) simNvs.clear();  // каждый сценарий — с "чистого" устройства
  tUs = 0;
  outBits = inBits = 0;
  memset(pwmDuty, 0, sizeof(pwmDuty));
  mot[0] = mot[1] = Motor();
  motorSteps = 0;
  jam[0] = jam[1] = false;
  controlStalled = false;
  sensorDead = false;
  noiseSigma = pCycleJump = pMiss = 0;
  echoRise = echoFall = -1;
  gFaults = gEvents = 0;
  zWdtTripped = false;
  settingsDefaults(S);
  controlInit();
  SavedPositions p;
  positionsLoad(p);
  dispBegin(p);
  headBegin(p);
  commandsBegin();
  zWdtArmed = true;
  Serial.tx.clear();
}

static double tri(double t, double lo, double hi, double v) {  // треугольник lo..hi со скоростью v
  const double span = hi - lo, period = 2 * span / v;
  double ph = fmod(t, period);
  return ph < period / 2 ? lo + v * ph : hi - v * (ph - period / 2);
}

// ============================== тесты ========================================
static void testParser() {
  printf("\n=== 1. Разбор команд ===\n");
  boot();
  cmd("speed40");
  check(S.zSpeed == 20.0f && S.pSpeed == 40.0f, "speed40 не меняет sp (в v0.1 sp сбрасывался в 1 мм/с)");
  cmd("height0.34; width0.4; diameter14.85");
  check(fabsf(S.pHeight - 0.34f) < 1e-6f && fabsf(S.pWidth - 0.4f) < 1e-6f && fabsf(S.pSyrDiam - 14.85f) < 1e-5f,
        "height/width/diameter принимают дробные значения (в v0.1 — uint16_t)");
  check(fabsf(S.dVel - 262.2f) < 1.0f, "скорость штока из w·h·v/S ≈ 262 тик/с");
  std::string o = cmd("kp");
  check(has(o, "kp=0.4"), "команда без значения выводит параметр");
  o = cmd("kp 0.5; ki=3");
  check(S.dKp == 0.5f && S.dKi == 3.0f, "форматы 'kp 0.5' и 'ki=3', две команды через ';'");
  o = cmd("velMMS0.01");
  check(fabsf(S.dVel - 83.44f) < 0.1f, "velMMS не задевает vel (точное совпадение имён)");
  o = cmd("foo12");
  check(has(o, "ERR unknown"), "неизвестная команда — ошибка");
  o = cmd("kp abc");
  check(has(o, "bad value"), "нечисловое значение — ошибка");
  o = cmd("kp 50");
  check(has(o, "ERR kp: range"), "значение вне диапазона отклоняется");
  o = cmd("rk5");
  check(has(o, "removed"), "rk / rconst удалены с пояснением");
  o = cmd("zgo10");
  check(has(o, "not homed"), "zgo без хоминга запрещён");
  captured.clear();
  Serial.rx += "status";  // без перевода строки: завершение по паузе 50 мс
  run(0.1);
  check(has(captured, "faults: none"), "строка без '\\n' выполняется после паузы");
  o = cmd("sp");
  check(has(o, "sp=20"), "sp не изменился после всех команд");
}

struct TrackStats {
  double rms, maxAbs, lagMs;
};

static TrackStats trackRun(double speed, bool artefacts, double seconds) {
  boot();
  surface = [speed](double t) { return 30.0 + tri(t, 0.0, 21.5, speed); };
  if (artefacts) {
    noiseSigma = 0.5;
    pCycleJump = 0.05;
    pMiss = 0.01;
  }
  cmd("zhome; sh20");
  run(0.2);
  cmd("track1");
  std::vector<double> err, cmdPos, want;
  run(seconds, [&]() {
    if (tUs < 5000000) return;  // пропустить выход на режим
    const double posMm = motorSteps / headStepsPerMm();
    const double target = surface(tUs * 1e-6) - 20.0;
    err.push_back(posMm - target);
    cmdPos.push_back(posMm);
    want.push_back(target);
  });
  double s2 = 0, mx = 0;
  for (double e : err) {
    s2 += e * e;
    mx = std::max(mx, fabs(e));
  }
  // Задержка: сдвиг (мс), минимизирующий СКО между позицией и целью
  double bestLag = 0, best = 1e18;
  for (int lag = 0; lag <= 500; lag += 5) {
    double acc = 0;
    int n = 0;
    for (size_t k = lag; k < cmdPos.size(); k++, n++) acc += pow(cmdPos[k] - want[k - lag], 2);
    if (n && acc / n < best) {
      best = acc / n;
      bestLag = lag;
    }
  }
  return {sqrt(s2 / err.size()), mx, bestLag};
}

static void testTracking() {
  printf("\n=== 2. Слежение за поверхностью (повтор эксперимента ВКР: треугольник 21,5 мм) ===\n");
  for (double v : {2.0, 4.0}) {
    for (bool art : {false, true}) {
      const TrackStats s = trackRun(v, art, 40);
      printf("  %.0f мм/с, датчик %-34s: ошибка зазора СКО %.2f мм, макс %.2f мм, задержка %.0f мс\n", v,
             art ? "с шумом 0,5 мм, перескоками и пропусками" : "идеальный", s.rms, s.maxAbs, s.lagMs);
      if (!art) check(s.lagMs <= 100 && s.rms < 0.5, "без артефактов датчика: задержка ≤ 100 мс, СКО < 0,5 мм");
      if (art) check(gFaults == 0, "редкие пропуски эха не вызывают ложной аварии");
    }
  }
  printf("  (ВКР, рис. 16–17, v0.1: задержка 400 мс; разные условия — сравнение ориентировочное)\n");
}

static void testZMoveSpeed() {
  printf("\n=== 3. Скорость главного привода и согласованность шагов ===\n");
  boot();
  cmd("zhome; sp20");
  cmd("zgo50", 0.0);
  const int64_t t0 = tUs;
  bool done = false;
  run(5.0, [&]() {
    if (!done && H.mode == Z_IDLE && tUs - t0 > 100000) {
      done = true;
      printf("  zgo50 при sp20: %.2f с (теория с разгоном 1000 мм/с²: %.2f с)\n", (tUs - t0) * 1e-6, 50.0 / 20.0 + 0.02);
    }
  });
  check(done, "перемещение завершено");
  printf("  шагов на выходе STEP: %lld, счётчик прошивки: %ld\n", (long long)motorSteps, (long)headPosSteps());
  check(llabs(motorSteps - headPosSteps()) == 0, "шаги на выходе STEP совпадают со счётчиком прошивки");
  check(fabs(headPosMm() - 50.0) < 0.06, "позиция 50 мм ± зона нечувствительности");
  std::string o = cmd("zgo100");
  check(has(o, "outside"), "цель за zmax отклоняется");
}

static void testSensorLoss() {
  printf("\n=== 4. Потеря датчика во время слежения ===\n");
  boot();
  surface = [](double) { return 60.0; };
  cmd("zhome; sh20");
  run(0.2);
  cmd("track1");
  run(3.0);
  const double before = headPosMm();
  sensorDead = true;
  const int64_t tLoss = tUs;
  int64_t tFault = -1;
  run(4.0, [&]() {
    if (tFault < 0 && (gFaults & F_SENSOR)) tFault = tUs;
  });
  printf("  позиция до обрыва %.2f мм, авария через %.0f мс, позиция после %.2f мм\n", before,
         (tFault - tLoss) * 1e-3, headPosMm());
  check(tFault > 0 && (tFault - tLoss) <= 130000, "авария SENSOR не позже 130 мс после обрыва");
  check(fabs(headPosMm()) < 0.06, "головка отведена в 0 мм");
  std::string o = cmd("track1");
  check(has(o, "fault active"), "слежение не включается до clear");

  printf("  -- поверхность ближе usmin (20 мм) --\n");
  boot();
  surface = [](double) { return 40.0; };
  cmd("zhome; sh10");
  run(0.2);
  cmd("track1");
  run(1.0);
  surface = [](double) { return 12.0; };
  run(0.5);
  check((gFaults & F_SENSOR) != 0, "недостоверная дистанция (< usmin) — авария и отвод");
}

static double dispDelivered() { return 0.5 * (D.enc[0] + D.enc[1]); }

static void testDispenserPrint() {
  printf("\n=== 5. Дозатор на скорости печати (262 тик/с ≈ 0,031 мм/с) ===\n");
  boot();
  cmd("Z0.5", 0.0);  // 0,5 мм = 4172 тика ≈ 15,9 с
  std::vector<double> v20;
  double prev = dispDelivered(), p5 = 0, p15 = 0;
  int reverse = 0, stopped = 0, n = 0;
  run(20.0, [&]() {
    if (tUs % 20000 != 0) return;
    const double now = dispDelivered();
    const double vw = (now - prev) / 0.02;
    prev = now;
    const double t = tUs * 1e-6;
    if (t > 2 && t < 14) {
      v20.push_back(vw);
      n++;
      if (vw < 0) reverse++;
      if (vw == 0) stopped++;
    }
    if (fabs(t - 5.0) < 1e-6) p5 = now;
    if (fabs(t - 15.0) < 1e-6) p15 = now;
  });
  double mean = 0, sd = 0;
  for (double x : v20) mean += x / v20.size();
  for (double x : v20) sd += (x - mean) * (x - mean) / v20.size();
  sd = sqrt(sd);
  printf("  средняя %.1f тик/с, СКО по окнам 20 мс %.1f, окон с остановкой %d%%, с обратным ходом %d%%\n", mean, sd,
         100 * stopped / std::max(n, 1), 100 * reverse / std::max(n, 1));
  printf("  подача за 5..15 с: %.0f тик (номинал %.0f), итог %.1f из 4172\n", p15 - p5, 10 * S.dVel, dispDelivered());
  check(fabs((p15 - p5) / (10 * S.dVel) - 1.0) < 0.02, "подача в пределах 2% от номинала (в v0.1 — недолив 4,6%)");
  check(reverse == 0, "нет обратного хода (в v0.1 модель давала ~10% окон)");
  check(fabs(dispDelivered() - 4172) <= S.dDeadband + 1, "толкатель остановился в цели ± ddb");
}

static double jamBolus(int32_t emax, double jamSec) {
  boot();
  char b[64];
  snprintf(b, sizeof(b), "emax%ld; vel2000", (long)emax);
  cmd(b);
  cmd("ext1", 0.0);
  run(2.0);
  jam[0] = jam[1] = true;
  run(jamSec);
  jam[0] = jam[1] = false;
  const double start = dispDelivered();
  run(1.0);
  return (dispDelivered() - start) - S.dVel * 1.0;  // сверх номинальной подачи за 1 с
}

static void testJam() {
  printf("\n=== 6. Заклинивание толкателя ===\n");
  const double withLimit = jamBolus(150, 0.5);
  const double noLimit = jamBolus(200000, 0.5);
  printf("  кратковременное (0,5 с) при 2000 тик/с: выброс после освобождения %.0f тик (≈%.1f мкл) с emax=150,\n"
         "  %.0f тик (≈%.1f мкл) без ограничителя (поведение v0.1 без редукции)\n",
         withLimit, withLimit / 8344.0 * 173.2, noLimit, noLimit / 8344.0 * 173.2);
  check(withLimit < 300, "ограничитель emax держит выброс в пределах ~2·emax");
  check(noLimit > 3 * withLimit, "без ограничителя выброс в разы больше");

  boot();
  cmd("vel2000");
  cmd("ext1", 0.0);
  run(1.0);
  jam[0] = jam[1] = true;
  const int64_t tj = tUs;
  int64_t tf = -1;
  run(2.0, [&]() {
    if (tf < 0 && (gFaults & F_JAM)) tf = tUs;
  });
  const double atFault = dispDelivered();
  jam[0] = jam[1] = false;
  run(1.0);
  printf("  длительное: авария JAM через %.0f мс, отвод %.0f тик\n", (tf - tj) * 1e-3, atFault - dispDelivered());
  check(tf > 0 && (tf - tj) < 1300000, "авария JAM ≈ через jamt (1 с)");
  check(atFault - dispDelivered() > 50, "после аварии толкатель отведён (сброс давления)");
  std::string o = cmd("Z1");
  check(has(o, "fault active"), "подача запрещена до clear");

  printf("  -- удержание против упора: толкатель сдвинут извне и зажат --\n");
  boot();
  cmd("vel2000; Z0.05", 0.0);
  run(1.0);
  for (Motor &m : mot) m.x += 60;  // внешний сдвиг на 60 тик
  jam[0] = jam[1] = true;
  run(0.3);
  const bool pushing = D.u[0] != 0 || D.u[1] != 0;
  const int u0 = D.u[0], u1 = D.u[1];
  run(1.5);
  printf("  через 0,3 с ШИМ %d/%d, через 1,8 с ШИМ %d/%d\n", u0, u1, D.u[0], D.u[1]);
  check(pushing && D.u[0] == 0 && D.u[1] == 0 && gFaults == 0, "через jamt позиция принята, мотор не давит бесконечно");
  jam[0] = jam[1] = false;
  boot();
  o = cmd("clear; Z0.01");
  check(has(o, "OK Z"), "после clear подача снова разрешена");
}

static void testSkew() {
  printf("\n=== 7. Перекос: один мотор встал, толкатель податлив ===\n");
  boot();
  KC = 0.0;
  cmd("vel2000");
  cmd("ext1", 0.0);
  run(0.5);
  jam[0] = true;
  run(2.0);
  check((gFaults & F_SKEW) != 0, "авария SKEW при расхождении энкодеров > skew");
  check((outBits & bit(PIN_D2_IN1)) && (outBits & bit(PIN_D2_IN2)), "второй мотор на тормозе");
  KC = 5.0;
}

static void testWatchdog() {
  printf("\n=== 8. Остановка контура управления (сторожевой таймер в ISR) ===\n");
  boot();
  cmd("zhome; sp20");
  cmd("zgo60; vel2000; ext1", 0.0);
  run(0.5);
  check(H.vCmd > 15.0f && D.moving, "до зависания ось Z и дозатор движутся");
  controlStalled = true;  // задача управления "зависла": скорость шагов и ШИМ застыли
  const int64_t stepsAtStall = motorSteps;
  run(0.2);
  const int64_t stepsAfter = motorSteps;
  const bool braked = (outBits & bit(PIN_D1_IN1)) && (outBits & bit(PIN_D1_IN2)) && (outBits & bit(PIN_D2_IN1)) &&
                      (outBits & bit(PIN_D2_IN2));
  printf("  за 200 мс зависания сделано %lld шагов (без защиты было бы ~%lld)\n", (long long)(stepsAfter - stepsAtStall),
         (long long)(0.2 * 20 * headStepsPerMm()));
  check(zWdtTripped, "сторожевой таймер сработал");
  check(stepsAfter - stepsAtStall < 0.06 * 20 * headStepsPerMm(), "шаги остановлены ≤ 60 мс после зависания");
  check(braked, "ДПТ дозатора на тормозе из ISR");
  controlStalled = false;
  run(0.1);
  check((gFaults & F_WDT) != 0, "после восстановления контура — авария WDT");
}

static void testPersistence() {
  printf("\n=== 9. Сохранение позиций и E-stop ===\n");
  simNvs.clear();
  boot();
  cmd("zhome");
  cmd("zgo10", 0.0);
  run(2.5);
  int32_t zpos = 0;
  uint8_t zok = 0;
  Preferences p;
  p.getBytes("zpos", &zpos, 4);
  p.getBytes("zok", &zok, 1);
  check(zok == 1 && zpos == headPosSteps(), "позиция Z записана в NVS после 1 с покоя");
  surface = [](double) { return 50.0; };
  run(0.1);
  const int writes = simNvsWrites;
  cmd("track1");
  run(2.0);
  p.getBytes("zok", &zok, 1);
  check(zok == 0, "при старте слежения запись помечена 'в движении'");
  check(simNvsWrites - writes <= 2, "во время слежения flash не пишется");
  std::string o = cmd("stop");
  const int64_t s0 = motorSteps;
  surface = [](double t) { return 50.0 + 10 * sin(t); };
  run(1.0);
  check(motorSteps == s0 && has(o, "STOP"), "stop: шаги прекращены немедленно");
  o = cmd("clear; track1");
  check(has(o, "OK track=1"), "clear + track1 возобновляют слежение");

  printf("  -- перезагрузка после покоя и после пропадания питания в движении --\n");
  cmd("track0");
  run(2.0);  // остановка и 1 с покоя -> запись позиции
  const int32_t saved = headPosSteps();
  boot(true);
  check(H.homed && headPosSteps() == saved, "после перезагрузки в покое позиция Z восстановлена");
  cmd("zgo20", 0.0);
  run(0.3);
  boot(true);  // "питание пропало" во время движения
  check(!H.homed, "после пропадания питания в движении Z считается не отхомленной");
}

int main() {
  testParser();
  testTracking();
  testZMoveSpeed();
  testSensorLoss();
  testDispenserPrint();
  testJam();
  testSkew();
  testWatchdog();
  testPersistence();
  printf("\n%s: %d failure(s)\n", failures ? "FAILED" : "ALL PASSED", failures);
  return failures ? 1 : 0;
}
