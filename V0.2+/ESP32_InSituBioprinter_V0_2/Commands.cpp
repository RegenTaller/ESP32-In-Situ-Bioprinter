// Commands.cpp — разбор команд, таблица параметров, телеметрия.
//
// Формат: <имя><число> | <имя> <число> | <имя>=<число>; несколько команд через ';'.
// Имя — ведущие буквы, сравнивается целиком и без учёта регистра. В v0.1 команды
// искались подстрокой (indexOf), поэтому "speed40" срабатывал ещё и как "sp".
// Команда без числа выводит текущее значение параметра.
#include "Commands.h"
#include "Config.h"
#include "Settings.h"
#include "Control.h"
#include "HeadAxis.h"
#include "Dispenser.h"
#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <strings.h>

#if BT_ENABLED
#include <BluetoothSerial.h>
static BluetoothSerial SerialBT;
static bool btOk = false;
#endif

// ------------------------------- вывод ---------------------------------------
void outWrite(const char *s) {
  Serial.print(s);
#if BT_ENABLED
  if (btOk && SerialBT.hasClient()) SerialBT.print(s);
#endif
}

void outPrintf(const char *fmt, ...) {
  char buf[256];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  outWrite(buf);
}

// ------------------------------- контекст ------------------------------------
struct Ctx {
  const char *name = "";
  float v = 0.0f;
  bool hasV = false;
  char out[384];
  size_t len = 0;
  Ctx() { out[0] = 0; }
  void say(const char *fmt, ...) __attribute__((format(printf, 2, 3)));
};

void Ctx::say(const char *fmt, ...) {
  if (len >= sizeof(out) - 1) return;
  va_list ap;
  va_start(ap, fmt);
  const int n = vsnprintf(out + len, sizeof(out) - len, fmt, ap);
  va_end(ap);
  if (n > 0) len = (len + (size_t)n < sizeof(out) - 1) ? len + (size_t)n : sizeof(out) - 1;
}

static bool is(const Ctx &c, const char *n) { return strcasecmp(c.name, n) == 0; }

// Проверка перед движением: нет аварий; пометить позиции в NVS как "в движении".
// Запись во flash — только пока ось Z стоит (иначе пауза записи сорвала бы слежение).
static bool prepareMotion(Ctx &c, bool markZ, bool markD) {
  uint32_t faults;
  bool zIdle;
  {
    Lock l;
    faults = gFaults;
    zIdle = headIsIdle();
  }
  if (faults) {
    char fb[48];
    c.say("ERR %s: fault active (%s) -> 'clear'", c.name, faultNames(faults, fb, sizeof(fb)));
    return false;
  }
  if (zIdle) {
    if (markZ) positionsMarkZDirty();
    if (markD) positionsMarkDDirty();
  }
  return true;
}

static bool bothIdle() {
  return headIsIdle() && H.mode != Z_TRACK && dispIsIdle();
}

// ------------------------------- параметры -----------------------------------
enum PType : uint8_t { PT_F32, PT_U8, PT_U16, PT_I16, PT_I32 };
enum : uint8_t { PF_ZIDLE = 1, PF_DIDLE = 2 };
typedef void (*ApplyFn)(Ctx &c);

struct ParamDef {
  const char *name;
  PType type;
  void *ptr;
  float lo, hi;
  const char *unit;
  ApplyFn apply;
  uint8_t flags;
};

static void apZLimits(Ctx &) { headUpdateLimits(); }
static void apZDriver(Ctx &) { headApplyDriverSettings(); }
static void apZDir(Ctx &) { headUpdateDir(); }
static void apUsPeriod(Ctx &) { headSetTrigPeriod(); }
static void apDispVel(Ctx &c) { c.say(" (%.4f mm/s)", S.dVel / dispTicksPerMm(S)); }
static void apDLimits(Ctx &c) {
  if (S.dMinMm >= S.dMaxMm) c.say(" WARN dmin >= dmax: dispenser moves will be rejected");
}
static void apPrint(Ctx &c) {
  const float mmS = printPlungerSpeedMmS(S);
  S.dVel = fminf(mmS * dispTicksPerMm(S), DISP_VEL_LIMIT);
  const float flow = mmS * (float)M_PI * S.pSyrDiam * S.pSyrDiam / 4.0f;  // мм³/с = мкл/с
  c.say(" -> plunger %.4f mm/s, vel=%.0f tick/s, flow %.2f ul/s", mmS, S.dVel, flow);
}

static const ParamDef PARAMS[] = {
    // Главный привод (ось Z)
    {"sp", PT_F32, &S.zSpeed, 1, 150, "mm/s", nullptr, 0},
    {"za", PT_F32, &S.zAccel, 10, 5000, "mm/s2", nullptr, 0},
    {"zkp", PT_F32, &S.zKp, 1, 100, "1/s", nullptr, 0},
    {"zdb", PT_F32, &S.zDeadband, 0, 2, "mm", nullptr, 0},
    {"zff", PT_F32, &S.zFF, 0, 1.5f, "", nullptr, 0},
    {"sh", PT_F32, &S.zGap, 0, 200, "mm", nullptr, 0},
    {"zmax", PT_F32, &S.zMax, 1, 300, "mm", apZLimits, PF_ZIDLE},
    {"zmmrev", PT_F32, &S.zMmPerRev, 5, 200, "mm/rev", apZLimits, PF_ZIDLE},
    {"zcur", PT_U16, &S.zCurrent, 50, 1200, "mA", apZDriver, 0},
    {"zstealth", PT_U8, &S.zStealth, 0, 1, "", apZDriver, 0},
    {"zdir", PT_U8, &S.zDirInvert, 0, 1, "", apZDir, PF_ZIDLE},
    // Датчик дистанции
    {"sos", PT_F32, &S.usSoS, 300, 360, "m/s", nullptr, 0},
    {"uscal", PT_F32, &S.usScale, 0.5f, 1.5f, "", nullptr, 0},
    {"usoff", PT_F32, &S.usOffset, -100, 100, "mm", nullptr, 0},
    {"usmin", PT_F32, &S.usMin, 2, 200, "mm", nullptr, 0},
    {"usmax", PT_F32, &S.usMax, 20, 5000, "mm", nullptr, 0},
    {"ustrack", PT_F32, &S.usTrackMax, 5, 500, "mm", nullptr, 0},
    {"usjump", PT_F32, &S.usJump, 0.5f, 500, "mm", nullptr, 0},
    {"usper", PT_U16, &S.usPeriodMs, 10, 200, "ms", apUsPeriod, 0},
    {"ustimeout", PT_U16, &S.usTimeoutMs, 20, 5000, "ms", nullptr, 0},
    {"fa", PT_F32, &S.fAlpha, 0.01f, 1, "", nullptr, 0},
    {"fb", PT_F32, &S.fBeta, 0, 1, "", nullptr, 0},
    // Дозатор
    {"vel", PT_F32, &S.dVel, 0, DISP_VEL_LIMIT, "tick/s", apDispVel, 0},
    {"acc", PT_F32, &S.dAcc, 0, 40000, "tick/s2", nullptr, 0},
    {"kp", PT_F32, &S.dKp, 0, 20, "pwm/tick", nullptr, 0},
    {"ki", PT_F32, &S.dKi, 0, 2000, "pwm/(tick*s)", nullptr, 0},
    {"kd", PT_F32, &S.dKd, 0, 20, "pwm/(tick/s)", nullptr, 0},
    {"kv", PT_F32, &S.dKv, 0, 2, "pwm/(tick/s)", nullptr, 0},
    {"imax", PT_F32, &S.dIMax, 0, DISP_PWM_MAX, "pwm", nullptr, 0},
    {"minDuty", PT_I16, &S.dMinDuty, 0, DISP_PWM_MAX, "pwm", nullptr, 0},
    {"maxDuty", PT_I16, &S.dMaxDuty, 1, DISP_PWM_MAX, "pwm", nullptr, 0},
    {"emax", PT_I32, &S.dEmax, 5, 200000, "tick", nullptr, 0},
    {"ddb", PT_I32, &S.dDeadband, 0, 500, "tick", nullptr, 0},
    {"skew", PT_I32, &S.dSkewMax, 10, 200000, "tick", nullptr, 0},
    {"jamt", PT_U16, &S.dJamMs, 50, 60000, "ms", nullptr, 0},
    {"relief", PT_I32, &S.dRelief, 0, 20000, "tick", nullptr, 0},
    {"dlead", PT_F32, &S.dLead, 0.1f, 20, "mm", nullptr, PF_DIDLE},
    {"dmin", PT_F32, &S.dMinMm, -500, 500, "mm", apDLimits, 0},
    {"dmax", PT_F32, &S.dMaxMm, -500, 500, "mm", apDLimits, 0},
    // Параметры печати
    {"speed", PT_F32, &S.pSpeed, 0.1f, 500, "mm/s", apPrint, 0},
    {"height", PT_F32, &S.pHeight, 0.01f, 10, "mm", apPrint, 0},
    {"width", PT_F32, &S.pWidth, 0.01f, 20, "mm", apPrint, 0},
    {"diameter", PT_F32, &S.pSyrDiam, 1, 60, "mm", apPrint, 0},
    // Телеметрия
    {"period", PT_U16, &S.tlmPeriodMs, 10, 60000, "ms", nullptr, 0},
};

static float readParam(const ParamDef &p) {
  switch (p.type) {
    case PT_F32: return *(float *)p.ptr;
    case PT_U8: return *(uint8_t *)p.ptr;
    case PT_U16: return *(uint16_t *)p.ptr;
    case PT_I16: return *(int16_t *)p.ptr;
    case PT_I32: return (float)*(int32_t *)p.ptr;
  }
  return 0.0f;
}

static void writeParam(const ParamDef &p, float v) {
  switch (p.type) {
    case PT_F32: *(float *)p.ptr = v; break;
    case PT_U8: *(uint8_t *)p.ptr = (uint8_t)lroundf(v); break;
    case PT_U16: *(uint16_t *)p.ptr = (uint16_t)lroundf(v); break;
    case PT_I16: *(int16_t *)p.ptr = (int16_t)lroundf(v); break;
    case PT_I32: *(int32_t *)p.ptr = (int32_t)lroundf(v); break;
  }
}

static void handleParam(Ctx &c, const ParamDef &p) {
  Lock l;
  if (!c.hasV) {
    c.say("%s=%.6g %s", p.name, readParam(p), p.unit);
    return;
  }
  if (!(c.v >= p.lo && c.v <= p.hi)) {
    c.say("ERR %s: range %.6g..%.6g %s", p.name, p.lo, p.hi, p.unit);
    return;
  }
  if ((p.flags & PF_ZIDLE) && !headIsIdle()) {
    c.say("ERR %s: stop Z motion first", p.name);
    return;
  }
  if ((p.flags & PF_DIDLE) && !dispIsIdle()) {
    c.say("ERR %s: stop dispenser first", p.name);
    return;
  }
  writeParam(p, c.v);
  c.say("OK %s=%.6g %s", p.name, readParam(p), p.unit);
  if (p.apply) p.apply(c);
}

// ------------------------------- действия ------------------------------------
static void printHelp();
static void printStatus();

static void applyAllSettings() {  // под Lock, после load / defaults
  headApplyDriverSettings();
  headUpdateDir();
  headUpdateLimits();
  headSetTrigPeriod();
}

static void actHelp(Ctx &) { printHelp(); }
static void actStatus(Ctx &) { printStatus(); }
static void actVer(Ctx &c) { c.say("fw %s, build %s %s", FW_VERSION, __DATE__, __TIME__); }

static void actStop(Ctx &c) {
  Lock l;
  faultRaise(F_ESTOP);
  c.say("OK STOP: all motion halted. 'clear' to resume");
}

static void actClear(Ctx &c) {
  Lock l;
  faultClear();
  c.say("OK faults cleared. Z mode %s, Z %s", zModeName(H.mode), H.homed ? "homed" : "NOT homed");
}

static void actSave(Ctx &c) {
  Settings copy;
  {
    Lock l;
    if (!bothIdle()) {
      c.say("ERR save: stop Z and dispenser first (flash write pauses motion)");
      return;
    }
    copy = S;
  }
  c.say(settingsSave(copy) ? "OK settings saved" : "ERR save: NVS write failed");
}

static void actLoad(Ctx &c) {
  Settings tmp;
  if (!settingsLoad(tmp)) {
    c.say("ERR load: no valid settings in NVS");
    return;
  }
  Lock l;
  if (!bothIdle()) {
    c.say("ERR load: stop Z and dispenser first");
    return;
  }
  const uint32_t baud = S.baud;
  S = tmp;
  S.baud = baud;  // скорость порта меняется только командой COM или после перезагрузки
  applyAllSettings();
  c.say("OK settings loaded");
}

static void actDefaults(Ctx &c) {
  Settings tmp;
  settingsDefaults(tmp);
  Lock l;
  if (!bothIdle()) {
    c.say("ERR defaults: stop Z and dispenser first");
    return;
  }
  const uint32_t baud = S.baud;
  S = tmp;
  S.baud = baud;
  applyAllSettings();
  c.say("OK defaults applied (RAM). 'save' to store");
}

static void actReboot(Ctx &) {
  outWrite("OK reboot\r\n");
  Serial.flush();
  delay(100);
  ESP.restart();
}

static void actCom(Ctx &c) {
  static const uint32_t bauds[] = {1200,  2400,   4800,   9600,   19200,  31250,  38400, 57600,
                                   74880, 115200, 230400, 250000, 460800, 500000, 921600};
  if (!c.hasV) {
    c.say("COM=%lu", (unsigned long)S.baud);
    return;
  }
  const uint32_t b = (uint32_t)lroundf(c.v);
  bool supported = false;
  for (uint32_t x : bauds) supported |= (x == b);
  if (!supported) {
    c.say("ERR COM: unsupported baud. Use 1200 2400 4800 9600 19200 31250 38400 57600 74880 115200 230400 250000 460800 500000 921600");
    return;
  }
  outPrintf("OK COM=%lu (switching now; 'save' keeps it after reboot)\r\n", (unsigned long)b);
  Serial.flush();
  Serial.updateBaudRate(b);
  Lock l;
  S.baud = b;
}

static void toggle(Ctx &c, uint8_t &flag, const char *label) {
  Lock l;
  flag = c.hasV ? (c.v != 0.0f) : !flag;
  c.say("OK %s=%u", label, flag);
}
static void actPD(Ctx &c) {
  toggle(c, S.tlmMaster, "PD");
  c.say(S.tlmMaster ? " ///////////Data:///////////" : " /////Stop Data sending/////");
}
static void actMillis(Ctx &c) { toggle(c, S.tlmMillis, "millis"); }
static void actPos(Ctx &c) { toggle(c, S.tlmPos, "POS"); }
static void actVels(Ctx &c) { toggle(c, S.tlmVel, "VELS"); }
static void actHead(Ctx &c) { toggle(c, S.tlmHead, "Head"); }
static void actPwm(Ctx &c) { toggle(c, S.tlmPwm, "PWM"); }

// ---- ось Z ----
static void actTrack(Ctx &c) {
  if (!c.hasV) {
    Lock l;
    c.say("track=%d (Z mode %s)", H.mode == Z_TRACK ? 1 : 0, zModeName(H.mode));
    return;
  }
  const bool on = c.v != 0.0f;
  if (on && !prepareMotion(c, true, true)) return;
  Lock l;
  const char *err = headCmdTrack(on, millis());
  if (err) c.say("ERR track: %s", err);
  else c.say("OK track=%d, gap sh=%.3f mm", on ? 1 : 0, S.zGap);
}

static void actRetract(Ctx &c) {
  Lock l;
  const char *err = headCmdRetract();
  if (err) c.say("ERR retract: %s", err);
  else c.say("OK retract -> Z 0 mm");
}

static void actZMove(Ctx &c) {
  if (!c.hasV) {
    Lock l;
    c.say("Z pos=%.3f mm, target=%.3f mm, mode %s", headPosMm(), H.targetMm, zModeName(H.mode));
    return;
  }
  if (!prepareMotion(c, true, false)) return;
  Lock l;
  const bool jog = is(c, "zjog");
  const char *err = jog ? headCmdJog(c.v) : headCmdMove(c.v);
  if (err) c.say("ERR %s: %s", c.name, err);
  else c.say("OK %s -> Z target %.3f mm%s", c.name, H.moveTargetMm, H.unhomedJog ? " (NOT homed: limits off, 5 mm/s)" : "");
}

static void actZSet(Ctx &c) {
  const float mm = is(c, "zhome") ? 0.0f : c.v;
  if (is(c, "zset") && !c.hasV) {
    c.say("ERR zset: value required (current position, mm)");
    return;
  }
  Lock l;
  const char *err = headCmdSetPos(mm);
  if (err) c.say("ERR %s: %s", c.name, err);
  else c.say("OK Z position set to %.3f mm (homed)", mm);
}

static void actZHomeHard(Ctx &c) {
  if (!prepareMotion(c, true, false)) return;
  Lock l;
  const char *err = headCmdHomeHard();
  if (err) c.say("ERR zhomehard: %s", err);
  else c.say("OK zhomehard: moving UP %.0f mm at %.0f mm/s, reduced current", S.zMax + Z_HOME_EXTRA_MM, Z_HOME_SPEED);
}

// ---- дозатор ----
static void actDispMove(Ctx &c) {
  if (!c.hasV) {
    Lock l;
    const float tpm = dispTicksPerMm(S);
    c.say("dispenser target=%ld tick (%.4f mm), ref=%.1f, enc=%ld/%ld", (long)D.target, D.target / tpm,
          D.ref, (long)D.enc[0], (long)D.enc[1]);
    return;
  }
  if (!prepareMotion(c, false, true)) return;
  Lock l;
  const float tpm = dispTicksPerMm(S);
  int64_t t;
  if (is(c, "tpos")) t = llroundf(c.v);
  else if (is(c, "tposMMS")) t = llroundf(c.v * tpm);
  else if (is(c, "tposob")) t = llroundf(c.v * DISP_TICKS_PER_REV);
  else if (is(c, "Z")) t = (int64_t)D.target + llroundf(c.v * tpm);
  else if (is(c, "Zob")) t = (int64_t)D.target + llroundf(c.v * DISP_TICKS_PER_REV);
  else t = (int64_t)D.target + llroundf(c.v);  // Ztick
  const char *err = dispCmdTarget(t);
  if (err) c.say("ERR %s: %s", c.name, err);
  else c.say("OK %s -> target %ld tick (%.4f mm) at %.0f tick/s", c.name, (long)D.target, D.target / tpm, S.dVel);
}

static void actExt(Ctx &c) {
  if (!c.hasV) {
    Lock l;
    c.say("ext=%d", D.ext);
    return;
  }
  const int dir = (c.v > 0.0f) ? 1 : (c.v < 0.0f ? -1 : 0);
  if (dir != 0 && !prepareMotion(c, false, true)) return;
  Lock l;
  const char *err = dispCmdExt(dir);
  if (err) c.say("ERR ext: %s", err);
  else if (dir) c.say("OK ext=%d at %.0f tick/s (%.4f mm/s) until 'ext0' or d%s", dir, S.dVel, S.dVel / dispTicksPerMm(S), dir > 0 ? "max" : "min");
  else c.say("OK ext=0 (decelerating)");
}

static void actDZero(Ctx &c) {
  Lock l;
  const char *err = dispCmdZero();
  if (err) c.say("ERR dzero: %s", err);
  else c.say("OK dispenser position = 0");
}

static void actSavePos(Ctx &c) {
  int32_t zp, e1, e2;
  bool homed;
  {
    Lock l;
    if (!bothIdle()) {
      c.say("ERR Savepos: stop Z and dispenser first");
      return;
    }
    zp = headPosSteps();
    homed = H.homed;
    e1 = D.enc[0];
    e2 = D.enc[1];
  }
  if (homed) positionsSaveZIfChanged(zp);
  positionsSaveDIfChanged(e1, e2);
  c.say("OK positions saved (dispenser %ld/%ld%s)", (long)e1, (long)e2, homed ? ", Z" : ", Z not homed - skipped");
}

static void actVelMMS(Ctx &c) {
  Lock l;
  const float tpm = dispTicksPerMm(S);
  if (!c.hasV) {
    c.say("velMMS=%.5f mm/s (vel=%.0f tick/s)", S.dVel / tpm, S.dVel);
    return;
  }
  const float maxMm = DISP_VEL_LIMIT / tpm;
  if (!(c.v >= 0.0f && c.v <= maxMm)) {
    c.say("ERR velMMS: range 0..%.3f mm/s", maxMm);
    return;
  }
  S.dVel = c.v * tpm;
  c.say("OK velMMS=%.5f mm/s (vel=%.0f tick/s)", c.v, S.dVel);
}

static void actRemoved(Ctx &c) {
  c.say("ERR %s: removed in v0.2 (PWM reduction replaced by following-error limit 'emax' and jam detector 'jamt')", c.name);
}

typedef void (*ActionFn)(Ctx &c);
struct ActionDef {
  const char *name;
  ActionFn fn;
};

static const ActionDef ACTIONS[] = {
    {"help", actHelp},       {"?", actHelp},          {"status", actStatus},      {"ver", actVer},
    {"stop", actStop},       {"!", actStop},          {"clear", actClear},        {"save", actSave},
    {"load", actLoad},       {"defaults", actDefaults}, {"reboot", actReboot},    {"COM", actCom},
    {"PD", actPD},           {"millis", actMillis},   {"POS", actPos},            {"VELS", actVels},
    {"Head", actHead},       {"PWM", actPwm},         {"track", actTrack},        {"retract", actRetract},
    {"zgo", actZMove},       {"zjog", actZMove},      {"zhome", actZSet},         {"zset", actZSet},
    {"zhomehard", actZHomeHard}, {"tpos", actDispMove}, {"tposMMS", actDispMove}, {"tposob", actDispMove},
    {"Z", actDispMove},      {"Zob", actDispMove},    {"Ztick", actDispMove},     {"ext", actExt},
    {"dzero", actDZero},     {"Savepos", actSavePos}, {"velMMS", actVelMMS},      {"rk", actRemoved},
    {"rconst", actRemoved},
};

// ------------------------------- разбор строки -------------------------------
static void runCommand(char *tok) {
  while (*tok == ' ' || *tok == '\t') tok++;
  char *end = tok + strlen(tok);
  while (end > tok && (end[-1] == ' ' || end[-1] == '\t')) *--end = 0;
  if (!*tok) return;

  char name[24];
  size_t n = 0;
  if (*tok == '?' || *tok == '!') {
    name[n++] = *tok++;
  } else {
    while (isalpha((unsigned char)*tok) || *tok == '_') {
      if (n < sizeof(name) - 1) name[n++] = *tok;
      tok++;
    }
  }
  name[n] = 0;
  if (!n) {
    outPrintf("ERR bad command '%s' ('help' - list)\r\n", tok);
    return;
  }
  while (*tok == ' ' || *tok == '=' || *tok == ':') tok++;

  Ctx c;
  c.name = name;
  if (*tok) {
    char *e = nullptr;
    const float v = strtof(tok, &e);
    while (e && (*e == ' ' || *e == '\t')) e++;
    if (e == tok || (e && *e) || !isfinite(v)) {
      outPrintf("ERR %s: bad value '%s'\r\n", name, tok);
      return;
    }
    c.v = v;
    c.hasV = true;
  }

  bool found = false;
  for (const ActionDef &a : ACTIONS) {
    if (strcasecmp(a.name, name) == 0) {
      c.name = a.name;
      a.fn(c);
      found = true;
      break;
    }
  }
  if (!found) {
    for (const ParamDef &p : PARAMS) {
      if (strcasecmp(p.name, name) == 0) {
        c.name = p.name;
        handleParam(c, p);
        found = true;
        break;
      }
    }
  }
  if (!found) c.say("ERR unknown command '%s' ('help' - list)", name);
  if (c.len) {
    outWrite(c.out);
    outWrite("\r\n");
  }
}

static void processLine(char *line) {
  outPrintf("> %s\r\n", line);
  char *save = nullptr;
  for (char *tok = strtok_r(line, ";", &save); tok; tok = strtok_r(nullptr, ";", &save)) runCommand(tok);
}

// Неблокирующий построчный приём. В v0.1 readString() ждал таймаут 1 с,
// и всё это время ПИД дозатора и слежение головки не обновлялись.
struct LineReader {
  char buf[160];
  size_t len = 0;
  uint32_t lastRxMs = 0;
  bool overflow = false;
};
static LineReader rxSerial;
#if BT_ENABLED
static LineReader rxBt;
#endif

static void finishLine(LineReader &r) {
  if (r.overflow) outWrite("ERR line too long (max 159 chars)\r\n");
  else if (r.len) {
    r.buf[r.len] = 0;
    processLine(r.buf);
  }
  r.len = 0;
  r.overflow = false;
}

static void pollStream(Stream &s, LineReader &r) {
  while (s.available() > 0) {
    const int ch = s.read();
    if (ch < 0) break;
    r.lastRxMs = millis();
    if (ch == '\n' || ch == '\r') {
      finishLine(r);
      continue;
    }
    if (r.len < sizeof(r.buf) - 1) r.buf[r.len++] = (char)ch;
    else r.overflow = true;
  }
  // Терминалы без перевода строки: строка завершается паузой LINE_IDLE_MS
  if ((r.len || r.overflow) && (uint32_t)(millis() - r.lastRxMs) > LINE_IDLE_MS) finishLine(r);
}

void commandsBegin() {
#if BT_ENABLED
  btOk = SerialBT.begin(BT_DEVICE_NAME);
  outPrintf("Bluetooth: %s\r\n", btOk ? BT_DEVICE_NAME : "FAILED to start");
#endif
}

void commandsPoll() {
  pollStream(Serial, rxSerial);
#if BT_ENABLED
  if (btOk) pollStream(SerialBT, rxBt);
#endif
}

// ------------------------------- события -------------------------------------
void eventsPoll() {
  uint32_t ev;
  {
    Lock l;
    ev = gEvents;
    gEvents = 0;
  }
  if (!ev) return;
  const uint32_t f = ev >> EV_FAULT_SHIFT;
  if (f & F_SENSOR) outPrintf("FAULT SENSOR: no valid distance for >%u ms -> Z retract, dispenser stop. 'clear' to reset\r\n", S.usTimeoutMs);
  if (f & F_JAM) outPrintf("FAULT JAM: dispenser lagging at emax for >%u ms -> relief %ld tick. Check nozzle/syringe, then 'clear'\r\n", S.dJamMs, (long)S.dRelief);
  if (f & F_SKEW) outPrintf("FAULT SKEW: |enc1-enc2| > %ld tick -> dispenser braked. Check pusher mechanics, 'dzero' + 'clear'\r\n", (long)S.dSkewMax);
  if (f & F_ESTOP) outWrite("STOP: all motion halted\r\n");
  if (f & F_WDT) outWrite("FAULT WDT: control loop stalled >50 ms -> all motors stopped. 'clear' to reset\r\n");
  if (ev & EV_Z_DONE) outWrite("EVT Z move done\r\n");
  if (ev & EV_RETRACT_DONE) outWrite("EVT Z retracted to 0 mm\r\n");
  if (ev & EV_HOME_DONE) outWrite("EVT Z homed (zhomehard): position = 0 mm\r\n");
  if (ev & EV_D_DONE) outWrite("EVT dispenser at target\r\n");
  if (ev & EV_D_LIMIT) outWrite("EVT dispenser reached dmin/dmax: ext stopped\r\n");
  if (ev & EV_REACQUIRE) outWrite("WARN sensor: distance jump confirmed, filter re-initialised\r\n");
  if (ev & EV_Z_SATURATED) outWrite("WARN track: surface closer than sh - head at upper limit (0 mm)\r\n");
}

// ------------------------------- телеметрия ----------------------------------
void telemetryPoll() {
  static uint32_t last = 0;
  const uint32_t now = millis();
  if (!S.tlmMaster || (uint32_t)(now - last) < S.tlmPeriodMs) return;
  last = now;
  Snapshot s;
  {
    Lock l;
    s = gSnap;
  }
  char ms[16] = "";
  if (S.tlmMillis) snprintf(ms, sizeof(ms), "%lu,", (unsigned long)s.ms);
  if (S.tlmPos) outPrintf("POS,%s%.1f,%ld,%ld,%ld\r\n", ms, s.dRef, (long)s.enc1, (long)s.enc2, (long)s.dTarget);
  if (S.tlmVel) outPrintf("VEL,%s%.1f,%.1f,%.1f\r\n", ms, s.vRef, s.v1, s.v2);
  if (S.tlmHead) outPrintf("HEAD,%s%.2f,%.2f,%.3f,%.3f,%d\r\n", ms, s.distRaw, s.distEst, s.zTarget, s.zPos, s.distFresh ? 1 : 0);
  if (S.tlmPwm) outPrintf("PWM,%s%d,%d,%.1f,%.1f\r\n", ms, s.u1, s.u2, s.i1, s.i2);
}

// ------------------------------- status / help -------------------------------
static void printStatus() {
  Snapshot s;
  Settings st;
  bool homedNvs;
  {
    Lock l;
    s = gSnap;
    st = S;
    homedNvs = H.homedFromNvs;
  }
  char fb[48];
  const float tpm = dispTicksPerMm(st);
  const float mmS = printPlungerSpeedMmS(st);
  outPrintf("=== InSitu-Bioprinter fw %s, uptime %lu s ===\r\n", FW_VERSION, (unsigned long)(s.ms / 1000));
  outPrintf("faults: %s\r\n", faultNames(s.faults, fb, sizeof(fb)));
  outPrintf("Z: mode=%s homed=%s pos=%.3f target=%.3f mm v=%.2f mm/s limits 0..%.1f mm\r\n", zModeName(s.zMode),
            s.zHomed ? (homedNvs ? "yes(NVS)" : "yes") : "NO", s.zPos, s.zTarget, s.zVel, st.zMax);
  outPrintf("  sp=%.1f za=%.0f zkp=%.1f zdb=%.3f zff=%.2f sh=%.3f zcur=%u zstealth=%u zmmrev=%.4f\r\n", st.zSpeed,
            st.zAccel, st.zKp, st.zDeadband, st.zFF, st.zGap, st.zCurrent, st.zStealth, st.zMmPerRev);
  outPrintf("sensor: raw=%.2f est=%.2f mm v=%.1f mm/s fresh=%d valid/invalid/rejected=%lu/%lu/%lu\r\n", s.distRaw,
            s.distEst, s.distVel, s.distFresh ? 1 : 0, (unsigned long)s.usValid, (unsigned long)s.usInvalid,
            (unsigned long)s.usReject);
  outPrintf("  sos=%.1f uscal=%.4f usoff=%.2f range %.0f..%.0f ustrack=%.1f usjump=%.1f usper=%u ustimeout=%u fa=%.3f fb=%.4f\r\n",
            st.usSoS, st.usScale, st.usOffset, st.usMin, st.usMax, st.usTrackMax, st.usJump, st.usPeriodMs,
            st.usTimeoutMs, st.fAlpha, st.fBeta);
  outPrintf("dispenser: %s ref=%.1f target=%ld enc=%ld/%ld vref=%.1f v=%.1f/%.1f tick/s u=%d/%d ext=%d%s\r\n",
            s.dEnabled ? (s.dMoving ? "MOVING" : "HOLD") : "BRAKED", s.dRef, (long)s.dTarget, (long)s.enc1,
            (long)s.enc2, s.vRef, s.v1, s.v2, s.u1, s.u2, s.dExt, s.dRelief ? " RELIEF" : "");
  outPrintf("  vel=%.0f tick/s (%.5f mm/s) acc=%.0f kp=%.3f ki=%.3f kd=%.4f kv=%.4f imax=%.0f minDuty=%d maxDuty=%d\r\n",
            st.dVel, st.dVel / tpm, st.dAcc, st.dKp, st.dKi, st.dKd, st.dKv, st.dIMax, st.dMinDuty, st.dMaxDuty);
  outPrintf("  emax=%ld ddb=%ld skew=%ld jamt=%u relief=%ld dlead=%.3f limits %.1f..%.1f mm\r\n", (long)st.dEmax,
            (long)st.dDeadband, (long)st.dSkewMax, st.dJamMs, (long)st.dRelief, st.dLead, st.dMinMm, st.dMaxMm);
  outPrintf("print: speed=%.2f mm/s width=%.3f height=%.3f diameter=%.2f -> plunger %.5f mm/s, flow %.2f ul/s\r\n",
            st.pSpeed, st.pWidth, st.pHeight, st.pSyrDiam, mmS, mmS * (float)M_PI * st.pSyrDiam * st.pSyrDiam / 4.0f);
  outPrintf("telemetry: PD=%u period=%u ms millis=%u POS=%u VELS=%u Head=%u PWM=%u, COM %lu, BT %s\r\n", st.tlmMaster,
            st.tlmPeriodMs, st.tlmMillis, st.tlmPos, st.tlmVel, st.tlmHead, st.tlmPwm, (unsigned long)st.baud,
#if BT_ENABLED
            btOk ? (SerialBT.hasClient() ? "connected" : "waiting") : "off"
#else
            "disabled"
#endif
  );
}

static const char HELP_TEXT[] =
    "Commands: <name><value> | <name> <value> | <name>=<value>; several per line separated by ';'.\r\n"
    "Name without value prints the current value. Full description: V0.2+/COMMANDS.md\r\n"
    "SYSTEM : help ? status ver stop ! clear save load defaults reboot COM<baud>\r\n"
    "TELEM  : PD millis POS VELS Head PWM [0|1] period<ms>\r\n"
    "Z AXIS : track<1|0> retract zgo<mm> zjog<mm> zhome zset<mm> zhomehard\r\n"
    "         sp za zkp zdb zff sh zmax zmmrev zcur zstealth zdir\r\n"
    "SENSOR : sos uscal usoff usmin usmax ustrack usjump usper ustimeout fa fb\r\n"
    "DISP   : tpos<tick> tposMMS<mm> tposob<rev> Z<mm> Zob<rev> Ztick<tick> ext<1|0|-1> dzero Savepos\r\n"
    "         vel velMMS acc kp ki kd kv imax minDuty maxDuty emax ddb skew jamt relief dlead dmin dmax\r\n"
    "PRINT  : speed<mm/s> height<mm> width<mm> diameter<mm>\r\n";

static void printHelp() { outWrite(HELP_TEXT); }
