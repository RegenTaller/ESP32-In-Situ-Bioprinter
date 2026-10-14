// Settings.cpp — значения по умолчанию, NVS (Preferences) с контрольной суммой.
#include "Settings.h"
#include "Config.h"
#include "Control.h"
#include <Preferences.h>
#include <math.h>
#include <stddef.h>

Settings S;

static const uint32_t SETTINGS_MAGIC = 0xB10C0002;
static const uint16_t SETTINGS_VERSION = 1;
static const char *NVS_NS = "bioprinter";
static Preferences prefs;

static uint32_t crc32(const uint8_t *p, size_t n) {
  uint32_t crc = 0xFFFFFFFFu;
  for (size_t i = 0; i < n; i++) {
    crc ^= p[i];
    for (int k = 0; k < 8; k++) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
  }
  return ~crc;
}

static uint32_t calcCrc(const Settings &s) {
  return crc32(reinterpret_cast<const uint8_t *>(&s), offsetof(Settings, crc));
}

float printPlungerSpeedMmS(const Settings &s) {
  const float area = (float)M_PI * s.pSyrDiam * s.pSyrDiam / 4.0f;  // мм²
  return s.pWidth * s.pHeight * s.pSpeed / area;                     // мм³/с ÷ мм² = мм/с
}

float dispTicksPerMm(const Settings &s) {
  return DISP_TICKS_PER_REV / s.dLead;
}

void settingsDefaults(Settings &s) {
  memset(&s, 0, sizeof(s));  // обнуляет и байты выравнивания — CRC воспроизводима
  s.magic = SETTINGS_MAGIC;
  s.version = SETTINGS_VERSION;
  s.size = sizeof(Settings);

  s.zSpeed = 20.0f;
  s.zAccel = 1000.0f;
  s.zKp = 15.0f;
  s.zDeadband = 0.05f;
  s.zFF = 0.5f;
  s.zGap = 0.0f;
  s.zMax = 82.0f;
  s.zMmPerRev = (float)M_PI * 10.0f;  // шестерня m = 0,5, z = 20 (ВКР, с. 33)
  s.zCurrent = 200;
  s.zStealth = 1;
  s.zDirInvert = 0;

  s.usSoS = 343.0f;
  s.usScale = 1.0f;
  s.usOffset = 0.0f;
  s.usMin = 20.0f;
  s.usMax = 400.0f;
  s.usTrackMax = 82.0f;
  s.usJump = 15.0f;
  s.usPeriodMs = 10;
  s.usTimeoutMs = 100;
  s.fAlpha = 0.25f;
  s.fBeta = 0.01f;

  s.dAcc = 30000.0f;
  s.dKp = 0.4f;
  s.dKi = 2.0f;
  s.dKd = 0.0f;
  s.dKv = 0.06f;
  s.dIMax = 300.0f;
  s.dMinDuty = 250;
  s.dMaxDuty = DISP_PWM_MAX;
  s.dEmax = 150;
  s.dDeadband = 4;
  s.dSkewMax = 1000;
  s.dRelief = 100;
  s.dJamMs = 1000;
  s.dLead = 1.0f;
  s.dMinMm = -60.0f;
  s.dMaxMm = 60.0f;

  s.pSpeed = 40.0f;
  s.pWidth = 0.4f;
  s.pHeight = 0.34f;
  s.pSyrDiam = 14.85f;
  s.dVel = printPlungerSpeedMmS(s) * dispTicksPerMm(s);

  s.baud = DEFAULT_BAUD;
  s.tlmPeriodMs = 100;
  s.tlmMaster = 1;
  s.crc = calcCrc(s);
}

bool settingsLoad(Settings &s) {
  Settings tmp;
  prefs.begin(NVS_NS, false);
  const size_t n = prefs.getBytes("cfg", &tmp, sizeof(tmp));
  prefs.end();
  if (n != sizeof(tmp) || tmp.magic != SETTINGS_MAGIC || tmp.version != SETTINGS_VERSION ||
      tmp.size != sizeof(Settings) || tmp.crc != calcCrc(tmp)) {
    return false;
  }
  s = tmp;
  return true;
}

bool settingsSave(Settings &s) {
  s.crc = calcCrc(s);
  gFlashBusy = true;
  prefs.begin(NVS_NS, false);
  const size_t n = prefs.putBytes("cfg", &s, sizeof(s));
  prefs.end();
  gFlashBusy = false;
  return n == sizeof(s);
}

// ------------------------------- позиции -------------------------------------
static bool zClean = false, dClean = false;
static int32_t zSaved = 0, d1Saved = 0, d2Saved = 0;

void positionsLoad(SavedPositions &p) {
  prefs.begin(NVS_NS, false);
  p.zValid = prefs.getUChar("zok", 0) == 1;
  p.zSteps = prefs.getInt("zpos", 0);
  p.dValid = prefs.getUChar("dok", 0) == 1;
  p.enc1 = prefs.getInt("d1", 0);
  p.enc2 = prefs.getInt("d2", 0);
  prefs.end();
  zClean = p.zValid;
  dClean = p.dValid;
  zSaved = p.zSteps;
  d1Saved = p.enc1;
  d2Saved = p.enc2;
}

static void writeFlag(const char *key, uint8_t v) {
  gFlashBusy = true;
  prefs.begin(NVS_NS, false);
  prefs.putUChar(key, v);
  prefs.end();
  gFlashBusy = false;
}

void positionsMarkZDirty() {
  if (!zClean) return;
  writeFlag("zok", 0);
  zClean = false;
}

void positionsMarkDDirty() {
  if (!dClean) return;
  writeFlag("dok", 0);
  dClean = false;
}

void positionsSaveZIfChanged(int32_t steps) {
  if (zClean && steps == zSaved) return;
  gFlashBusy = true;
  prefs.begin(NVS_NS, false);
  prefs.putInt("zpos", steps);
  prefs.putUChar("zok", 1);
  prefs.end();
  gFlashBusy = false;
  zClean = true;
  zSaved = steps;
}

void positionsSaveDIfChanged(int32_t enc1, int32_t enc2) {
  if (dClean && enc1 == d1Saved && enc2 == d2Saved) return;
  gFlashBusy = true;
  prefs.begin(NVS_NS, false);
  prefs.putInt("d1", enc1);
  prefs.putInt("d2", enc2);
  prefs.putUChar("dok", 1);
  prefs.end();
  gFlashBusy = false;
  dClean = true;
  d1Saved = enc1;
  d2Saved = enc2;
}
