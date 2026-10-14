// Settings.h — настраиваемые параметры (NVS) и сохранение позиций.
#pragma once
#include <Arduino.h>

// Все параметры, меняемые командами. Хранятся в NVS (команды save / load / defaults).
struct Settings {
  uint32_t magic;
  uint16_t version;
  uint16_t size;

  // --- Главный привод (ось Z головки) ---
  float zSpeed;       // мм/с, предел скорости (sp)
  float zAccel;       // мм/с², предел ускорения (za)
  float zKp;          // 1/с, коэффициент контура положения (zkp)
  float zDeadband;    // мм, зона нечувствительности (zdb)
  float zFF;          // доля упреждения по скорости поверхности (zff)
  float zGap;         // мм, отступ (sh): цель = дистанция − sh
  float zMax;         // мм, программный предел хода (zmax)
  float zMmPerRev;    // мм на оборот шестерни (zmmrev): π·m·z = π·0,5·20
  uint16_t zCurrent;  // мА RMS (zcur)
  uint8_t zStealth;   // 1 — stealthChop (zstealth)
  uint8_t zDirInvert; // 1 — инвертировать DIR (zdir)

  // --- Ультразвуковой датчик ---
  float usSoS;        // м/с, скорость звука (sos)
  float usScale;      // калибровочный множитель дистанции (uscal)
  float usOffset;     // мм, калибровочное смещение (usoff)
  float usMin;        // мм, минимум достоверной дистанции (usmin)
  float usMax;        // мм, максимум достоверной дистанции (usmax)
  float usTrackMax;   // мм, дальше этой дистанции слежение не опускает головку (ustrack)
  float usJump;       // мм, максимальный правдоподобный скачок между отсчётами (usjump)
  uint16_t usPeriodMs;  // мс, период запуска датчика (usper)
  uint16_t usTimeoutMs; // мс, нет достоверных отсчётов дольше — авария (ustimeout)
  float fAlpha;       // α-β фильтр: α (fa)
  float fBeta;        // α-β фильтр: β (fb)

  // --- Дозатор ---
  float dVel;         // тик/с, скорость толкателя (vel)
  float dAcc;         // тик/с², ускорение толкателя (acc)
  float dKp;          // ШИМ на тик ошибки (kp)
  float dKi;          // ШИМ на тик·с (ki)
  float dKd;          // ШИМ на тик/с ошибки скорости (kd)
  float dKv;          // ШИМ на тик/с заданной скорости, упреждение (kv)
  float dIMax;        // ограничение интегральной составляющей, ШИМ (imax)
  int16_t dMinDuty;   // компенсация трения покоя, ШИМ (minDuty)
  int16_t dMaxDuty;   // предел ШИМ (maxDuty)
  int32_t dEmax;      // тик, предел ошибки слежения — ограничитель болюса (emax)
  int32_t dDeadband;  // тик, зона удержания (ddb)
  int32_t dSkewMax;   // тик, допустимый перекос между моторами (skew)
  int32_t dRelief;    // тик, отвод толкателя после заклинивания (relief)
  uint16_t dJamMs;    // мс, время в пределе ошибки до аварии "заклинивание" (jamt)
  float dLead;        // мм, ход винта (dlead)
  float dMinMm;       // мм, программный предел хода толкателя снизу (dmin)
  float dMaxMm;       // мм, программный предел хода толкателя сверху (dmax)

  // --- Параметры печати ---
  float pSpeed;       // мм/с (speed)
  float pHeight;      // мм (height)
  float pWidth;       // мм (width)
  float pSyrDiam;     // мм, внутренний диаметр шприца (diameter)

  // --- Связь и телеметрия ---
  uint32_t baud;
  uint16_t tlmPeriodMs;
  uint8_t tlmMaster, tlmMillis, tlmPos, tlmVel, tlmHead, tlmPwm;

  uint32_t crc;  // всегда последним полем
};

extern Settings S;

void settingsDefaults(Settings &s);
bool settingsLoad(Settings &s);   // false — в NVS нет совместимой записи
bool settingsSave(Settings &s);

float printPlungerSpeedMmS(const Settings &s);  // скорость штока по w·h·v / (π·d²/4)
float dispTicksPerMm(const Settings &s);

// --- Позиции приводов ---
// Позиция пишется только в покое. На время движения запись помечается "грязной":
// если питание пропало в движении, после перезагрузки позиция не считается известной.
struct SavedPositions {
  int32_t zSteps;
  int32_t enc1, enc2;
  bool zValid, dValid;
};
void positionsLoad(SavedPositions &p);
void positionsMarkZDirty();
void positionsMarkDDirty();
void positionsSaveZIfChanged(int32_t steps);
void positionsSaveDIfChanged(int32_t enc1, int32_t enc2);
