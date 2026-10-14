// =============================================================================
// ESP32 In-Situ Bioprinter — рабочий орган in situ биопринтера, прошивка v0.2
//
// Главный привод (2× NEMA14 + TMC2130) держит постоянный зазор сопло — поверхность
// по ультразвуковому датчику HC-SR04. Дозатор (2× ДПТ N20 298:1 с энкодерами,
// TB6612FNG) толкает шток шприца винтом T5×1.
//
// Структура:
//   Config.h      — пины и константы
//   Settings.*    — параметры и позиции в NVS
//   Control.*     — задача управления (1 кГц), блокировка, аварии
//   HeadAxis.*    — ось Z: шаги в ISR, датчик, α-β фильтр, слежение, хоминг
//   Dispenser.*   — дозатор: энкодеры, планировщик, ПИД, ограничитель болюса
//   Commands.*    — команды COM/Bluetooth, телеметрия
// Команды: ../COMMANDS.md. Изменения относительно v0.1 и порядок запуска: ../README.md
// =============================================================================
#include "Config.h"
#include "Settings.h"
#include "Control.h"
#include "HeadAxis.h"
#include "Dispenser.h"
#include "Commands.h"
#include "esp_system.h"

static const char *resetReasonName() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON: return "power-on";
    case ESP_RST_SW: return "software restart";
    case ESP_RST_PANIC: return "PANIC (firmware crash)";
    case ESP_RST_INT_WDT: return "interrupt watchdog";
    case ESP_RST_TASK_WDT: return "task watchdog";
    case ESP_RST_WDT: return "other watchdog";
    case ESP_RST_BROWNOUT: return "BROWNOUT (supply voltage dip)";
    case ESP_RST_EXT: return "external reset";
    default: return "other";
  }
}

void setup() {
  const bool loaded = settingsLoad(S);
  if (!loaded) settingsDefaults(S);
  Serial.begin(S.baud);
  delay(100);
  outPrintf("\r\n=== InSitu-Bioprinter firmware %s ===\r\n", FW_VERSION);
  outPrintf("reset reason: %s\r\n", resetReasonName());
  outPrintf("settings: %s\r\n", loaded ? "loaded from NVS" : "defaults (NVS empty or from another version)");

  controlInit();
  SavedPositions p;
  positionsLoad(p);
  dispBegin(p);
  headBegin(p);
  outPrintf("Z: %s\r\n", p.zValid ? "position restored from NVS"
                                  : "NOT homed -> zhome / zset<mm> / zhomehard before track1 or zgo");
  outPrintf("dispenser: %s\r\n", p.dValid ? "position restored from NVS" : "position = 0");

  commandsBegin();
  controlStart();
  outWrite("ready. Tracking is OFF until 'track1'. 'help' - commands, 'status' - state\r\n");
}

void loop() {
  commandsPoll();
  eventsPoll();
  telemetryPoll();
  persistencePoll();
  delay(1);
}
