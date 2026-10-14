// =============================================================================
// ESP32 In-Situ Bioprinter — прошивка v0.2
// Config.h — пины, константы механики, тайминги.
// Схема подключения: Scheme.png в корне репозитория (рис. 14 ВКР).
// =============================================================================
#pragma once
#include <Arduino.h>

#define FW_VERSION "0.2.0"

// ----------------------------- Связь -----------------------------------------
#define BT_ENABLED 1                        // 0 — собрать без Bluetooth
#define BT_DEVICE_NAME "InSitu-Bioprinter"
#define DEFAULT_BAUD 115200
#define LINE_IDLE_MS 50  // строка без '\n' считается законченной после 50 мс тишины

// ----------------------------- Пины: дозатор (TB6612FNG) ---------------------
#define PIN_D1_PWM 21   // PWMA
#define PIN_D1_IN1 27   // AIN1
#define PIN_D1_IN2 14   // AIN2
#define PIN_D1_ENCA 32
#define PIN_D1_ENCB 33
#define PIN_D2_PWM 22   // PWMB
#define PIN_D2_IN1 12   // BIN1. GPIO12 — strapping-пин: при старте должен быть LOW
#define PIN_D2_IN2 13   // BIN2
#define PIN_D2_ENCA 34  // GPIO34/35 — только вход, внутренних подтяжек нет
#define PIN_D2_ENCB 35

// ----------------------------- Пины: главный привод (2× TMC2130) -------------
#define PIN_Z_EN 15
#define PIN_Z_DIR 16
#define PIN_Z_STEP 17
#define PIN_Z_CS 5      // SPI: SCK 18, MISO 19, MOSI 23 (VSPI по умолчанию)

// ----------------------------- Пины: HC-SR04 ---------------------------------
#define PIN_US_TRIG 2
#define PIN_US_ECHO 4

// ----------------------------- Дозатор ---------------------------------------
// Знак квадратурного декодирования каждого мотора — как в v0.1 (pin_A*/pin_B*_ISR).
#define ENC1_SIGN (-1)
#define ENC2_SIGN (+1)
constexpr float DISP_TICKS_PER_REV = 8344.0f;   // 28 импульсов × 298:1 на оборот винта
constexpr float DISP_VEL_LIMIT = 17000.0f;      // тик/с, верхняя граница команды vel
constexpr uint32_t DISP_PWM_FREQ = 20000;
constexpr uint8_t DISP_PWM_BITS = 10;
constexpr int DISP_PWM_MAX = 1023;
constexpr uint32_t DISP_PERIOD_MS = 10;          // период регулятора дозатора
constexpr float DISP_RELIEF_SPEED = 2000.0f;     // тик/с, отвод толкателя после заклинивания

// ----------------------------- Главный привод --------------------------------
constexpr int Z_FULL_STEPS = 200;
constexpr int Z_MICROSTEPS = 16;
constexpr float Z_RSENSE = 0.11f;
constexpr float Z_HOME_SPEED = 5.0f;          // мм/с, хоминг об верхний упор
constexpr float Z_HOME_CURRENT_K = 0.7f;      // доля рабочего тока при хоминге
constexpr float Z_HOME_EXTRA_MM = 10.0f;      // запас хода сверх zmax при хоминге
constexpr float Z_UNHOMED_JOG_SPEED = 5.0f;   // мм/с, zjog без известной позиции
constexpr float Z_FF_HOLD_EXIT = 0.5f;        // мм/с, скорость поверхности, выводящая из удержания
constexpr float Z_SURFACE_SPEED_MAX = 1000.0f;// мм/с, ограничение оценки скорости поверхности

// ----------------------------- Таймер шагов и ISR ----------------------------
constexpr uint32_t STEP_TIMER_HZ = 1000000;                 // 1 МГц
constexpr uint32_t STEP_TICK_US = 20;                       // ISR каждые 20 мкс (50 кГц)
constexpr uint32_t STEP_TICK_HZ = STEP_TIMER_HZ / STEP_TICK_US;
constexpr float Z_MAX_STEP_RATE = STEP_TICK_HZ / 2.0f;      // 25 000 шаг/с
constexpr uint32_t CTRL_WDT_TICKS = 50000 / STEP_TICK_US;   // 50 мс без контура управления

// ----------------------------- Задача управления -----------------------------
constexpr uint32_t CONTROL_PERIOD_MS = 1;
constexpr UBaseType_t CONTROL_TASK_PRIO = 5;
constexpr uint32_t PERSIST_IDLE_MS = 1000;   // позиции пишутся в NVS после 1 с покоя
