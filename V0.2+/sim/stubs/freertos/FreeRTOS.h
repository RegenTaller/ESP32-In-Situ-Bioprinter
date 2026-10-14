// Заглушка FreeRTOS для симуляции: однопоточная модель, блокировки — пустые.
#pragma once
#include <cstdint>

#define configTICK_RATE_HZ 1000
#define portMAX_DELAY 0xFFFFFFFFu
#define pdMS_TO_TICKS(x) (x)

typedef unsigned int UBaseType_t;
typedef uint32_t TickType_t;
typedef void *TaskHandle_t;
typedef void *SemaphoreHandle_t;

struct portMUX_TYPE {
  int unused;
};
#define portMUX_INITIALIZER_UNLOCKED {0}
inline void portENTER_CRITICAL(portMUX_TYPE *) {}
inline void portEXIT_CRITICAL(portMUX_TYPE *) {}
inline void portENTER_CRITICAL_ISR(portMUX_TYPE *) {}
inline void portEXIT_CRITICAL_ISR(portMUX_TYPE *) {}

inline TickType_t xTaskGetTickCount() { return 0; }
inline void vTaskDelayUntil(TickType_t *, TickType_t) {}
inline int xTaskCreatePinnedToCore(void (*)(void *), const char *, uint32_t, void *, UBaseType_t, TaskHandle_t *, int) {
  return 1;
}
