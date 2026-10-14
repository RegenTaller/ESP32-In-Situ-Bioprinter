// Заглушка регистров GPIO: симулятор перехватывает чтение и запись.
#pragma once
#include <cstdint>

#define GPIO_OUT_W1TS_REG 1u
#define GPIO_OUT_W1TC_REG 2u
#define GPIO_IN_REG 3u
#define GPIO_IN1_REG 4u

void simRegWrite(uint32_t reg, uint32_t value);
uint32_t simRegRead(uint32_t reg);

#define REG_WRITE(r, v) simRegWrite((r), (v))
#define REG_READ(r) simRegRead(r)
