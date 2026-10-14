// Commands.h — приём команд (COM и Bluetooth), ответы, телеметрия.
#pragma once
#include <Arduino.h>

void commandsBegin();   // запуск Bluetooth
void commandsPoll();    // приём строк из COM и Bluetooth
void eventsPoll();      // сообщения о событиях и авариях
void telemetryPoll();   // периодический вывод данных

void outWrite(const char *s);
void outPrintf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
