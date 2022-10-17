#ifndef Arduino_h
#include <Arduino.h>
#endif
#ifndef _MAIN_HPP
#define _MAIN_HPP
void setup();
void loop();
bool updateNetworkTime();
void timeButtonISR();
void dateButtonISR();
void batteryVoltageButtonISR();
void timeAlarmISR();
void blink(uint16_t times, uint16_t delay_time = 500);
#endif