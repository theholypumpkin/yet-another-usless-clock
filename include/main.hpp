#ifndef Arduino_h
#include <Arduino.h>
#endif
#ifndef _MAIN_HPP
#define _MAIN_HPP
void setup();
void loop();
bool updateNetworkTime();
uint8_t calcDisplayBrightness(int x);
float calcBatteryPercentageLiPo(float x);
void timeAlarmISR();
void blink(uint16_t times, uint16_t delay_time = 500);
#endif