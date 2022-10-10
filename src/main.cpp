/*main.cpp*/
/*================================================================================================*/
#include <Arduino.h>
#include "main.hpp"
#include "secrets.hpp"
#include <Adafruit_GFX.h>
#include <Adafruit_LEDBackpack.h>
#include <time.h>
#include <math.h>
#ifdef ARDUINO_ADAFRUIT_QTPY_ESP32S2
#include <ESP32Time.h>
#include <WiFi.h>
#include <JC_Button_ESP.h>
#elif ARDUINO_SAMD_NANO_33_IOT
#include <RTCZero.h>
#include <WiFiNINA.h>
#include <JC_Button.h>
#endif
//#include <WiFiUdp.h>
#include <NTPClient.h>
/*================================================================================================*/
//definitions
#ifdef ARDUINO_ADAFRUIT_QTPY_ESP32S2
#define PHOTORESISTOR_BRIGHTNESS A3
#define BATTERY_VOLTAGE_PIN A2
#define TIME_BUTTON_PIN 36
#define DATE_BUTTON_PIN 37
#define BATTERY_VOLTAGE_BUTTON_PIN 35

#define VIN_VOLTAGE_MAX 6
#define BATTERY_VOLTAGE VIN_VOLTAGE_MAX
#define ADC_RESOLUTION_BITS 4
#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 60
#elif ARDUINO_SAMD_NANO_33_IOT
#define PHOTORESISTOR_BRIGHTNESS A7
#define BATTERY_VOLTAGE_PIN A6
#define TIME_BUTTON_PIN 9
#define DATE_BUTTON_PIN 10
#define BATTERY_VOLTAGE_BUTTON_PIN 11

#define VIN_VOLTAGE_MAX 21
#define BATTERY_VOLTAGE VIN_VOLTAGE_MAX
#define ADC_RESOLUTION_BITS 4
#endif
/*================================================================================================*/
enum statemachine_t
{
    DISPLAY_TIME,
    DISPLAY_DATE,
    DISPALY_BATTERY,
    DISPLAY_ERROR
};
volatile statemachine_t e_state = DISPLAY_TIME;
/*================================================================================================*/
//Only wate from external interrupt on the Nano 33 IoT ESP32 has very different way to handel INT
#ifdef ARDUINO_SAMD_NANO_33_IOT 
bool g_timeButtonIsrFlag = false, 
     g_dateButtonIsrFlag = false , 
     g_batteryVoltageIsrFlag = false,
     g_timeAlarmIsrFlag = false;
#endif
/*________________________________________________________________________________________________*/
Button timeButton(TIME_BUTTON_PIN),
       dateButton(DATE_BUTTON_PIN),
       batteryVoltageButton(BATTERY_VOLTAGE_BUTTON_PIN);
#ifdef ARDUINO_ADAFRUIT_QTPY_ESP32S2
    ESP32Time rtc;
#elif ARDUINO_SAMD_NANO_33_IOT
    RTCZero rtc;
#endif
Adafruit_7segment display = Adafruit_7segment();
/*================================================================================================*/
void setup(){
    analogReadResolution(ADC_RESOLUTION_BITS); //4bits
    //Serial.begin(9600);
    //while(!Serial);
    #ifdef ARDUINO_SAMD_NANO_33_IOT
        rtc.begin();
    #endif
    if(!updateNetworkTime()){ //set real time acording to network
        e_state = DISPLAY_ERROR; 
    }
    display.begin(0x70);
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    timeButton.begin();
    dateButton.begin();
    batteryVoltageButton.begin();
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    #ifdef ARDUINO_SAMD_NANO_33_IOT
        attachInterrupt(digitalPinToInterrupt(TIME_BUTTON_PIN), timeButtonISR, FALLING);
        attachInterrupt(digitalPinToInterrupt(DATE_BUTTON_PIN), dateButtonISR, FALLING);
        attachInterrupt(digitalPinToInterrupt(BATTERY_VOLTAGE_BUTTON_PIN), 
            batteryVoltageButtonISR, FALLING);
        /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */    
        rtc.setAlarmSeconds(0); //Trigger an alarm on every full minute
        rtc.attachInterrupt(timeAlarmISR);
        rtc.enableAlarm(rtc.MATCH_SS);
    #endif
    delay(5000); //To allow new programms to be uploaded
}
/*________________________________________________________________________________________________*/
void loop(){
    uint16_t brightness = analogRead(PHOTORESISTOR_BRIGHTNESS);
    static uint8_t oldHour, hour, minute, oldDay, day, month, batteryPercent;
    static uint8_t batteryPercentLut[16] = {10,16,22,28,34,40,46,52,58,64,70,76,82,88,94,100};
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    timeButton.read();
    dateButton.read();
    batteryVoltageButton.read();
    
    if(timeButton.wasPressed()){
        e_state = DISPLAY_TIME;
    }
    else if(dateButton.wasPressed()){
        e_state = DISPLAY_DATE;
    }
    else if(batteryVoltageButton.wasPressed()){
        e_state = DISPALY_BATTERY;
    }
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    switch(e_state){
        /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
        case DISPLAY_TIME:
            if(g_timeAlarmIsrFlag || g_timeButtonIsrFlag){
                //Serial.println("Display Time");
                #ifdef ARDUINO_ADAFRUIT_QTPY_ESP32S2
                    hour = rtc.getHour(true);
                    minute = rtc.getMinute();
                #elif ARDUINO_SAMD_NANO_33_IOT
                    oldHour = hour;
                    hour = rtc.getHours();
                    minute = rtc.getMinutes();
                    //Serial.println(hour);
                    //Serial.println(minute);
                #endif
                display.setBrightness(brightness); //Right shift by 10 bits
                display.printNumber(hour*100+minute, DEC);
                display.drawColon(true);
                display.writeDisplay();
            }
            break;
        /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
        case DISPLAY_DATE:
            oldDay = day;
            day = rtc.getDay();
            month = rtc.getMonth();
            //only update Display when date changes or button was pressed
            if((oldDay != day) || g_dateButtonIsrFlag){ 
                //Serial.println("Display Date");
                //Serial.println(day);
                //Serial.println(month);
                #ifdef ARDUINO_ADAFRUIT_QTPY_ESP32S2
                month+=1; //0 to 11 hence +1
                #endif
                display.setBrightness(brightness); //Right shift by 10 bits
                display.print((double)(day+(month/100.0)));
                display.writeDisplay();
            }
            break;
        /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
        case DISPALY_BATTERY:
            if(g_timeAlarmIsrFlag || g_batteryVoltageIsrFlag){
                batteryPercent = analogRead(BATTERY_VOLTAGE_PIN);
                display.writeDigitNum(0, batteryPercentLut[batteryPercent]/10);
                display.writeDigitNum(1, batteryPercentLut[batteryPercent]%10);
                display.writeDigitRaw(3, 0b00000000);
                display.drawColon(false);
                display.writeDigitRaw(4, 0b11010010);
                display.writeDisplay(); 
            }
            break;
        /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
        case DISPLAY_ERROR:
            display.printError();
            display.writeDisplay();
            break;
    }
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    /*Just to wake up on button press we trigger an external interrupt.
     * JC Button does the rest
     */
    if(g_dateButtonIsrFlag || g_timeButtonIsrFlag || g_batteryVoltageIsrFlag || g_timeAlarmIsrFlag){
        g_timeButtonIsrFlag = false;
        g_dateButtonIsrFlag = false;
        g_batteryVoltageIsrFlag = false;
        g_timeAlarmIsrFlag = false;
    }
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    //Due to te lack of a crystal on the MCU the rtc is not really rtc and drifts a lot.
    // We adjust for drifting every hour.
    if(oldHour != hour){
        updateNetworkTime();
    }
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    #ifdef ARDUINO_ADAFRUIT_QTPY_ESP32S2
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_37, LOW);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_36, LOW);
    esp_deep_sleep(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    #elif ARDUINO_SAMD_NANO_33_IOT
    rtc.standbyMode();
    //Serial.println("Loop");
    //delay(1000);
    #endif
}
/*________________________________________________________________________________________________*/
bool updateNetworkTime(){
    WiFi.setHostname(g_hostname);
    uint8_t connection_attempts = 0;
    WiFi.begin(g_wifiSsid, g_wifiPass);
    while (WiFi.status() != WL_CONNECTED){ //Loop until connected to Wifi
        connection_attempts++;
        if(connection_attempts == 4){
            #ifdef ARDUINO_ADAFRUIT_QTPY_ESP32S2
                WiFi.disconnect(true, false); //now we no longer need wifi
            #elif ARDUINO_SAMD_NANO_33_IOT
                WiFi.end();
            #endif
            return false;
        }
        delay(1000);
    } 
    WiFiUDP ntpUdpObject;
    NTPClient ntpClient(ntpUdpObject, g_ntpTimeServerURL, 7200);
    ntpClient.begin();
    connection_attempts = 0;
    while(!ntpClient.update()){ //attempt to connect to ntp server up to 5 times.
        connection_attempts++;
        if(connection_attempts == 4){
            #ifdef ARDUINO_ADAFRUIT_QTPY_ESP32S2
                WiFi.disconnect(true, false); //now we no longer need wifi
            #elif ARDUINO_SAMD_NANO_33_IOT
                WiFi.end();
            #endif
            return false;
        }
        delay(1000);
    }
    unsigned long epochTime = ntpClient.getEpochTime();
    //Serial.print("Epoch: ");
    //Serial.println(epochTime);
    #ifdef ARDUINO_ADAFRUIT_QTPY_ESP32S2
        rtc.setTime(epochTime);
        WiFi.disconnect(true, false); //now we no longer need wifi
    #elif ARDUINO_SAMD_NANO_33_IOT
        rtc.setEpoch(epochTime);
        WiFi.end();
    #endif
    return true;
}
 #ifdef ARDUINO_SAMD_NANO_33_IOT
/*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
void timeButtonISR(){
    g_timeButtonIsrFlag = true;
}
/*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
void dateButtonISR(){
    g_dateButtonIsrFlag = true;
}
/*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
void batteryVoltageButtonISR(){
    g_batteryVoltageIsrFlag = true;
}
/*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
void timeAlarmISR(){
    g_timeAlarmIsrFlag = true;
}
/*________________________________________________________________________________________________*/
#endif
/*end of file*/