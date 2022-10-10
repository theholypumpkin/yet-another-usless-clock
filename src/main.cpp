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
#define PHOTORESISTOR_BRIGHTNESS A3
#define BATTERY_VOLTAGE_PIN A2
#define TIME_BUTTON_PIN 36
#define DATE_BUTTON_PIN 37
#define BATTERY_VOLTAGE_BUTTON_PIN 35

#define VIN_VOLTAGE_MAX 21
#define BATTERY_VOLTAGE VIN_VOLTAGE_MAX
#define ADC_RESOLUTION_BITS 4
#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 60
/*================================================================================================*/
enum statemachine_t
{
    DISPLAY_TIME,
    DISPLAY_DATE,
    DISPALY_BATTERY
};
volatile statemachine_t e_state = DISPLAY_TIME;
/*================================================================================================*/
//Only wate from external interrupt on the Nano 33 IoT ESP32 has very different way to handel INT
#ifdef ARDUINO_SAMD_NANO_33_IOT 
bool g_timeButtonIsrFlag = false, g_dateButtonIsrFlag = false , g_batteryVoltageIsrFlag = false;
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
    Serial.begin(9600);
    if(updateNetworkTime()){ //set real time acording to network
        Serial.println("Network Time successful");
    }else{
        Serial.println("ERROR: No Network Time");  
    }
    display.begin(0x70);
    timeButton.begin();
    dateButton.begin();
    batteryVoltageButton.begin();

    //delay(10000);
}
/*________________________________________________________________________________________________*/
void loop(){
    uint16_t brightness = analogRead(PHOTORESISTOR_BRIGHTNESS);
    /*static float voltage = 
        ((float) analogRead(BATTERY_VOLTAGE_PIN)*BATTERY_VOLTAGE) / pow((float)2, 
            (float)ADC_RESOLUTION_BITS);*/
    static uint8_t hour, minute, day, month, batteryPercent;
    static uint8_t batteryPercentLut[16] = {10,16,22,28,34,40,46,52,58,64,70,76,82,88,94,100};
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

    switch(e_state){
        case DISPLAY_TIME:
            #ifdef ARDUINO_ADAFRUIT_QTPY_ESP32S2
                hour = rtc.getHour(true); //TODO longpress button to change
                minute = rtc.getMinute();
            #elif ARDUINO_SAMD_NANO_33_IOT
                hour = rtc.getHours();
                minute = rtc.getMinutes();
            #endif
            display.setBrightness(brightness); //Right shift by 10 bits
            display.printNumber(hour*100+minute, DEC);
            display.drawColon(true);
            display.writeDisplay();
            break;
        case DISPLAY_DATE:
            day = rtc.getDay();
            month = rtc.getMonth(); 
            #ifdef ARDUINO_ADAFRUIT_QTPY_ESP32S2
            month+=1; //0 to 11 hence +1
            #endif
            display.setBrightness(brightness); //Right shift by 10 bits
            display.print((double)(day+(month/100.0)));
            display.writeDisplay();
            break;
        case DISPALY_BATTERY:
            batteryPercent = analogRead(BATTERY_VOLTAGE_PIN);
            display.writeDigitRaw(0, batteryPercentLut[batteryPercent]/10);
            display.writeDigitRaw(1, batteryPercentLut[batteryPercent]%10);
            display.writeDigitAscii(4, 'P', true);
            display.writeDisplay(); 
            break;
    }
    //Untested because of Switch of Chip
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_37, LOW);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_36, LOW);
    esp_deep_sleep(TIME_TO_SLEEP * uS_TO_S_FACTOR);

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
/*________________________________________________________________________________________________*/
void timeButtonISR(){
    g_timeButtonIsrFlag = true;
}
/*________________________________________________________________________________________________*/
void dateButtonISR(){
    g_dateButtonIsrFlag = true;
}
/*________________________________________________________________________________________________*/
void batteryVoltageButtonISR(){
    g_batteryVoltageIsrFlag = true;
}
#endif
/*end of file*/