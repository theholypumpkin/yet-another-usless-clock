/*main.cpp*/
/*================================================================================================*/
#include <Arduino.h>
#include "main.hpp"
#include "secrets.hpp"
#include <Adafruit_GFX.h>
#include <Adafruit_LEDBackpack.h>
#include <JC_Button_ESP.h>
#include <time.h>
#ifdef ARDUINO_ADAFRUIT_QTPY_ESP32S2
#include <ESP32Time.h>
#include <WiFi.h>
#elif ARDUINO_SAMD_NANO_33_IOT
#include <RTCZero.h>
#include <WiFiNINA.h>
#endif
//#include <WiFiUdp.h>
#include <NTPClient.h>
/*================================================================================================*/
//definitions
#define PHOTORESISTOR_BRIGHTNESS A3
#define BATTERY_VOLTAGE_PIN A2
#define TIME_BUTTON_PIN 8
#define DATE_BUTTON_PIN 9
#define BATTERY_VOLTAGE_BUTTON_PIN 10

#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 60
/*================================================================================================*/
enum statemachine_t
{
    DISPLAY_TIME,
    DISPLAY_DATE,
    DISPALY_BATTERY
};
volatile statemachine_t e_state = DISPLAY_DATE;
/*================================================================================================*/
//uint64_t sleep = 0;
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
    analogReadResolution(4); //4bits
    Serial.begin(9600);
    if(updateNetworkTime()){ //set real time acording to network
        Serial.println("Network Time successful");
    }else{
        Serial.println("ERROR: No Network Time");  
    }
    bool begun = display.begin(0x70);
    Serial.println(begun ? "True" : "False");
    delay(10000);
    //e_state = DISPLAY_TIME;
}
/*________________________________________________________________________________________________*/
void loop(){
    //Serial.println(rtc.getTime("%A, %B %d %Y %H:%M:%S"));
    uint16_t brightness = analogRead(PHOTORESISTOR_BRIGHTNESS);
    static float voltage = ((float) analogRead(BATTERY_VOLTAGE_PIN)*4.2) / 4095;
    static uint8_t hour, minute, day, month;
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
            display.setBrightness(brightness); //Right shift by 10 bits
            display.printFloat(day + (month/100), 2, DEC);
            display.writeDisplay();
            break;
        case DISPALY_BATTERY:
            display.printFloat(voltage, 2, DEC);
            display.writeDisplay(); 
            break;
    }
    //esp_deep_sleep(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    delay(10000);
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
/*time_t charTotimeStruct(const char* time, const char* date)
{
    // sample input: date = "Dec 06 2009", time = "12:34:56"
    _yearFrom2000 = StringToUint8(date + 9);
    // Jan Feb Mar Apr May Jun Jul Aug Sep Oct Nov Dec
    switch (date[0])
    {
    case 'J':
        if ( date[1] == 'a' )
            _month = 1;
        else if ( date[2] == 'n' )
            _month = 6;
        else
            _month = 7;
        break;
    case 'F':
        _month = 2;
        break;
    case 'A':
        _month = date[1] == 'p' ? 4 : 8;
        break;
    case 'M':
        _month = date[2] == 'r' ? 3 : 5;
        break;
    case 'S':
        _month = 9;
        break;
    case 'O':
        _month = 10;
        break;
    case 'N':
        _month = 11;
        break;
    case 'D':
        _month = 12;
        break;
    }
    _dayOfMonth = StringToUint8(date + 4);
    _hour = StringToUint8(time);
    _minute = StringToUint8(time + 3);
    _second = StringToUint8(time + 6);
}*/
/*end of file*/