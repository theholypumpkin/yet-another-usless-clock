/*main.cpp*/
/*================================================================================================*/
#include <Arduino.h>
#include "main.hpp"
#include "secrets.hpp"
#include <Adafruit_GFX.h>
#include <Adafruit_LEDBackpack.h>
#include <JC_Button_ESP.h>
#include <time.h>
#include <ESP32Time.h>
#include <WiFi.h>
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
/*enum statemachine_t
{
    READ_BRIGHTNESS_LEVEL,
    READ_BATTERY_VOLTAGE,
    DISPLAY_TIME,
    DISPLAY_DATE,
    DISPALY_BATTERY,
    IDLE
};
volatile statemachine_t e_state = IDLE;*/
/*================================================================================================*/
//uint64_t sleep = 0;
/*________________________________________________________________________________________________*/
Button timeButton(TIME_BUTTON_PIN),
       dateButton(DATE_BUTTON_PIN),
       batteryVoltageButton(BATTERY_VOLTAGE_BUTTON_PIN);
ESP32Time rtc;
/*================================================================================================*/
void setup(){
    //analogReadResolution(SOC_ADC_MAX_BITWIDTH);
    Serial.begin(9600);
    if(updateNetworkTime()){ //set real time acording to network
        Serial.println("Network Time successful");
        //Serial.println(rtc.getTime("%A, %B %d %Y %H:%M:%S"));
    }else{
        Serial.println("ERROR: No Network Time");  
    }
    delay(10000);
    //sleep = TIME_TO_SLEEP * uS_TO_S_FACTOR;
    //esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
}
/*________________________________________________________________________________________________*/
void loop(){
    Serial.println(rtc.getTime("%A, %B %d %Y %H:%M:%S"));
    uint16_t brightness = analogRead(PHOTORESISTOR_BRIGHTNESS);
    Serial.print(brightness,DEC);
    Serial.print(", ");
    Serial.println(brightness, BIN);
    Serial.println("Going to Sleep");
    delay(1000);
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
            WiFi.disconnect(true, false); //now we no longer need wifi
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
            WiFi.disconnect(true, false); //now we no longer need wifi
            return false;
        }
        delay(1000);
    }
    unsigned long epochTime = ntpClient.getEpochTime();
    rtc.setTime(epochTime);
    WiFi.disconnect(true, false); //now we no longer need wifi
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