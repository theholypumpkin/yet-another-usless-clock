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
#define PHOTORESISTOR_BRIGHTNESS A1
#define BATTERY_VOLTAGE_PIN A2
#define TIME_BUTTON_PIN 2
#define DATE_BUTTON_PIN 3
#define BATTERY_VOLTAGE_BUTTON_PIN 9

#define VIN_VOLTAGE_MAX 21
#define BATTERY_VOLTAGE VIN_VOLTAGE_MAX
//#define ADC_RESOLUTION_BITS 4
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
volatile bool g_timeAlarmIsrFlag = true; //intial state is true so time is displayed on first loop.
uint16_t g_error_WiFi_Status_Code = WL_CONNECTED; 
#endif
const float MAX_BATTERY_VOLTAGE = 4.2, //use a R1 = 1k, 3.3k or or higher in same ratio.
            ADC_VOLTAGE_FACTOR = MAX_BATTERY_VOLTAGE / powf(2.0, ADC_RESOLUTION);
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
    pinMode(LED_BUILTIN, OUTPUT);
    analogReadResolution(ADC_RESOLUTION); //4bits
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
        rtc.setAlarmSeconds(0); //Trigger an alarm on every full minute
        rtc.attachInterrupt(timeAlarmISR);
        rtc.enableAlarm(rtc.MATCH_SS);
    #endif
}
/*________________________________________________________________________________________________*/
void loop(){
    uint8_t brightness = calcDisplayBrightness(analogRead(PHOTORESISTOR_BRIGHTNESS));
    static uint8_t oldHour, hour, minute, oldDay, day, month;
    long timeNumber;
    float batteryVoltage, batteryPercent;
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    //Read button 100 times and than when the time alarm triggers stop because the flag is true.
    while(!g_timeAlarmIsrFlag){ //when the alarm was not trigged loop over buttons
        timeButton.read();
        dateButton.read();
        batteryVoltageButton.read();
        
        if(timeButton.wasPressed()){
            e_state = DISPLAY_TIME;
            break;
        }
        else if(dateButton.wasPressed()){
            e_state = DISPLAY_DATE;
            break;
        }
        else if(batteryVoltageButton.wasPressed()){
            e_state = DISPALY_BATTERY;
            break;
        }
        delay(10); 
        /* power consumtion savings for sleep are minmal because most of the power consumption
         * comes from the LEDs not the MCU. So sleep wont contribute much of anything.
         * Also ArduinoLowPower.h also uses the RTC to set wakeup alarms. This could interfere
         * with the minutly time alarm of the rtc.
         */
    }
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    switch(e_state){
        /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
        case DISPLAY_TIME:
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
            timeNumber = hour*100+minute;
            display.printNumber(timeNumber, DEC);
            //Write a leading 0 to the clockface if we are before 10 o clock
            if(timeNumber < 1000){
                display.writeDigitNum(0, 0);
                //if we have midnight write two leading 0 to the clockface to indecate zero o clock.
                if(timeNumber < 100){
                    display.writeDigitNum(1, 0);
                }
            }
            display.drawColon(true);
            display.writeDisplay();
            break;
        /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
        case DISPLAY_DATE:
            oldDay = day;
            day = rtc.getDay();
            month = rtc.getMonth();
            //only update Display when date changes or button was pressed
            if((oldDay != day)){ 
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
            batteryVoltage = analogRead(BATTERY_VOLTAGE_PIN) * ADC_VOLTAGE_FACTOR;
            batteryPercent = calcBatteryPercentageLiPo(batteryVoltage);
            display.writeDigitNum(0,(long) batteryPercent/10);
            display.writeDigitNum(1,(long) batteryPercent%10);
            display.writeDigitRaw(3, 0b00000000);
            display.drawColon(false);
            display.writeDigitRaw(4, 0b11010010);
            //display.printNumber(analogRead(BATTERY_VOLTAGE_PIN), DEC);
            //display.printFloat(batteryVoltage, 2, DEC);
            display.writeDisplay(); 
            break;
        /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
        case DISPLAY_ERROR:
            display.printNumber(g_error_WiFi_Status_Code, DEC);
            display.writeDisplay();
            break;
    }
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    /*Just to wake up on button press we trigger an external interrupt.
     * JC Button does the rest
     */
    g_timeAlarmIsrFlag = false;
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    //Due to te lack of a crystal on the MCU the rtc is not really rtc and drifts a lot.
    // We fetch the networktime every hour to compensate for the drifting
    if(oldHour != hour){
        updateNetworkTime();
    }
    /*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
    #ifdef ARDUINO_ADAFRUIT_QTPY_ESP32S2
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_37, LOW);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_36, LOW);
    esp_deep_sleep(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    #endif
}
/*________________________________________________________________________________________________*/
bool updateNetworkTime(){
    /*String statusString[] = {"WL_IDLE_STATUS","WL_NO_SSID_AVAIL","WL_SCAN_COMPLETED",
        "WL_CONNECTED","WL_CONNECT_FAILED","WL_CONNECTION_LOST","WL_DISCONNECTED","WL_AP_LISTENING",
        "WL_AP_CONNECTED","WL_AP_FAILED"}; */
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
                //uint8_t status = WiFi.status();
                //blink(status);
                g_error_WiFi_Status_Code = WiFi.status();
            #endif
            return false;
        }
        delay(1000);
    }
    //if WiFi is sucessful I should exprect 3 blinks
    //uint8_t status = WiFi.status();
    //blink(status);

    WiFiUDP ntpUdpObject;
    //Summer time
    //NTPClient ntpClient(ntpUdpObject, g_ntpTimeServerURL, 7200);
    //Winter time
    NTPClient ntpClient(ntpUdpObject, g_ntpTimeServerURL, 3600);
    ntpClient.begin();
    connection_attempts = 0;
    while(!ntpClient.update()){ //attempt to connect to ntp server up to 5 times.
        connection_attempts++;
        //Serial.println("NTP failed");
        if(connection_attempts == 4){
            #ifdef ARDUINO_ADAFRUIT_QTPY_ESP32S2
                WiFi.disconnect(true, false); //now we no longer need wifi
            #elif ARDUINO_SAMD_NANO_33_IOT
                WiFi.end();
                g_error_WiFi_Status_Code = 1000;
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
/*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
uint8_t calcDisplayBrightness(int x){
    if(x > 90)
        return 15;
    else if(x < 15)
        return 0;
    else
        return (x/5)-3; //calculate brightness.
}
 #ifdef ARDUINO_SAMD_NANO_33_IOT
/*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
float calcBatteryPercentageLiPo(float x)
{
    if(x > 3.896)
        return 120.0f*x-404;
    else if (x > 3.648)
        return 255.0f*x - 930.0f;      
    else
        return 0.0;
}
/*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
void timeAlarmISR(){
    g_timeAlarmIsrFlag = true;
}
/*-   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   */
void blink(uint16_t times, uint16_t delay_time){
    delay(delay_time);
    for(uint16_t i = 1; i < times+1; i++){
        digitalWrite(LED_BUILTIN, HIGH);
        delay(delay_time);
        digitalWrite(LED_BUILTIN, LOW);
        delay(delay_time);
    }
}
/*________________________________________________________________________________________________*/
#endif
/*end of file*/