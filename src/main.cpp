/*main.cpp*/
/*===============================================================================================*/
#include <Arduino.h>
#include "main.hpp"
#include "secrets.hpp"
#include <Adafruit_GFX.h>
#include <Adafruit_LEDBackpack.h>
#include <Battery.h>
#ifdef ARDUINO_ADAFRUIT_QTPY_ESP32S2
#include <ESP32Time.h>
#include <WiFi.h>
#include <JC_Button_ESP.h>
#elif ARDUINO_SAMD_NANO_33_IOT
#include <RTCZero.h>
#include <WiFiNINA.h>
#include <JC_Button.h>
#endif
#include <NTPClient.h>
/*===============================================================================================*/
//definitions
#ifdef ARDUINO_ADAFRUIT_QTPY_ESP32S2

    #define PHOTORESISTOR_BRIGHTNESS A3
    #define BATTERY_VOLTAGE_PIN A2
    #define TIME_BUTTON_PIN 36
    #define DATE_BUTTON_PIN 37
    #define BATTERY_VOLTAGE_BUTTON_PIN 35
    #define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
    #define TIME_TO_SLEEP 60

#elif ARDUINO_SAMD_NANO_33_IOT
    /* -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   - */
    // pin defintions
    #define PHOTORESISTOR_BRIGHTNESS A0
    #define BATTERY_VOLTAGE_PIN A7
    #define TIME_BUTTON_PIN 2
    #define DATE_BUTTON_PIN 3
    #define BATTERY_VOLTAGE_BUTTON_PIN 10
#endif
/* -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   - */
// macro to calculate the power ADC resulation at compile time
#define POW_TWO(x) (1 << (x))
/* -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   - */
// refrerence voltages
#define MAX_BATTERY_MILLI_VOLTAGE 4200
#define MIN_BATTERY_MILLI_VOLTAGE 3000
#define BOARD_REFERENCE_VOLTAGE 3300
/* -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   - */
// calculate voltage divider ratio
#define VOLTAGE_DIVIDER_R1 980
#define VOLTAGE_DIVIDER_R2 1501
#define VOLTAGE_DIVIDER_RATIO (VOLTAGE_DIVIDER_R1 + VOLTAGE_DIVIDER_R2) / VOLTAGE_DIVIDER_R2
/* -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   - */
// calculate adc reading to battery voltage conversion factor
#define MAX_BATTERY_VOLTAGE MAX_BATTERY_MILLI_VOLTAGE / 1000.0f
#define ADC_VOLTAGE_FACTOR MAX_BATTERY_VOLTAGE / POW_TWO(ADC_RESOLUTION)

/*===============================================================================================*/
// enums, structs, unions, typedef
enum statemachine_t
{
    IDLE,
    DISPLAY_TIME,
    DISPLAY_DATE,
    DISPALY_BATTERY,
    DISPLAY_ERROR
};
volatile statemachine_t e_state = DISPLAY_TIME;
/*===============================================================================================*/
// global objects
#ifdef ARDUINO_SAMD_NANO_33_IOT 
uint16_t g_error_WiFi_Status_Code = WL_CONNECTED; 
#endif
/*_______________________________________________________________________________________________*/
// create the buttons
Button timeButton(TIME_BUTTON_PIN),
       dateButton(DATE_BUTTON_PIN),
       batteryVoltageButton(BATTERY_VOLTAGE_BUTTON_PIN);
/* -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   - */
#ifdef ARDUINO_ADAFRUIT_QTPY_ESP32S2
    ESP32Time rtc;
#elif ARDUINO_SAMD_NANO_33_IOT
    RTCZero rtc;
#endif
/* -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   - */
Adafruit_7segment display = Adafruit_7segment();
/* -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   - */
Battery LiPo = Battery(3000, MAX_BATTERY_MILLI_VOLTAGE, BATTERY_VOLTAGE_PIN, ADC_RESOLUTION);
/*===============================================================================================*/
void setup(){
    pinMode(LED_BUILTIN, OUTPUT);
    analogReadResolution(ADC_RESOLUTION); // set ADC resulution to max of 12 bits
    Serial.begin(9600);
    //while(!Serial);
    #ifdef ARDUINO_SAMD_NANO_33_IOT
        rtc.begin();
    #endif
    // if current time can not be fetched show error
    if(!updateNetworkTime()){
        e_state = DISPLAY_ERROR; 
    }
    display.begin();
    /* -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   - */
    timeButton.begin();
    dateButton.begin();
    batteryVoltageButton.begin();
    /* -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   - */
    #ifdef ARDUINO_SAMD_NANO_33_IOT   
        rtc.setAlarmSeconds(0); //Trigger an alarm on every full minute
        rtc.attachInterrupt(timeAlarmISR);
        rtc.enableAlarm(rtc.MATCH_SS);
    #endif
    /* -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   - */
    /* while this is battery powered, the power consumption is already rather high, so the extra
     * consumption of a complex calulation (the most precise one), doesn;t really factor in.
     */
    LiPo.begin(BOARD_REFERENCE_VOLTAGE, VOLTAGE_DIVIDER_RATIO, &asigmoidal);
}
/*_______________________________________________________________________________________________*/

void loop(){
    static uint8_t oldHour, hour, oldDay, day;
    /* -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   - */
    // read button (debounce)
    timeButton.read();
    dateButton.read();
    batteryVoltageButton.read();
    
    // check if buttons where pressed
    if(timeButton.wasPressed())
        e_state = DISPLAY_TIME;

    else if(dateButton.wasPressed())
        e_state = DISPLAY_DATE;

    else if(batteryVoltageButton.wasPressed())
        e_state = DISPALY_BATTERY;

    /* -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   - */
    switch(e_state){

        case IDLE:
            // The do nothing state
            break;

        case DISPLAY_TIME:
            { //dummy block too keep on stack only what is needed
                #ifdef ARDUINO_ADAFRUIT_QTPY_ESP32S2
                hour = rtc.getHour(true);
                uint8_t minute = rtc.getMinute();

                #elif ARDUINO_SAMD_NANO_33_IOT
                oldHour = hour;
                hour = rtc.getHours();
                uint8_t minute = rtc.getMinutes();

                #endif

                uint8_t brightness = calcDisplayBrightness(analogRead(PHOTORESISTOR_BRIGHTNESS));
                display.setBrightness(brightness);
                
                long timeNumber = hour * 100 + minute;
                display.printNumber(timeNumber, DEC);

                //Write a leading 0 to the clockface if we are before 10 o clock
                if(timeNumber < 1000){
                    display.writeDigitNum(0, 0);

                    //if we have midnight write two leading 0 to the clockface to indecate 
                    //zero 0 clock.
                    if(timeNumber < 100){
                        display.writeDigitNum(1, 0);
                    }
                }
                display.drawColon(true);
                display.writeDisplay();

                e_state = IDLE;
            }
            break;
        /* -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   - */
        case DISPLAY_DATE:
            {
                oldDay = day;
                day = rtc.getDay();
                uint8_t month = rtc.getMonth();

                    
                #ifdef ARDUINO_ADAFRUIT_QTPY_ESP32S2
                month+=1; //0 to 11 hence +1
                #endif

                uint8_t brightness = calcDisplayBrightness(
                    analogRead(PHOTORESISTOR_BRIGHTNESS));
                display.setBrightness(brightness);
                display.print((double)(day+(month/100.0)));
                display.writeDisplay();

                e_state = IDLE;
            }
            break;
        /* -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   - */
        case DISPALY_BATTERY:
            {

                uint8_t brightness = calcDisplayBrightness(
                            analogRead(PHOTORESISTOR_BRIGHTNESS));
                        display.setBrightness(brightness);

                display.printNumber(LiPo.level());
                
                display.writeDisplay();

                Serial.print("Battery voltage is ");
	            Serial.print(LiPo.voltage());
	            Serial.print(" (");
	            Serial.print(LiPo.level());
	            Serial.println("%)");

                e_state = IDLE;
            }
            break;
        /* -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   - */
        case DISPLAY_ERROR:
            display.printNumber(g_error_WiFi_Status_Code, DEC);
            display.writeDisplay();

            e_state = IDLE;
            
            break;
    }
    /* -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   - */
    //Due to te lack of a crystal on the MCU the rtc isn't mucht better than a software clock.
    // We fetch the networktime every hour to compensate for the drifting
    if(oldHour != hour){
        updateNetworkTime();
    }
    /* -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   - */
    #ifdef ARDUINO_ADAFRUIT_QTPY_ESP32S2
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_37, LOW);
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_36, LOW);
    esp_deep_sleep(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    #endif
}
/*_______________________________________________________________________________________________*/
bool updateNetworkTime(){
    /*String statusString[] = {"WL_IDLE_STATUS","WL_NO_SSID_AVAIL","WL_SCAN_COMPLETED",
        "WL_CONNECTED","WL_CONNECT_FAILED","WL_CONNECTION_LOST","WL_DISCONNECTED",
        "WL_AP_LISTENING", "WL_AP_CONNECTED","WL_AP_FAILED"}; */
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
    #ifdef ARDUINO_ADAFRUIT_QTPY_ESP32S2
        rtc.setTime(epochTime);
        WiFi.disconnect(true, false); //now we no longer need wifi
    #elif ARDUINO_SAMD_NANO_33_IOT
        rtc.setEpoch(epochTime);
        WiFi.end();
    #endif
    return true;
}
/* -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   - */
uint8_t calcDisplayBrightness(int adc_reading){
    //return adc_reading >> 8; // shift from 12 bits to 4 bits resolution
    return adc_reading/273; //plot brightness liniar to adc reading
}

/* -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   - */
 #ifdef ARDUINO_SAMD_NANO_33_IOT
void timeAlarmISR(){
    e_state = DISPLAY_TIME;
}
#endif
/* -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   -   - */
 #ifdef ARDUINO_SAMD_NANO_33_IOT
void blink(uint16_t times, uint16_t delay_time){

    delay(delay_time);

    for(uint16_t i = 1; i < times+1; i++){
        digitalWrite(LED_BUILTIN, HIGH);
        delay(delay_time);
        digitalWrite(LED_BUILTIN, LOW);
        delay(delay_time);
    }
}
#endif
/*===============================================================================================*/
/*end of file*/