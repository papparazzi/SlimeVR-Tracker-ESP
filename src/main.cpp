/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 Eiren Rain

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.
*/

#include "Wire.h"
#include "ota.h"
#include "sensor.h"
#include "configuration.h"
#include "wifihandler.h"
#include "udpclient.h"
#include "defines.h"
#include "credentials.h"
#include <i2cscan.h>
#include "serialcommands.h"

#if IMU == IMU_BNO080 || IMU == IMU_BNO085
    BNO080Sensor sensor{};
    #if defined(SECOND_IMU) && SECOND_IMU && defined(PIN_IMU_INT_2)
        #define HAS_SECOND_IMU true
        BNO080Sensor sensor2{};
    #endif
#elif IMU == IMU_BNO055
    BNO055Sensor sensor{};
#elif IMU == IMU_MPU9250
    MPU9250Sensor sensor{};
#elif IMU == IMU_MPU6500
    MPU6050Sensor sensor{};
#else
    #error Unsupported IMU
#endif
#ifndef HAS_SECOND_IMU
    EmptySensor sensor2{};
#endif
DeviceConfig config{};

bool isCalibrating = false;
bool blinking = false;
unsigned long blinkStart = 0;
unsigned long now_ms, last_ms = 0; //millis() timers
unsigned long last_battery_sample = 0;
bool secondImuActive = false;

void setConfig(DeviceConfig newConfig)
{
    config = newConfig;
    saveConfig(&config);
}

void commandRecieved(int command, void * const commandData, int commandDataLength)
{
    switch (command)
    {
    case COMMAND_CALLIBRATE:
        isCalibrating = true;
        break;
    case COMMAND_SEND_CONFIG:
        sendConfig(&config, PACKET_CONFIG);
        break;
    case COMMAND_BLINK:
        blinking = true;
        blinkStart = now_ms;
        break;
    }
}

void processBlinking();

void setup()
{
    //wifi_set_sleep_type(NONE_SLEEP_T);
    // Glow diode while loading
    pinMode(LOADING_LED, OUTPUT);
    pinMode(CALIBRATING_LED, OUTPUT);
    digitalWrite(CALIBRATING_LED, HIGH);
    digitalWrite(LOADING_LED, LOW);
    
    Serial.begin(serialBaudRate);
    setUpSerialCommands();

    // join I2C bus
    Wire.begin(PIN_IMU_SDA, PIN_IMU_SCL);
#ifdef ESP8266
    Wire.setClockStretchLimit(150000L); // Default streatch limit 150mS
#endif
    Wire.setClock(I2C_SPEED);

    if (hasConfigStored())
    {
        loadConfig(&config);
    }
    
    setConfigRecievedCallback(setConfig);
    setCommandRecievedCallback(commandRecieved);
    // Wait for IMU to boot
    delay(500);
    
    // Currently only second BNO08X is supported
#if IMU == IMU_BNO080 || IMU == IMU_BNO085
    #ifdef HAS_SECOND_IMU
        uint8_t first = I2CSCAN::pickDevice(BNO_ADDR_1, BNO_ADDR_2, true);
        uint8_t second = I2CSCAN::pickDevice(BNO_ADDR_2, BNO_ADDR_1, false);
        if(first != second) {
            sensor.setupBNO080(false, first, PIN_IMU_INT);
            sensor2.setupBNO080(true, second, PIN_IMU_INT_2);
            secondImuActive = true;
        } else {
            sensor.setupBNO080(false, first, PIN_IMU_INT);
        }
    #else
    sensor.setupBNO080(false, I2CSCAN::pickDevice(BNO_ADDR_1, BNO_ADDR_2, true), PIN_IMU_INT);
    #endif
#endif
    processBlinking();
    sensor.motionSetup(&config);
#ifdef HAS_SECOND_IMU
    if(secondImuActive)
        sensor2.motionSetup(&config);
#endif

    setUpWiFi(&config);
    otaSetup(otaPassword);
    digitalWrite(LOADING_LED, HIGH);
}

// AHRS loop

void loop()
{
    serialCommandsUpdate();
    wifiUpkeep();
    // sascha OTA not used now
    //otaUpdate();
    clientUpdate(&sensor, &sensor2);
    if (isCalibrating)
    {
        sensor.startCalibration(0);
        //sensor2.startCalibration(0);
        isCalibrating = false;
    }
#ifndef UPDATE_IMU_UNCONNECTED
        if(isConnected()) {
#endif
    sensor.motionLoop();
#ifdef HAS_SECOND_IMU
    sensor2.motionLoop();
#endif
#ifndef UPDATE_IMU_UNCONNECTED
        }
#endif
    // Send updates
    now_ms = millis();
    if (now_ms - last_ms >= samplingRateInMillis)
    {
        last_ms = now_ms;
        processBlinking();
    }
#ifndef SEND_UPDATES_UNCONNECTED
    if(isConnected()) {
#endif
        sensor.sendData();
#ifdef HAS_SECOND_IMU
        sensor2.sendData();
#endif
#ifndef SEND_UPDATES_UNCONNECTED
    }
#endif
#ifdef PIN_BATTERY_LEVEL
    if(now_ms - last_battery_sample >= batterySampleRate) {
        last_battery_sample = now_ms;
        float battery = ((float) analogRead(PIN_BATTERY_LEVEL)) * batteryADCMultiplier;
        sendFloat(battery, PACKET_BATTERY_LEVEL);
    }
#endif
}

void processBlinking() {
    if (blinking)
    {
        if (blinkStart + sensorIdTime < now_ms)
        {
            blinking = false;
            digitalWrite(LOADING_LED, HIGH);
        }
        else
        {
            int t = (now_ms - blinkStart) / sensorIdInterval;
            if(t % 2) {
                digitalWrite(LOADING_LED, LOW);
            } else {
                digitalWrite(LOADING_LED, HIGH);
            }
        }
        
    }
}