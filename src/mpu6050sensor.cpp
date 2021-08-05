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

//#include "MPU9250.h"
#include "sensor.h"
#include "udpclient.h"
#include <i2cscan.h>

namespace {
    void signalAssert() {
        for(int i = 0; i < 200; ++i) {
            delay(50);
            digitalWrite(LOADING_LED, LOW);
            delay(50);
            digitalWrite(LOADING_LED, HIGH);
        }
    }
}

bool hasNewData = false;

void gatherCalibrationData(MPU9250 &imu);

void MPU6050Sensor::motionSetup(DeviceConfig * config) {
    /*
    uint8_t addr = 0x68;
    if(!I2CSCAN::isI2CExist(addr)) {
        addr = 0x69;
        if(!I2CSCAN::isI2CExist(addr)) {
            Serial.println("Can't find I2C device on addr 0x4A or 0x4B, scanning for all I2C devices and returning");
            I2CSCAN::scani2cports();
            signalAssert();
            return;
        }
    }
    // initialize device
    imu.initialize(addr);
/////
   
        // Do a quick and dirty calibration. As the imu warms up the offsets will change a bit, but this will be good-enough
        Serial.println("Put down the device and wait for baseline gyro reading calibration");
        delay(2000);
        digitalWrite(LOADING_LED, HIGH);
        imu.CalibrateAccel(6);
        imu.CalibrateGyro(6);
        imu.PrintActiveOffsets();
        digitalWrite(LOADING_LED, LOW);

///


    if(!imu.testConnection()) {
        Serial.print("Can't communicate with MPU9250, response ");
        Serial.println(imu.getDeviceID(), HEX);
    }
    devStatus = imu.dmpInitialize();
    // sascha - disable as calibration not working

    //imu.setXGyroOffset(config->calibration.G_off[0]);
    //imu.setYGyroOffset(config->calibration.G_off[1]);
    //imu.setZGyroOffset(config->calibration.G_off[2]);
    //imu.setXAccelOffset(config->calibration.A_B[0]);
    //imu.setYAccelOffset(config->calibration.A_B[1]);
    //imu.setZAccelOffset(config->calibration.A_B[2]);

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        
     

// turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        imu.setDMPEnabled(true);

        mpuIntStatus = imu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = imu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
*/    
}

void MPU6050Sensor::motionLoop() {
  /*
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    mpuIntStatus = imu.getIntStatus();

    // get current FIFO count
    fifoCount = imu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        imu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

        // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = imu.getFIFOCount();

        // read a packet from FIFO
        imu.getFIFOBytes(fifoBuffer, packetSize);

        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        imu.dmpGetQuaternion(&rawQuat, fifoBuffer);
        q[0] = rawQuat.x;
        q[1] = rawQuat.y;
        q[2] = rawQuat.z;
        q[3] = rawQuat.w;
        quaternion.set(-q[1], q[0], q[2], q[3]);
        quaternion *= sensorOffset;
        hasNewData = true;
    }
    */
}

void MPU6050Sensor::sendData() {
    if(hasNewData) {
        sendQuat(&quaternion, PACKET_ROTATION);
        hasNewData = false;
    }
}

void MPU6050Sensor::startCalibration(int calibrationType) {
    digitalWrite(CALIBRATING_LED, LOW);
    Serial.println("Starting offset finder");
    gatherCalibrationData(imu);
    Serial.println("Process is over");
    digitalWrite(CALIBRATING_LED, HIGH);
}

void gatherCalibrationData(MPU9250 &imu) {
    /*
    Serial.println("Gathering raw data for device calibration...");
    int calibrationSamples = 500;
    // Reset values
    float Gxyz[3];
    int16_t gx;
    int16_t gy;
    int16_t gz;
    int16_t ax;
    int16_t ay;
    int16_t az;

    // Wait for sensor to calm down before calibration
    Serial.println("Put down the device and wait for baseline gyro reading calibration");
    delay(2000);
    for (int i = 0; i < calibrationSamples; i++)
    {
        imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
        Gxyz[0] += float(gx);
        Gxyz[1] += float(gy);
        Gxyz[2] += float(gz);
        delay(10);
    }
    Gxyz[0] /= calibrationSamples;
    Gxyz[1] /= calibrationSamples;
    Gxyz[2] /= calibrationSamples;
    Serial.printf("Gyro calibration results: %f %f %f\n", Gxyz[0], Gxyz[1], Gxyz[2]);
    sendVector(Gxyz, PACKET_GYRO_CALIBRATION_DATA);

    // Blink calibrating led before user should rotate the sensor
    Serial.println("Gently rotate the device while it's gathering accelerometer data");
    for (int i = 0; i < 3000 / 310; ++i)
    {
        digitalWrite(CALIBRATING_LED, LOW);
        delay(15);
        digitalWrite(CALIBRATING_LED, HIGH);
        delay(300);
    }
    int calibrationData[6];
    for (int i = 0; i < calibrationSamples; i++)
    {
        digitalWrite(CALIBRATING_LED, LOW);
        imu.getAcceleration(&ax, &ay, &az);
        calibrationData[0] = ax;
        calibrationData[1] = ay;
        calibrationData[2] = az;
        calibrationData[3] = 0;
        calibrationData[4] = 0;
        calibrationData[5] = 0;
        sendRawCalibrationData(calibrationData, PACKET_RAW_CALIBRATION_DATA);
        digitalWrite(CALIBRATING_LED, HIGH);
        delay(50);
    }
    Serial.println("Calibration data gathered and sent");
*/
}