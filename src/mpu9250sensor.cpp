/*
    SlimeVR Code is placed under the MIT license
    Copyright (c) 2021 Eiren Rain, S.J. Remington

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

#include "MPU9250.h"
#include "sensor.h"
#include "udpclient.h"
#include "defines.h"
#include "helper_3dmath.h"
#include <i2cscan.h>


#define gscale (250. / 32768.0) * (PI / 180.0) //gyro default 250 LSB per d/s -> rad/s
// These are the free parameters in the Mahony filter and fusion scheme,
// Kp for proportional feedback, Ki for integral
// with MPU-9250, angles start oscillating at Kp=40. Ki does not seem to help and is not required.
#define Kp 10.0
#define Ki 0.0

CalibrationConfig * calibration;

void get_MPU_scaled();
void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat);

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

// loop counter
long lastMillis = 0;
long loops = 0;
long quats = 0; 

void MPU9250Sensor::motionSetup(DeviceConfig * config) {
    calibration = &config->calibration;
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
    if (!imu.setup(0x68)) {  // change to your own address
        while (1) {
            Serial.println("MPU connection failed. Please check your connection with `connection_check` example.");
            delay(5000);
        }
    }
    imu.verbose(true);
    imu.selectFilter(QuatFilterSel::MAHONY);
    
    MPU9250Setting setting;
    setting.accel_fs_sel = ACCEL_FS_SEL::A2G;
    setting.gyro_fs_sel = GYRO_FS_SEL::G250DPS;
    setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
    setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_200HZ;
    setting.gyro_fchoice = 0x03;
    setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;
    setting.accel_fchoice = 0x01;
    setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;


    
    Serial.println("Accel Gyro calibration will start in 5sec.");
    Serial.println("Please leave the device still on the flat plane.");
    imu.verbose(true);
    delay(5000);
    imu.calibrateAccelGyro();
    
    Serial.println("Mag calibration will start in 5sec.");
    Serial.println("Please Wave device in a figure eight until done.");
    delay(5000);
    digitalWrite(LOADING_LED, HIGH);
    imu.calibrateMag();
    digitalWrite(LOADING_LED, LOW);
    Serial.println("< calibration parameters >");
    Serial.println("accel bias [g]: ");
    Serial.print(imu.getAccBiasX() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(imu.getAccBiasY() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.print(", ");
    Serial.print(imu.getAccBiasZ() * 1000.f / (float)MPU9250::CALIB_ACCEL_SENSITIVITY);
    Serial.println();
    Serial.println("gyro bias [deg/s]: ");
    Serial.print(imu.getGyroBiasX() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(imu.getGyroBiasY() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.print(", ");
    Serial.print(imu.getGyroBiasZ() / (float)MPU9250::CALIB_GYRO_SENSITIVITY);
    Serial.println();
    Serial.println("mag bias [mG]: ");
    Serial.print(imu.getMagBiasX());
    Serial.print(", ");
    Serial.print(imu.getMagBiasY());
    Serial.print(", ");
    Serial.print(imu.getMagBiasZ());
    Serial.println();
    Serial.println("mag scale []: ");
    Serial.print(imu.getMagScaleX());
    Serial.print(", ");
    Serial.print(imu.getMagScaleY());
    Serial.print(", ");
    Serial.print(imu.getMagScaleZ());
    Serial.println();
    Serial.print("i2C clock @ ");
    Serial.println(Wire.getClock());
    //wait for readability
    delay(5000);

    imu.selftest();
}

void MPU9250Sensor::motionLoop() {
    // Update quaternion
    now = micros();
    deltat = (now - last) * 1.0e-6; //seconds since last update
    last = now;
    // loop count
    long currentMillis = millis();
    loops++;
    
    imu.update();
    // send after x ms for filtertime
    static uint32_t prev_ms = millis();
    if (millis() > prev_ms + 10) {
        
        
        quaternion.x = imu.getQuaternionX();
        quaternion.y = imu.getQuaternionY();
        quaternion.z = imu.getQuaternionZ();
        quaternion.w = imu.getQuaternionW();
        quaternion *= sensorOffset;
        newData = true; //ensure sendData() only sends after filter runtime is exceeded
        //Serial.print("DBG: new data interval in s :");
        //Serial.println(deltat);
        prev_ms = millis(); 
    }
    
  if(currentMillis - lastMillis > 10000){
    Serial.print("[DBG] Average Motion Loops per second:");
    Serial.println(loops/10);
    Serial.print("[DBG] Quaternions per second:");
    Serial.println(quats/10);
    
    lastMillis = currentMillis;
    loops = 0;
    quats = 0;
  }
    
}

void MPU9250Sensor::sendData() {
    if(newData) {
        newData = false;
    sendQuat(&quaternion, PACKET_ROTATION);
    quats++;
    }
    //sendVector(rawMag, PACKET_RAW_MAGENTOMETER);
    //sendVector(Axyz, PACKET_ACCEL);
    //sendVector(Mxyz, PACKET_MAG);
}

void MPU9250Sensor::getMPUScaled()
{
    float temp[3];
    int i;
    //imu.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
    

    //sascha no calibration implemented
    /*
    Gxyz[0] = ((float)gx - calibration->G_off[0]) * gscale; //250 LSB(d/s) default to radians/s
    Gxyz[1] = ((float)gy - calibration->G_off[1]) * gscale;
    Gxyz[2] = ((float)gz - calibration->G_off[2]) * gscale;
    */
    Gxyz[0] = ((float)gx ) * gscale; //250 LSB(d/s) default to radians/s
    Gxyz[1] = ((float)gy ) * gscale;
    Gxyz[2] = ((float)gz ) * gscale;


    Axyz[0] = (float)ax;
    Axyz[1] = (float)ay;
    Axyz[2] = (float)az;
    //apply offsets (bias) and scale factors from Magneto
    // sascha no calibration implemented
    /*
    if(useFullCalibrationMatrix) {
        for (i = 0; i < 3; i++)
            temp[i] = (Axyz[i] - calibration->A_B[i]);
        Axyz[0] = calibration->A_Ainv[0][0] * temp[0] + calibration->A_Ainv[0][1] * temp[1] + calibration->A_Ainv[0][2] * temp[2];
        Axyz[1] = calibration->A_Ainv[1][0] * temp[0] + calibration->A_Ainv[1][1] * temp[1] + calibration->A_Ainv[1][2] * temp[2];
        Axyz[2] = calibration->A_Ainv[2][0] * temp[0] + calibration->A_Ainv[2][1] * temp[1] + calibration->A_Ainv[2][2] * temp[2];
    } else {
        for (i = 0; i < 3; i++)
            Axyz[i] = (Axyz[i] - calibration->A_B[i]);
    }
    */

    //vector_normalize(Axyz);

    Mxyz[0] = (float)mx;
    Mxyz[1] = (float)my;
    Mxyz[2] = (float)mz;
    //apply offsets and scale factors from Magneto
    //sascha no calibration implemented
    /*   
    if(useFullCalibrationMatrix) {
        for (i = 0; i < 3; i++)
            temp[i] = (Mxyz[i] - calibration->M_B[i]);
        Mxyz[0] = calibration->M_Ainv[0][0] * temp[0] + calibration->M_Ainv[0][1] * temp[1] + calibration->M_Ainv[0][2] * temp[2];
        Mxyz[1] = calibration->M_Ainv[1][0] * temp[0] + calibration->M_Ainv[1][1] * temp[1] + calibration->M_Ainv[1][2] * temp[2];
        Mxyz[2] = calibration->M_Ainv[2][0] * temp[0] + calibration->M_Ainv[2][1] * temp[1] + calibration->M_Ainv[2][2] * temp[2];
    } else {
        for (i = 0; i < 3; i++)
            Mxyz[i] = (Mxyz[i] - calibration->M_B[i]);
    }
    */
    rawMag[0] = Mxyz[0];
    rawMag[1] = Mxyz[1];
    rawMag[2] = Mxyz[2];
    //vector_normalize(Mxyz);
}

// Mahony orientation filter, assumed World Frame NWU (xNorth, yWest, zUp)
// Modified from Madgwick version to remove Z component of magnetometer:
// reference vectors are Up (Acc) and West (Acc cross Mag)
// sjr 12/2020
// input vectors ax, ay, az and mx, my, mz MUST be normalized!
// gx, gy, gz must be in units of radians/second
//
void MPU9250Sensor::MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float deltat)
{
    // Vector to hold integral error for Mahony method
    static float eInt[3] = {0.0, 0.0, 0.0};

    // short name local variable for readability
    float q1 = q[0], q2 = q[1], q3 = q[2], q4 = q[3];
    float norm;
    float hx, hy, hz;  //observed West vector W = AxM
    float ux, uy, uz, wx, wy, wz; //calculated A (Up) and W in body frame
    float ex, ey, ez;
    float pa, pb, pc;

    // Auxiliary variables to avoid repeated arithmetic
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Measured horizon vector = a x m (in body frame)
    hx = ay * mz - az * my;
    hy = az * mx - ax * mz;
    hz = ax * my - ay * mx;
    // Normalise horizon vector
    norm = sqrt(hx * hx + hy * hy + hz * hz);
    if (norm == 0.0f) return; // Handle div by zero

    norm = 1.0f / norm;
    hx *= norm;
    hy *= norm;
    hz *= norm;

    // Estimated direction of Up reference vector
    ux = 2.0f * (q2q4 - q1q3);
    uy = 2.0f * (q1q2 + q3q4);
    uz = q1q1 - q2q2 - q3q3 + q4q4;

    // estimated direction of horizon (West) reference vector
    wx = 2.0f * (q2q3 + q1q4);
    wy = q1q1 - q2q2 + q3q3 - q4q4;
    wz = 2.0f * (q3q4 - q1q2);

    // Error is cross product between estimated direction and measured direction of the reference vectors
    ex = (ay * uz - az * uy) + (hy * wz - hz * wy);
    ey = (az * ux - ax * uz) + (hz * wx - hx * wz);
    ez = (ax * uy - ay * ux) + (hx * wy - hy * wx);

    if (Ki > 0.0f)
    {
        eInt[0] += ex; // accumulate integral error
        eInt[1] += ey;
        eInt[2] += ez;
        // Apply I feedback
        gx += Ki * eInt[0];
        gy += Ki * eInt[1];
        gz += Ki * eInt[2];
    }

    // Apply P feedback
    gx = gx + Kp * ex;
    gy = gy + Kp * ey;
    gz = gz + Kp * ez;

    // Integrate rate of change of quaternion
    pa = q2;
    pb = q3;
    pc = q4;
    q1 = q1 + (-q2 * gx - q3 * gy - q4 * gz) * (0.5f * deltat);
    q2 = pa + (q1 * gx + pb * gz - pc * gy) * (0.5f * deltat);
    q3 = pb + (q1 * gy - pa * gz + pc * gx) * (0.5f * deltat);
    q4 = pc + (q1 * gz + pa * gy - pb * gx) * (0.5f * deltat);

    // Normalise quaternion
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);
    norm = 1.0f / norm;
    q[0] = q1 * norm;
    q[1] = q2 * norm;
    q[2] = q3 * norm;
    q[3] = q4 * norm;
}

void MPU9250Sensor::startCalibration(int calibrationType) {
    //tbd
}