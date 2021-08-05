# SlimeVR Tracker firmware for ESP

Firmware for ESP32 microcontrollers and different IMU sensors to use them as a vive-like trackers in VR.

Requires [SlimeVR Server](https://github.com/SlimeVR/SlimeVR-Server) to work with SteamVR and resolve pose. Should be compatible with [owoTrack](https://github.com/abb128/owo-track-driver), but is not guaranteed.

## Compatibility

Compatible and tested with these IMUs (select during compilation):
* BNO085, BNO086
  * Using any fusion in internal DMP. Best results with ARVR Stabilized Game Rotation Vector or ARVR Stabilized Rotation Vector if in good magnetic environment
* BNO080
  * Using any fusion in internal DMP. Doesn't have BNO085's ARVR stabilization, but still gives good results.
* BNO055
  * Work in progress. Should be roughly equal BNO080, but cheaper
* MPU-9250
  * Using Mahony sensor fusion of Gyroscope, Magnetometer and Accelerometer, requires good magnetic environment. Implemented gyro/accl calibration on boot, mag calibration on boot with LED on which requires user to move sensor in "figure 8" for 15 seconds. 
* MPU-6500
  * Using internal DMP to fuse Gyroscope and Accelerometer, can be used with MPU-9250, can drift substantially
* MPU-6050
  * Same as MPU-6500

Firmware can work with both ESP8266 and ESP32, however this fork is focusing on ESP32 optimization for low cost sensors utilizing higher fusion rates and possibly the dual core cpu architecture. Please edit defines.h and set your pinout properly according to how you connected the IMU.
