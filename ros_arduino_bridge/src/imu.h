#ifndef IMU_H
#define IMU_H

#include <Arduino.h>

void initializeIMU();
float readIMUZAngle(); // Returns Z (yaw) angle in degrees

#endif