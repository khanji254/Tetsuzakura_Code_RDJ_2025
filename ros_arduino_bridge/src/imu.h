#ifndef IMU_H
#define IMU_H

#include <Arduino.h>

#define IMU_INIT_TIMEOUT 3000  // 3 second timeout for IMU initialization

bool initializeIMU();  // Returns true if successful, false if failed/timeout
float readIMUZAngle(); // Returns Z (yaw) angle in degrees
bool isIMUReady();     // Check if IMU is ready for use
// Read full IMU packet: yaw pitch roll gyro_x gyro_y gyro_z accel_x accel_y accel_z
// Values returned in an array of 9 floats
void readIMUFull(float outVals[9]);

#endif