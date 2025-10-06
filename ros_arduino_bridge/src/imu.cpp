#include "imu.h"
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>

MPU6050 mpu;

bool dmpReady = false;
bool imuInitialized = false;
uint8_t devStatus;
uint16_t packetSize;
uint8_t fifoBuffer[64];

Quaternion q;
VectorFloat gravity;
float ypr[3];

bool initializeIMU() {
    unsigned long startTime = millis();
    
    Serial.println(F("[IMU] Initializing..."));
    
    Wire.begin();
    Wire.setClock(400000);
    Wire.setWireTimeout(5000, true);  // 5ms I2C timeout

    // Initialize MPU6050
    mpu.initialize();
    
    // Test connection with timeout
    Serial.print(F("[IMU] Testing connection..."));
    unsigned long testStart = millis();
    bool connectionOK = false;
    
    while (millis() - testStart < 1000) {  // 1 second timeout for connection test
        if (mpu.testConnection()) {
            connectionOK = true;
            break;
        }
        delay(100);
    }
    
    if (!connectionOK) {
        Serial.println(F(" FAILED (timeout)"));
        Serial.println(F("[IMU] Not detected - continuing without IMU"));
        dmpReady = false;
        imuInitialized = false;
        return false;
    }
    
    Serial.println(F(" OK"));
    
    // Check if we're approaching timeout
    if (millis() - startTime > IMU_INIT_TIMEOUT - 2000) {
        Serial.println(F("[IMU] Timeout approaching - skipping full initialization"));
        dmpReady = false;
        imuInitialized = false;
        return false;
    }

    // Initialize DMP
    Serial.print(F("[IMU] Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    if (devStatus != 0) {
        Serial.print(F(" FAILED (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
        dmpReady = false;
        imuInitialized = false;
        return false;
    }
    
    Serial.println(F(" OK"));

    // Set gyro and accel offsets
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788);

    // Calibrate (skip if approaching timeout)
    if (millis() - startTime < IMU_INIT_TIMEOUT - 1500) {
        Serial.print(F("[IMU] Calibrating..."));
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        Serial.println(F(" OK"));
        mpu.PrintActiveOffsets();
    } else {
        Serial.println(F("[IMU] Skipping calibration due to timeout"));
    }

    // Enable DMP
    Serial.print(F("[IMU] Enabling DMP..."));
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    dmpReady = true;
    imuInitialized = true;
    
    Serial.println(F(" OK"));
    Serial.print(F("[IMU] Initialization complete in "));
    Serial.print(millis() - startTime);
    Serial.println(F("ms"));
    
    return true;
}

float readIMUZAngle() {
    if (!dmpReady) return 0.0;
    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        return -ypr[0] * 180.0 / M_PI; // Z (yaw) in degrees, inverted so left is negative
    }
    return 0.0;
}

bool isIMUReady() {
    return dmpReady && imuInitialized;
}

void readIMUFull(float outVals[9]) {
    // Initialize output to zeros
    for (int i = 0; i < 9; i++) outVals[i] = 0.0f;

    if (!dmpReady) return;

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
        // Yaw/Pitch/Roll
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        // Invert yaw so left is negative (consistent with existing behavior)
        outVals[0] = -ypr[0] * 180.0f / M_PI; // yaw
        outVals[1] = ypr[1] * 180.0f / M_PI;  // pitch
        outVals[2] = ypr[2] * 180.0f / M_PI;  // roll

        // Raw gyroscope and accelerometer readings are not directly provided by the DMP
        // Extract gyro and accel from FIFO as integers via helper functions
        int16_t gx, gy, gz, ax, ay, az;
        // Use I2Cdev helper to read the sensor registers if available
        // Read gyroscope
        gx = mpu.getRotationX();
        gy = mpu.getRotationY();
        gz = mpu.getRotationZ();
        // Read accelerometer
        ax = mpu.getAccelerationX();
        ay = mpu.getAccelerationY();
        az = mpu.getAccelerationZ();

        // Convert raw values to sensible units (deg/s for gyro if desired)
        // MPU6050 default FS is +/- 250 deg/s and +/- 2g for accel in many modules
        // But since full-scale settings are unknown here, return raw ints for now
        outVals[3] = (float)gx;
        outVals[4] = (float)gy;
        outVals[5] = (float)gz;
        outVals[6] = (float)ax;
        outVals[7] = (float)ay;
        outVals[8] = (float)az;
    }
}