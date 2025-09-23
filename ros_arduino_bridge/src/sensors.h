#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>

// Ultrasonic
float microsecondsToCm(long microseconds);
long Ping(int trigPin, int echoPin);

// Color sensor
void initializeColorSensor();
void readColorSensor();
void colorSensorLEDOn();
void colorSensorLEDOff();

#endif // SENSORS_H