#include "sensors.h"
#include "config.h"
#include <Wire.h>
#include <Adafruit_TCS34725.h>

// --- Ultrasonic Functions ---
float microsecondsToCm(long microseconds) {
  return microseconds / 29.0 / 2.0;
}

long Ping(int trigPin, int echoPin) {
  long duration, range;
  pinMode(trigPin, OUTPUT);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(trigPin, LOW);

  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH, ULTRASONIC_TIMEOUT);
  range = microsecondsToCm(duration);
  return range;
}

// --- Color Sensor Functions ---
Adafruit_TCS34725 tcs = Adafruit_TCS34725(
    TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void initializeColorSensor() {
  if (tcs.begin()) {
    Serial.println("TCS34725 found");
    pinMode(TCS34725_LED_PIN, OUTPUT);
    digitalWrite(TCS34725_LED_PIN, HIGH); // Turn on LED
  } else {
    Serial.println("No TCS34725 found ... check your wiring!");
  }
}

void readColorSensor() {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);
  Serial.print(r); Serial.print(" ");
  Serial.print(g); Serial.print(" ");
  Serial.println(b);
}