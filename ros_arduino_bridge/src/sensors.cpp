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
  delayMicroseconds(10); // Use 10us like your working code
  digitalWrite(trigPin, LOW);

  pinMode(echoPin, INPUT);
  duration = pulseIn(echoPin, HIGH, 20000); // Use 20ms timeout like your working code
  range = duration / 58; // Use the same conversion as your working code

  // Debug print
  //Serial.print("Ping trig: "); 
   Serial.print(trigPin);
 // Serial.print(" echo: "); 
   Serial.print(echoPin);
 // Serial.print(" duration: "); Serial.print(duration);
 // Serial.print(" range: "); Serial.println(range);

  return range;
  //delay(500); // Short delay to avoid sensor interference
}
// --- Color Sensor Functions ---
Adafruit_TCS34725 tcs = Adafruit_TCS34725(
    TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void initializeColorSensor() {
  if (tcs.begin()) {
    //Serial.println("TCS34725 found");
    pinMode(TCS34725_LED_PIN, OUTPUT);
    digitalWrite(TCS34725_LED_PIN, LOW); // Turn on LED
  } else {
    //Serial.println("No TCS34725 found ... check your wiring!");
  }
}

void readColorSensor() {
  uint16_t r, g, b, c;
  tcs.getRawData(&r, &g, &b, &c);
  Serial.print(r); Serial.print(" ");
  Serial.print(g); Serial.print(" ");
  Serial.println(b);
}

void colorSensorLEDOn() {
    digitalWrite(TCS34725_LED_PIN, HIGH);
}

void colorSensorLEDOff() {
    digitalWrite(TCS34725_LED_PIN, LOW);
}