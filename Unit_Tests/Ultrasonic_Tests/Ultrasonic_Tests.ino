#include <Arduino.h>

// Pin configuration (update if needed)
#define ULTRASONIC_LEFT_TRIG_PIN  A6
#define ULTRASONIC_LEFT_ECHO_PIN  A4
#define ULTRASONIC_RIGHT_TRIG_PIN 8
#define ULTRASONIC_RIGHT_ECHO_PIN 11

long readUltrasonicCM(uint8_t trigPin, uint8_t echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH, 20000); // 20ms timeout
  long distance = duration / 58; // Convert to cm
  return distance;
}

void setup() {
  Serial.begin(115200);
  pinMode(ULTRASONIC_LEFT_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_LEFT_ECHO_PIN, INPUT);
  pinMode(ULTRASONIC_RIGHT_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_RIGHT_ECHO_PIN, INPUT);

  Serial.println("Ultrasonic Sensor Unit Test (Left & Right)");
}

void loop() {
  long leftDist = readUltrasonicCM(ULTRASONIC_LEFT_TRIG_PIN, ULTRASONIC_LEFT_ECHO_PIN);
  long rightDist = readUltrasonicCM(ULTRASONIC_RIGHT_TRIG_PIN, ULTRASONIC_RIGHT_ECHO_PIN);

  Serial.print("Left: ");
  Serial.print(leftDist);
  Serial.print(" cm | Right: ");
  Serial.print(rightDist);
  Serial.println(" cm");

  delay(500);
}