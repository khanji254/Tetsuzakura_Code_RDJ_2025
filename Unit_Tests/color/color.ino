#include <Wire.h>
#include <Adafruit_TCS34725.h>

// Pin definitions
#define TCS34725_LED_PIN  A3    // LED control pin
#define TCS34725_INT_PIN  A0    // Interrupt pin (not used for basic RGB read)

// Create the color sensor object
Adafruit_TCS34725 tcs = Adafruit_TCS34725(
    TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

void setup() {
  Serial.begin(115200);
  Serial.println("TCS34725 Color Sensor Test - Send 'C' to get RGB");

  pinMode(TCS34725_LED_PIN, OUTPUT);
  digitalWrite(TCS34725_LED_PIN, HIGH); // Turn on LED

  pinMode(TCS34725_INT_PIN, INPUT_PULLUP); // Interrupt pin as input (not used here)

  if (tcs.begin()) {
    Serial.println("TCS34725 found");
  } else {
    Serial.println("No TCS34725 found ... check your wiring!");
    while (1); // Halt
  }
}

void loop() {
  // Wait for a serial command
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd == 'C' || cmd == 'c') {
      uint16_t r, g, b, c;
      tcs.getRawData(&r, &g, &b, &c);
      Serial.print(r); Serial.print(" ");
      Serial.print(g); Serial.print(" ");
      Serial.println(b);
    }
    delay(50);
  }
}