#include <Arduino.h>
#include <AccelStepper.h>

// === Stepper Motor Pin Definitions (ULN2003 driver) ===
#define STEPPER_IN1_PIN  45
#define STEPPER_IN2_PIN  47
#define STEPPER_IN3_PIN  49
#define STEPPER_IN4_PIN  51

// === Stepper Motor Constants ===
#define STEPPER_STEPS_PER_REV 2048  // 28BYJ-48: 32 steps * 64:1 gear ratio = 2048 steps/rev

// === Pulley and Travel Constants ===
#define PULLEY_DIAMETER_MM    30.0
#define PULLEY_CIRCUM_MM      (PI * PULLEY_DIAMETER_MM)

// === AccelStepper Setup ===
AccelStepper stepper(
    AccelStepper::FULL4WIRE,
    STEPPER_IN1_PIN, STEPPER_IN3_PIN, STEPPER_IN2_PIN, STEPPER_IN4_PIN
);

String inputBuffer = "";

void setup() {
  Serial.begin(115200);
  Serial.println("Send command: q <rpm>:<distance_mm>:<flag>");
  stepper.setMaxSpeed(150);       // Lowered for reliability (~4.4 RPM)
  stepper.setAcceleration(60);    // Lowered for reliability
}

void loop() {
  stepper.run(); // Non-blocking stepper update

  // Disable outputs when not moving to prevent heating
  if (!stepper.isRunning()) {
    stepper.disableOutputs();
  }

  // Read serial input
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (inputBuffer.length() > 0) {
        processCommand(inputBuffer);
        inputBuffer = "";
      }
    } else if (isPrintable(c)) {
      inputBuffer += c;
    }
  }
}

void processCommand(const String& cmd) {
  if (cmd.charAt(0) == 'q') {
    // Parse q <rpm>:<distance_mm>:<flag>
    int rpm = 0, distance = 0, flag = 0;
    int firstColon = cmd.indexOf(':');
    int secondColon = cmd.indexOf(':', firstColon + 1);

    if (firstColon > 0 && secondColon > firstColon) {
      rpm = cmd.substring(2, firstColon).toInt();
      distance = cmd.substring(firstColon + 1, secondColon).toInt();
      flag = cmd.substring(secondColon + 1).toInt();

      if (flag == 1) {
        // Return to original position (zero) at 20 RPM using relative move
        float returnRPM = 20.0;
        float returnSpeed = returnRPM * STEPPER_STEPS_PER_REV / 60.0; // steps/sec
        stepper.setAcceleration(60); // Lower acceleration for reliability
        stepper.setMaxSpeed(returnSpeed);
        long stepsToZero = -stepper.currentPosition();
        if (stepsToZero != 0) {
          stepper.enableOutputs();
          stepper.move(stepsToZero);
          Serial.print("Stepper: Returning to zero position at 20 RPM, steps: ");
          Serial.println(stepsToZero);
        } else {
          Serial.println("Stepper: Already at zero position.");
        }
      } else {
        // Calculate steps for travel
        float pulley_circ = PULLEY_CIRCUM_MM;
        float revs = distance / pulley_circ;
        long steps = (long)(revs * STEPPER_STEPS_PER_REV);

        // Set speed and move
        float moveSpeed = abs(rpm) * STEPPER_STEPS_PER_REV / 60.0;
        stepper.setAcceleration(60); // Lower acceleration for reliability
        stepper.setMaxSpeed(moveSpeed);
        stepper.enableOutputs();
        stepper.move((rpm >= 0) ? steps : -steps);

        Serial.print("Stepper: rpm=");
        Serial.print(rpm);
        Serial.print(" distance(mm)=");
        Serial.print(distance);
        Serial.print(" steps=");
        Serial.println((rpm >= 0) ? steps : -steps);
      }
    } else {
      Serial.println("Invalid format. Use: q <rpm>:<distance_mm>:<flag>");
    }
  } else {
    Serial.println("Unknown command.");
  }
}