#include "stepper_driver.h"
#include <AccelStepper.h>
#include "config.h"

AccelStepper stepper(
    AccelStepper::FULL4WIRE,
    STEPPER_IN1_PIN, STEPPER_IN3_PIN, STEPPER_IN2_PIN, STEPPER_IN4_PIN
);

void initializeStepper() {
    stepper.setMaxSpeed(340); // ~10 RPM
    stepper.setAcceleration(200);
}

void handleStepperCommand(int rpm, int distance_mm, int flag) {
    if (flag == 1) {
        float returnRPM = 20.0;
        float returnSpeed = returnRPM * STEPPER_STEPS_PER_REV / 60.0;
        stepper.setAcceleration(80);
        stepper.setMaxSpeed(returnSpeed);
        long stepsToZero = -stepper.currentPosition();
        if (stepsToZero != 0) {
            stepper.move(stepsToZero);
            Serial.print("Stepper: Returning to zero position at 20 RPM, steps: ");
            Serial.println(stepsToZero);
        } else {
            Serial.println("Stepper: Already at zero position.");
        }
    } else {
        float pulley_circ = PULLEY_CIRCUM_MM;
        float revs = distance_mm / pulley_circ;
        long steps = (long)(revs * STEPPER_STEPS_PER_REV);

        float moveSpeed = abs(rpm) * STEPPER_STEPS_PER_REV / 60.0;
        stepper.setAcceleration(80);
        stepper.setMaxSpeed(moveSpeed);
        stepper.move((rpm >= 0) ? steps : -steps);

        Serial.print("Stepper: rpm=");
        Serial.print(rpm);
        Serial.print(" distance(mm)=");
        Serial.print(distance_mm);
        Serial.print(" steps=");
        Serial.println((rpm >= 0) ? steps : -steps);
    }
}

void runStepper() {
    stepper.run();
}