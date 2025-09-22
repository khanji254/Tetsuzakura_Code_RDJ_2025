/***************************************************************
   Motor driver definitions for TB6612
   *************************************************************/
#define TB6612_MOTOR_DRIVER


#ifdef USE_BASE

#ifdef TB6612_MOTOR_DRIVER

#include "config.h"

// Helper arrays for pin mapping
const int pwmPins[4] = {M1_PWM, M2_PWM, M3_PWM, M4_PWM};
const int in1Pins[4] = {M1_IN1, M2_IN1, M3_IN1, M4_IN1};
const int in2Pins[4] = {M1_IN2, M2_IN2, M3_IN2, M4_IN2};

void initMotorControllerTB6612() {
  pinMode(MOTOR_STBY, OUTPUT);
  digitalWrite(MOTOR_STBY, HIGH); // Enable TB6612

  for (int i = 0; i < 4; i++) {
    pinMode(pwmPins[i], OUTPUT);
    pinMode(in1Pins[i], OUTPUT);
    pinMode(in2Pins[i], OUTPUT);
    analogWrite(pwmPins[i], 0);
    digitalWrite(in1Pins[i], LOW);
    digitalWrite(in2Pins[i], LOW);
  }
}

void setMotorSpeedTB6612(int motor, int speed) {
  if (motor < 0 || motor > 3) return;
  int pwm = constrain(abs(speed), 0, MOTOR_MAX_SPEED);

  if (speed > 0) {
    digitalWrite(in1Pins[motor], HIGH);
    digitalWrite(in2Pins[motor], LOW);
  } else if (speed < 0) {
    digitalWrite(in1Pins[motor], LOW);
    digitalWrite(in2Pins[motor], HIGH);
  } else {
    digitalWrite(in1Pins[motor], LOW);
    digitalWrite(in2Pins[motor], LOW);
  }
  analogWrite(pwmPins[motor], pwm);
}

void setMotorSpeedsTB6612(int flSpeed, int frSpeed, int rlSpeed, int rrSpeed) {
  setMotorSpeedTB6612(0, flSpeed);
  setMotorSpeedTB6612(1, frSpeed);
  setMotorSpeedTB6612(2, rlSpeed);
  setMotorSpeedTB6612(3, rrSpeed);
}

#endif // TB6612_MOTOR_DRIVER

#endif // USE_BASE