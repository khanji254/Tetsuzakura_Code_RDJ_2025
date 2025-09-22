/***************************************************************
   Servo Sweep - by Nathaniel Gallinger

   Sweep servos one degree step at a time with a user defined
   delay in between steps.  Supports changing direction 
   mid-sweep.  Important for applications such as robotic arms
   where the stock servo speed is too fast for the strength
   of your system.

 *************************************************************/

#ifdef USE_SERVOS

#include <Servo.h>
#include "servos.h"
#include "config.h"

// Constructor
SweepServo::SweepServo()
{
  this->currentPositionDegrees = 0;
  this->targetPositionDegrees = 0;
  this->lastSweepCommand = 0;
  this->stepDelayMs = 10;
}

// Init
void SweepServo::initServo(
    int servoPin,
    int stepDelayMs,
    int initPosition)
{
  this->servo.attach(servoPin);
  this->stepDelayMs = stepDelayMs;
  this->currentPositionDegrees = initPosition;
  this->targetPositionDegrees = initPosition;
  this->lastSweepCommand = millis();
  this->servo.write(initPosition);
}

// Perform Sweep
void SweepServo::doSweep()
{
  // Get elapsed time
  int delta = millis() - this->lastSweepCommand;

  // Check if time for a step
  if (delta > this->stepDelayMs) {
    // Check step direction
    if (this->targetPositionDegrees > this->currentPositionDegrees) {
      this->currentPositionDegrees++;
      this->servo.write(this->currentPositionDegrees);
    }
    else if (this->targetPositionDegrees < this->currentPositionDegrees) {
      this->currentPositionDegrees--;
      this->servo.write(this->currentPositionDegrees);
    }
    // if target == current position, do nothing

    // reset timer
    this->lastSweepCommand = millis();
  }
}

// Set a new target position
void SweepServo::setTargetPosition(int position)
{
  this->targetPositionDegrees = position;
}

// Set a new target position and speed (ms per degree)
void SweepServo::setTargetPositionWithSpeed(int position, int speedMsPerDeg)
{
  this->targetPositionDegrees = position;
  this->stepDelayMs = speedMsPerDeg;
}

// Accessor for servo object
Servo SweepServo::getServo()
{
  return this->servo;
}

// --- Servo setup and loop helpers ---

void initAllServos() {
  for (int i = 0; i < N_SERVOS; i++) {
    servos[i].initServo(servoPins[i], stepDelay[i], servoInitPosition[i]);
  }
}

void setServoAngleWithSpeed(uint8_t index, int targetAngle, int speedMsPerDeg) {
  if (index >= N_SERVOS) return;
  servos[index].setTargetPositionWithSpeed(targetAngle, speedMsPerDeg);
}

void sweepAllServos() {
  for (int i = 0; i < N_SERVOS; i++) {
    servos[i].doSweep();
  }
}

#endif