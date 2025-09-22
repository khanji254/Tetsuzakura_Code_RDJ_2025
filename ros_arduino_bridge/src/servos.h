#ifndef SERVOS_H
#define SERVOS_H

#define N_SERVOS 2

// Servo indexes for clarity
#define CAMERA_SERVO_INDEX 0
#define TIPPER_SERVO_INDEX 1

// This delay in milliseconds determines the pause 
// between each one degree step the servo travels.  
int stepDelay [N_SERVOS] = { 0, 0 }; // ms

// Pins (as per config.h)
byte servoPins [N_SERVOS] = { CAMERA_SERVO_PIN, TIPPER_SERVO_PIN };

// Initial Position
byte servoInitPosition [N_SERVOS] = { 90, 90 }; // [0, 180] degrees

class SweepServo
{
  public:
    SweepServo();
    void initServo(
        int servoPin,
        int stepDelayMs,
        int initPosition);
    void doSweep();
    void setTargetPosition(int position);
    Servo getServo();

  private:
    Servo servo;
    int stepDelayMs;
    int currentPositionDegrees;
    int targetPositionDegrees;
    long lastSweepCommand;
};

SweepServo servos [N_SERVOS];

#endif