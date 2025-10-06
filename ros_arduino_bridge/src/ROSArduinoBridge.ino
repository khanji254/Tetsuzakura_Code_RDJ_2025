/*********************************************************************
 *  ROSArduinoBridge

    A set of simple serial commands to control a differential drive
    robot and receive back sensor and odometry data. Default 
    configuration assumes use of an Arduino Mega + Pololu motor
    controller shield + Robogaia Mega Encoder shield.  Edit the
    readEncoder() and setMotorSpeed() wrapper functions if using 
    different motor controller or encoder method.

    Created for the Pi Robot Project: http://www.pirobot.org
    and the Home Brew Robotics Club (HBRC): http://hbrobotics.org
    
    Authors: Patrick Goebel, James Nugen

    Inspired and modeled after the ArbotiX driver by Michael Ferguson
    
    Software License Agreement (BSD License)

    Copyright (c) 2012, Patrick Goebel.
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

     * Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following discla                   
     * Redistributions in binary form must reproduce the above
       copyright notice, this list of conditions and the following
       disclaimer in the documentation and/or other materials provided
       with the distribution.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#define USE_BASE      // Enable the base controller code
#define USE_STEPPER      // Enable the IMU code if an IMU is connected
/* Define the motor controller and encoder library you are using */
#ifdef USE_BASE
   /* The Pololu VNH5019 dual motor driver shield */
   //#define POLOLU_VNH5019

   /* The Pololu MC33926 dual motor driver shield */
   //#define POLOLU_MC33926

   /* The RoboGaia encoder shield */
   //#define ROBOGAIA
   
   /* Encoders directly attached to Arduino board */
   #define ARDUINO_ENC_COUNTER

   /* TB6612 Motor driver */
   #define TB6612_MOTOR_DRIVER
#endif

#define USE_SERVOS  // Enable use of PWM servos as defined in servos.h

/* Serial port baud rate */
#define BAUDRATE     57600

/* Maximum PWM signal */
#define MAX_PWM        255

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

/* Include definition of serial commands */
#include "commands.h"

/* Include configuration parameters */
#include "config.h"
#include "sensors.h"
#include "encoder_driver.h"
#include "stepper_driver.h"
#include "imu.h"

/* Sensor functions */
#include "sensors.h"

/* Include servo support if required */
#ifdef USE_SERVOS
   #include <Servo.h>
   #include "servos.h"
#endif

#ifdef USE_BASE
  /* Motor driver function definitions */
  #include "motor_driver.h"

  /* Encoder driver function definitions */
  #include "encoder_driver.h"

  /* PID parameters and functions */
  #include "diff_controller.h"

  /* Run the PID loop at 50 times per second */
  #define PID_RATE           50     // Hz (increased from 30Hz for better odometry)

  /* Convert the rate into an interval */
  const int PID_INTERVAL = 1000 / PID_RATE;
  
  /* Track the next time we make a PID calculation */
  unsigned long nextPID = PID_INTERVAL;

  /* Stop the robot if it hasn't received a movement command
   in this number of milliseconds */
  #define AUTO_STOP_INTERVAL 2000
  long lastMotorCommand = AUTO_STOP_INTERVAL;
#endif

/* State Machine Definitions */
typedef enum {
  STATE_OFFLOADING,   // Offloading state - minimal data transmission
  STATE_MOVING,       // Moving state - send all sensor data
  STATE_STATIONARY    // Stationary state - limited data transmission
} RobotState;

// Current state variable
RobotState currentState = STATE_STATIONARY;

// State detection variables
bool motorMoving = false;
bool offloadingActive = false;
unsigned long lastMovementTime = 0;
unsigned long lastStepperActivity = 0;
#define MOVEMENT_THRESHOLD 15       // Minimum encoder count change to consider moving (increased for mecanum stability)
#define STATIONARY_TIME_MS 500      // Time without movement to be considered stationary
#define STEPPER_ACTIVITY_TIMEOUT 1000 // Time after stepper activity to consider offloading (reduced from 3000ms)

// Previous encoder values for movement detection
long lastM1 = 0, lastM2 = 0, lastM3 = 0, lastM4 = 0;

/* State Machine Functions */
void updateStateMachine() {
  // Check for stepper activity (offloading)
  if (isStepperActive()) {
    lastStepperActivity = millis();
    offloadingActive = true;
  }
  
  // Check if we're still in offloading state (within timeout after stepper activity)
  if (offloadingActive && (millis() - lastStepperActivity < STEPPER_ACTIVITY_TIMEOUT)) {
    currentState = STATE_OFFLOADING;
    return;
  } else {
    offloadingActive = false;
  }
  
  // Check if motors are moving by comparing encoder values
  long currentM1 = getM1Encoder();
  long currentM2 = getM2Encoder();
  long currentM3 = getM3Encoder();
  long currentM4 = getM4Encoder();
  
  // Check if any encoder has changed significantly
  bool encodersMoving = (abs(currentM1 - lastM1) > MOVEMENT_THRESHOLD) ||
                       (abs(currentM2 - lastM2) > MOVEMENT_THRESHOLD) ||
                       (abs(currentM3 - lastM3) > MOVEMENT_THRESHOLD) ||
                       (abs(currentM4 - lastM4) > MOVEMENT_THRESHOLD);
  
  if (encodersMoving) {
    motorMoving = true;
    lastMovementTime = millis();
  } else if (motorMoving && (millis() - lastMovementTime > STATIONARY_TIME_MS)) {
    motorMoving = false;
  }
  
  // Update state based on movement
  if (motorMoving) {
    currentState = STATE_MOVING;
  } else {
    currentState = STATE_STATIONARY;
  }
  
  // Save current encoder values for next comparison
  lastM1 = currentM1;
  lastM2 = currentM2;
  lastM3 = currentM3;
  lastM4 = currentM4;
}

/* Data Transmission Control Functions */
bool shouldSendOdometry() {
  return currentState != STATE_OFFLOADING;  // Always send unless offloading
}

bool shouldSendIMUData() {
  return currentState != STATE_OFFLOADING;  // Always send unless offloading
}

bool shouldSendEncoderData() {
  // Send encoder data in all states except offloading
  return currentState != STATE_OFFLOADING;
}

bool shouldSendSensorData() {
  // Send sensor data in all states except offloading
  return currentState != STATE_OFFLOADING;
}

/* Variable initialization */

// A pair of varibles to help parse serial commands (thanks Fergs)
int arg = 0;
int index = 0;

// Variable to hold an input character
char chr;

// Variable to hold the current single-character command
char cmd;

// Character arrays to hold the first and second arguments
char argv1[16];
char argv2[16];

// The arguments converted to integers
long arg1;
long arg2;

/* Clear the current command parameters */
void resetCommand() {
  cmd = 0;
  memset(argv1, 0, sizeof(argv1));
  memset(argv2, 0, sizeof(argv2));
  arg1 = 0;
  arg2 = 0;
  arg = 0;
  index = 0;
}

/* Run a command.  Commands are defined in commands.h */
int runCommand() {
  int i = 0;
  char *p = argv1;
  char *str;
  int pid_args[4];
  arg1 = atoi(argv1);
  arg2 = atoi(argv2);

  switch(cmd) {
  case GET_BAUDRATE:
    Serial.println(BAUDRATE);
    break;
  case ANALOG_READ:
    Serial.println(analogRead(arg1));
    break;
  case DIGITAL_READ:
    Serial.println(digitalRead(arg1));
    break;
  case ANALOG_WRITE:
    analogWrite(arg1, arg2);
    Serial.println("OK"); 
    break;
  case DIGITAL_WRITE:
    if (arg2 == 0) digitalWrite(arg1, LOW);
    else if (arg2 == 1) digitalWrite(arg1, HIGH);
    Serial.println("OK"); 
    break;
  case PIN_MODE:
    if (arg2 == 0) pinMode(arg1, INPUT);
    else if (arg2 == 1) pinMode(arg1, OUTPUT);
    Serial.println("OK");
    break;
  case PING_BOT:
    Serial.println(Ping(arg1, arg2)); // Now expects trigPin, echoPin
    break;
#ifdef USE_SERVOS
  case SERVO_WRITE:
    servos[arg1].setTargetPosition(arg2);
    Serial.println("OK");
    break;
  case SERVO_READ:
    Serial.println(servos[arg1].getServo().read());
    break;
  case SERVO_WRITE_SPEED: // w <index> <angle> <speed>
    {
      int idx = atoi(argv1);
      int angle = atoi(argv2);
      // Read speed from serial (third argument)
      int speed = 10; // default speed
      char buf[8] = {0};
      int bidx = 0;
      while (Serial.available() > 0 && bidx < 7) {
        char c = Serial.read();
        if (c == 13 || c == ' ') break;
        buf[bidx++] = c;
      }
      buf[bidx] = 0;
      if (bidx > 0) speed = atoi(buf);
      setServoAngleWithSpeed(idx, angle, speed);
      Serial.println("OK");
    }
    break;    
#endif

#ifdef USE_BASE
  case READ_ENCODERS:
    if (shouldSendEncoderData()) {
      printAllEncoders();
    }
    // Don't send anything if encoders shouldn't transmit
    break;
  case RESET_ENCODERS:
    resetEncoders();
    resetPID();
    // Reset movement detection variables
    lastM1 = lastM2 = lastM3 = lastM4 = 0;
    Serial.println("OK");
    break;
  case MOTOR_SPEEDS:
  case MOTOR_RAW_PWM:
    {
      /* Reset the auto stop timer */
      lastMotorCommand = millis();
      int speeds[4] = {0, 0, 0, 0};
      // Combine argv1 and argv2 and read the rest of the line
      char args[32] = {0};
      strncpy(args, argv1, sizeof(args) - 1);
      if (strlen(argv2) > 0) {
        strncat(args, ":", sizeof(args) - strlen(args) - 1);
        strncat(args, argv2, sizeof(args) - strlen(args) - 1);
      }
      // Read the rest of the line if available
      while (Serial.available() > 0 && strlen(args) < sizeof(args) - 1) {
        char c = Serial.read();
        if (c == 13 || c == '\n' || c == '\r') break;
        strncat(args, &c, 1);
      }
      // Try splitting on ':'
      char *q = args;
      int idx = 0;
      while ((str = strtok_r(q, ":", &q)) && idx < 4) {
        speeds[idx++] = atoi(str);
      }
      // If not enough values, try splitting on spaces
      if (idx < 4) {
        q = args;
        idx = 0;
        while ((str = strtok_r(q, " ", &q)) && idx < 4) {
          speeds[idx++] = atoi(str);
        }
      }
      
      // COMMAND-BASED MOVEMENT DETECTION
      bool anyMotorActive = (speeds[0] != 0 || speeds[1] != 0 || 
                            speeds[2] != 0 || speeds[3] != 0);
      if (anyMotorActive) {
        motorMoving = true;
        lastMovementTime = millis();
      }
      
      setMotorSpeedsTB6612(speeds[0], speeds[1], speeds[2], speeds[3]);
      Serial.println("OK");
      if (cmd == MOTOR_RAW_PWM) {
        resetPID();
        moving = 0;
      }
    }
    break;
    case GET_IMU_ANGLE: // IMU full data command (i)
       if (shouldSendIMUData()) {
         float imuVals[9] = {0};
         readIMUFull(imuVals);
         // Print values space-separated: yaw pitch roll gx gy gz ax ay az
         for (int i = 0; i < 9; i++) {
           if (i > 0) Serial.print(' ');
           Serial.print(imuVals[i]);
         }
         Serial.println();
       }
       // Don't send anything if IMU shouldn't transmit
     break;

    case STEPPER: {
        // Example: q -25:400:0
        int rpm = 0, distance = 0, flag = 0;
        char *p = argv1;
        char *str;
        int params[3] = {0, 0, 0};
        int idx = 0;
        // Parse colon-separated values
        while ((str = strtok_r(p, ":", &p)) && idx < 3) {
            params[idx++] = atoi(str);
        }
        rpm = params[0];
        distance = params[1];
        flag = params[2];
        handleStepperCommand(rpm, distance, flag);
        Serial.println("OK");
        break;
    }  
    case COLOR_READ: // Color sensor command
    if (shouldSendSensorData()) {
        if (argv1[0] == '0') {
            colorSensorLEDOff();
            readColorSensor();
        } else if (argv1[0] == '1') {
            colorSensorLEDOn();
            readColorSensor();
        } else {
            readColorSensor();
        }
    }
    // Don't send anything if sensors shouldn't transmit
    break;
    
    case ULTRASONIC_READ: // Ultrasonic sensor command, returns both left and right in cm
     if (shouldSendSensorData()) {
        long left = Ping(ULTRASONIC_LEFT_TRIG_PIN, ULTRASONIC_LEFT_ECHO_PIN);
        long right = Ping(ULTRASONIC_RIGHT_TRIG_PIN, ULTRASONIC_RIGHT_ECHO_PIN);
        Serial.print(left); Serial.print(" "); Serial.println(right);
     }
     // Don't send anything if sensors shouldn't transmit
    break;
    
  case UPDATE_PID:
    while ((str = strtok_r(p, ":", &p)) != nullptr) {
       pid_args[i] = atoi(str);
       i++;
    }
    Kp = pid_args[0];
    Kd = pid_args[1];
    Ki = pid_args[2];
    Ko = pid_args[3];
    Serial.println("OK");
    break;

  case GET_ROBOT_STATE:
    Serial.println(currentState);
    break;
    
  /* NEW COMMANDS FOR DEBUGGING */
  case 'D': // Debug state info
    Serial.print("State:");
    Serial.print(currentState);
    Serial.print(" Moving:");
    Serial.print(motorMoving);
    Serial.print(" Stepper:");
    Serial.println(isStepperActive());
    break;
    
  case 'F': // Force state reset (emergency)
    offloadingActive = false;
    motorMoving = false;
    currentState = STATE_STATIONARY;
    Serial.println("State reset");
    break;
#endif
  default:
    Serial.println("Invalid Command");
    break;
  }
  return 0;
}

void printAllEncoders() {
    Serial.print(getM1Encoder()); Serial.print(" ");
    Serial.print(getM2Encoder()); Serial.print(" ");
    Serial.print(getM3Encoder()); Serial.print(" ");
    Serial.println(getM4Encoder());
}

/* Setup function--runs once at startup. */
void setup() {

  Serial.begin(BAUDRATE);
  //Serial.println("ROSArduinoBridge Ready");
  pinMode(MOTOR_STBY, OUTPUT);
  digitalWrite(MOTOR_STBY, HIGH);
  //setMotorSpeedsTB6612(100, 100, 100, 100);
  //delay(2000);
  //setMotorSpeedsTB6612(0, 0, 0, 0);  
#ifdef USE_BASE
  #ifdef ARDUINO_ENC_COUNTER
    //set as inputs
    // DDRD &= ~(1<<LEFT_ENC_PIN_A);
    // DDRD &= ~(1<<LEFT_ENC_PIN_B);
    // DDRC &= ~(1<<RIGHT_ENC_PIN_A);
    // DDRC &= ~(1<<RIGHT_ENC_PIN_B);
    
    // //enable pull up resistors
    // PORTD |= (1<<LEFT_ENC_PIN_A);
    // PORTD |= (1<<LEFT_ENC_PIN_B);
    // PORTC |= (1<<RIGHT_ENC_PIN_A);
    // PORTC |= (1<<RIGHT_ENC_PIN_B);
    
    // // tell pin change mask to listen to left encoder pins
    // PCMSK2 |= (1 << LEFT_ENC_PIN_A)|(1 << LEFT_ENC_PIN_B);
    // // tell pin change mask to listen to right encoder pins
    // PCMSK1 |= (1 << RIGHT_ENC_PIN_A)|(1 << RIGHT_ENC_PIN_B);
    
    // // enable PCINT1 and PCINT2 interrupt in the general interrupt mask
    // PCICR |= (1 << PCIE1) | (1 << PCIE2);
  #endif
  
  initializeEncoders();
  initializeColorSensor();
  initMotorControllerTB6612();
  resetPID();
  
  // Initialize IMU with timeout protection (non-blocking)
  if (!initializeIMU()) {
    Serial.println(F("[WARN] IMU initialization failed or timeout - robot will operate without IMU"));
  }
  
  initializeStepper();
  
  // Initialize encoder values for movement detection
  lastM1 = getM1Encoder();
  lastM2 = getM2Encoder();
  lastM3 = getM3Encoder();
  lastM4 = getM4Encoder();
#endif

#ifdef USE_SERVOS
  initAllServos();
#endif
}

/* Enter the main loop.  Read and parse input from the serial port
   and run any valid commands. Run a PID calculation at the target
   interval and check for auto-stop conditions.
*/
void loop() {
  while (Serial.available() > 0) {
    // Read the next character
    chr = Serial.read();

    // Terminate a command with a CR
    if (chr == 13) {
      if (arg == 1) argv1[index] = 0;
      else if (arg == 2) argv2[index] = 0;
      runCommand();
      resetCommand();
    }
    // Use spaces to delimit parts of the command
    else if (chr == ' ') {
      // Step through the arguments
      if (arg == 0) arg = 1;
      else if (arg == 1)  {
        argv1[index] = 0;
        arg = 2;
        index = 0;
      }
      continue;
    }
    else {
      if (arg == 0) {
        // The first arg is the single-letter command
        cmd = chr;
      }
      else if (arg == 1) {
        // Subsequent arguments can be more than one character
        argv1[index] = chr;
        index++;
      }
      else if (arg == 2) {
        argv2[index] = chr;
        index++;
      }
    }
  }
  
  // Update the state machine first
  updateStateMachine();
 // printAllEncoders();
  runStepper();
  
#ifdef USE_BASE
  if (millis() > nextPID) {
    // Always update PID unless offloading (was only when moving)
    if (shouldSendOdometry()) {
      updatePID();
    } else if (currentState == STATE_STATIONARY) {
      // Reset PID integrators when stationary to prevent windup
      resetPID();
    }
    nextPID += PID_INTERVAL;
  }
  
  // AUTO-STOP SAFETY DISABLED (removed by user request)
  // Motors will continue running until explicitly stopped
  /*
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
    setMotorSpeedsTB6612(0, 0, 0, 0);
    resetPID();
    motorMoving = false;  // Update state tracking
  }
  */
#endif

#ifdef USE_SERVOS
  sweepAllServos();
#endif
}