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
       notice, this list of conditions and the following disclaimer.
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

  /* Run the PID loop at 30 times per second */
  #define PID_RATE           30     // Hz

  /* Convert the rate into an interval */
  const int PID_INTERVAL = 1000 / PID_RATE;
  
  /* Track the next time we make a PID calculation */
  unsigned long nextPID = PID_INTERVAL;

  /* Stop the robot if it hasn't received a movement command
   in this number of milliseconds */
  #define AUTO_STOP_INTERVAL 2000
  long lastMotorCommand = AUTO_STOP_INTERVAL;
#endif

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
#endif

#ifdef USE_BASE
  case READ_ENCODERS:
    printAllEncoders();
    break;
  case RESET_ENCODERS:
    resetEncoders();
    resetPID();
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
      setMotorSpeedsTB6612(speeds[0], speeds[1], speeds[2], speeds[3]);
      Serial.println("OK");
      if (cmd == MOTOR_RAW_PWM) {
        resetPID();
        moving = 0;
      }
    }
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
#endif
  default:
    Serial.println("Invalid Command");
    break;
  }
  return 0;
}

/* Setup function--runs once at startup. */
void setup() {
  Serial.begin(BAUDRATE);
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
  initMotorControllerTB6612();
  resetPID();
  initializeStepper();
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
  runStepper();

  
#ifdef USE_BASE
  if (millis() > nextPID) {
    updatePID();
    nextPID += PID_INTERVAL;
  }
  
  // Check to see if we have exceeded the auto-stop interval
  if ((millis() - lastMotorCommand) > AUTO_STOP_INTERVAL) {
    setMotorSpeedsTB6612(0, 0, 0, 0);
    moving = 0;
  }
#endif

#ifdef USE_SERVOS
  sweepAllServos();
#endif
}


void printAllEncoders() {
    Serial.print(getM1Encoder()); Serial.print(" ");
    Serial.print(getM2Encoder()); Serial.print(" ");
    Serial.print(getM3Encoder()); Serial.print(" ");
    Serial.println(getM4Encoder());
}

