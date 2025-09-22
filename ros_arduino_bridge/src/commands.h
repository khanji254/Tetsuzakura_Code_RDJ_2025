/* Define single-letter commands for TB6612 motor driver (4-motor support).
   Duplicated and extended from commands.h for TB6612-specific logic.
*/

#ifndef COMMANDS_H
#define COMMANDS__H

#define ANALOG_READ_TB6612    'a'
#define GET_BAUDRATE_TB6612   'b'
#define PIN_MODE_TB6612       'c'
#define DIGITAL_READ_TB6612   'd'
#define READ_ENCODERS_TB6612  'e'
#define MOTOR_SPEEDS_TB6612   'm'  // Set all 4 motor speeds
#define MOTOR_RAW_PWM_TB6612  'o'
#define PING_TB6612           'p'
#define RESET_ENCODERS_TB6612 'r'
#define SERVO_WRITE_TB6612    's'
#define SERVO_READ_TB6612     't'
#define UPDATE_PID_TB6612     'u'
#define DIGITAL_WRITE_TB6612  'w'
#define ANALOG_WRITE_TB6612   'x'

// Motor indices for 4-motor TB6612
#define MOTOR_FRONT_LEFT_TB6612   0
#define MOTOR_FRONT_RIGHT_TB6612  1
#define MOTOR_REAR_LEFT_TB6612    2
#define MOTOR_REAR_RIGHT_TB6612   3

#endif