/* Define single-letter commands for TB6612 motor driver (4-motor support).
   Unified command definitions for TB6612 logic.
*/

#ifndef COMMANDS_H
#define COMMANDS_H

#define ANALOG_READ        'a'
#define GET_BAUDRATE       'b'
#define PIN_MODE           'c'
#define DIGITAL_READ       'd'
#define READ_ENCODERS      'e'
#define SERVO_WRITE_SPEED  'f'  // w <index> <angle> <speed>
#define MOTOR_SPEEDS       'm'  // Set all 4 motor speeds
#define MOTOR_RAW_PWM      'o'
#define PING_BOT           'p'
#define STEPPER            'q'
#define ULTRASONIC_READ    'U'  // New command for ultrasonic sensor reading
#define COLOR_READ         'v'
#define RESET_ENCODERS     'r'
#define SERVO_WRITE        's'
#define SERVO_READ         't'
#define UPDATE_PID         'u'
#define DIGITAL_WRITE      'w'
#define ANALOG_WRITE       'x'
#define GET_IMU_ANGLE     'z'  // New command to get IMU Z angle

// Motor indices for 4-motor TB6612
#define MOTOR_FRONT_LEFT   0
#define MOTOR_FRONT_RIGHT  1
#define MOTOR_REAR_LEFT    2
#define MOTOR_REAR_RIGHT   3

#endif