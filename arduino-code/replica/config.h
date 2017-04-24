/* Debugging -----------------------------------------------------------------*/
#define SERIAL_DEBUGGING

// I2C Settings
#define SLAVE_ADDRESS 0x04

/* Sensors -------------------------------------------------------------------*/
// ultrasonic
#define SONAR_NUM       3       // number of sensors
#define MAX_DISTANCE    200     // maximum distance (in cm) to ping
#define PING_INTERVAL   35      // milliseconds between sensor pings

#define SONAR_LEFT_TRIG         10
#define SONAR_LEFT_ECHO         11

#define SONAR_CENTRE_TRIG       A0
#define SONAR_CENTRE_ECHO       A1

#define SONAR_RIGHT_TRIG        A2
#define SONAR_RIGHT_ECHO        A3

#define ENCODER_NUM 2
/* Motors --------------------------------------------------------------------*/
#define LEFT_MOTOR_A    5
#define LEFT_MOTOR_B    4
#define RIGHT_MOTOR_A   6
#define RIGHT_MOTOR_B   7

/* General -------------------------------------------------------------------*/
#define MAX_SPEED   255

