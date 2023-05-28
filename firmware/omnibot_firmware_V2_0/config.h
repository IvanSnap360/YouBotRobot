#include "actuator.h"
#include <Arduino.h>

#define ROS_MODE 



#ifndef ROS_MODE

#define ENCODER_DEBUG
#define ENCODER_RAW_VALS
#define ENCODER_VELOCITY_VALS

#endif 


#define LED LED_BUILTIN

// ---- SERIAL SETTINGS --- //
#define SERIAL_BAUDRATE 57600
#define SERIAL_CONNECT_LED_BLINK_HZ    2
#define SERIAL_DISCONNECT_LED_BLINK_HZ 20

// ----- ROS SETTINGS ----- //
#ifdef ROS_MODE

#define ROS_BAUDRATE 57600
#define ROS_CONNECT_LED_BLINK_HZ    1
#define ROS_DISCONNECT_LED_BLINK_HZ 10

#define WHEELS_JOINTS_STATE_TOPIC_NAME   "/joint_state"
#define WHEELS_JOINTS_CONTROL_TOPIC_NAME "/joint_control"

#define JOINT_STATE_PUB_RATE_HZ 10

// --- ROSPARAMS NAMES  --- //



#endif // ROS_MODE
//  WHEEL NAMES ENUMERATOR  //
enum
{
    LEFT_FRONT,
    RIGHT_FRONT,
    LEFT_BACK,
    RIGHT_BACK,

    WHEELS_COUNT
} wheels_enum;


// -- COMMON MOTOR PARAMS - //
#define MAX_PWM             255
#define MIN_PWM             0
#define START_DEADZONE_PWM  50
#define REDUCTION           (double)(1 / 100)
#define MAX_VELOCITY        110.0 // rev/min
#define MIN_VELOCITY        0.0   // rev/min
#define MIN_PWM_VELOCITY    1.0   // rev/min
#define PID_HZ              20.0  // HZ

//  LEFT FRONT MOTOR CONFIG  //
#define LEFT_FRONT_MOTOR_PIN_A      0
#define LEFT_FRONT_MOTOR_PIN_B      0
#define LEFT_FRONT_ENCODER_PIN_A    0
#define LEFT_FRONT_ENCODER_PIN_B    0
#define LEFT_FRONT_REVERSE          false


//  RIGHT FRONT MOTOR CONFIG  //
#define RIGHT_FRONT_MOTOR_PIN_A     0
#define RIGHT_FRONT_MOTOR_PIN_B     0
#define RIGHT_FRONT_ENCODER_PIN_A   0
#define RIGHT_FRONT_ENCODER_PIN_B   0
#define RIGHT_FRONT_REVERSE         false


//  LEFT BACK MOTOR CONFIG  //
#define LEFT_BACK_MOTOR_PIN_A       0
#define LEFT_BACK_MOTOR_PIN_B       0
#define LEFT_BACK_ENCODER_PIN_A     0
#define LEFT_BACK_ENCODER_PIN_B     0
#define LEFT_BACK_REVERSE           false

//  RIGHT BACK MOTOR CONFIG  //
#define RIGHT_BACK_MOTOR_PIN_A      0
#define RIGHT_BACK_MOTOR_PIN_B      0
#define RIGHT_BACK_ENCODER_PIN_A    0
#define RIGHT_BACK_ENCODER_PIN_B    0
#define RIGHT_BACK_REVERSE          false


/////////////////////////////////////////////////////////////////////////////////////////////////////
actuator_cfg_t actuators_config[WHEELS_COUNT] =
    {
        // LEFT_FRONT
        {
            .motor_pinA = LEFT_FRONT_MOTOR_PIN_A,
            .motor_pinB = LEFT_FRONT_MOTOR_PIN_B,
            .encoder_pinA = LEFT_FRONT_ENCODER_PIN_A,
            .encoder_pinB = LEFT_FRONT_ENCODER_PIN_B,
            .reverse = LEFT_FRONT_REVERSE,
            .reduction = REDUCTION,
            .start_deadzone_pwm_val = START_DEADZONE_PWM,
            .max_pwm = MAX_PWM,
            .max_velocity = MAX_VELOCITY,
            .min_velocity = MIN_VELOCITY,
            .min_pwm_velocity = MIN_PWM_VELOCITY,
            .pid_hz = PID_HZ
        },
        // RIGHT_FRONT
        {
            .motor_pinA = RIGHT_FRONT_MOTOR_PIN_A,
            .motor_pinB = RIGHT_FRONT_MOTOR_PIN_B,
            .encoder_pinA = RIGHT_FRONT_ENCODER_PIN_A,
            .encoder_pinB = RIGHT_FRONT_ENCODER_PIN_B,
            .reverse = RIGHT_FRONT_REVERSE,
            .reduction = REDUCTION,
            .start_deadzone_pwm_val = START_DEADZONE_PWM,
            .max_pwm = MAX_PWM,
            .max_velocity = MAX_VELOCITY,
            .min_velocity = MIN_VELOCITY,
            .min_pwm_velocity = MIN_PWM_VELOCITY,
            .pid_hz = PID_HZ
        },
        // LEFT_BACK
        {
            .motor_pinA = LEFT_BACK_MOTOR_PIN_A,
            .motor_pinB = LEFT_BACK_MOTOR_PIN_B,
            .encoder_pinA = LEFT_BACK_ENCODER_PIN_A,
            .encoder_pinB = LEFT_BACK_ENCODER_PIN_B,
            .reverse = LEFT_BACK_REVERSE,
            .reduction = REDUCTION,
            .start_deadzone_pwm_val = START_DEADZONE_PWM,
            .max_pwm = MAX_PWM,
            .max_velocity = MAX_VELOCITY,
            .min_velocity = MIN_VELOCITY,
            .min_pwm_velocity = MIN_PWM_VELOCITY,
            .pid_hz = PID_HZ
        },

        // RIGHT_BACK
        {
            .motor_pinA =  RIGHT_BACK_MOTOR_PIN_A,
            .motor_pinB =  RIGHT_BACK_MOTOR_PIN_B,
            .encoder_pinA =  RIGHT_BACK_ENCODER_PIN_A,
            .encoder_pinB =  RIGHT_BACK_ENCODER_PIN_B,
            .reverse =  RIGHT_BACK_REVERSE,
            .reduction = REDUCTION,
            .start_deadzone_pwm_val = START_DEADZONE_PWM,
            .max_pwm = MAX_PWM,
            .max_velocity = MAX_VELOCITY,
            .min_velocity = MIN_VELOCITY,
            .min_pwm_velocity = MIN_PWM_VELOCITY,
            .pid_hz = PID_HZ
        }};
