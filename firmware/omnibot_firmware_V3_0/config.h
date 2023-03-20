/**
 * @file config.h
 * @author Ivan
 * @brief
 * @version 0.1
 * @date 2023-02-21
 *
 * @copyright Copyright (c) 2023
 *
 */
#include "actuator.h"
/**
 * WORK MODE SETUP
 *
 */
// #define WORK_MODE__ROS
// #define WORK_MODE__SERIAL

#ifdef WORK_MODE__ROS
#pragma message("ROS WORKMODE")
#endif // WORK_MODE__ROS

#ifdef WORK_MODE__SERIAL
#pragma message("SERIAL WORKMODE")
#endif // WORK_MODE__SERIAL
/**
 *
 * ROS SETUP
 *
 */

#define ROS_SERIAL__BAUDRATE /*                   */ 115200
#define ROS_SERIAL__HARDWARE /*                   */ ArduinoHardware
#define ROS_SERIAL__MAX_PUBLISHERS /*             */ 25
#define ROS_SERIAL__MAX_SUBSCRIBERS /*            */ 25
#define ROS_SERIAL__MAX_INPUT_BUFFER_SIZE /*      */ 2048
#define ROS_SERIAL__MAX_OUTPUT_BUFFER_SIZE /*     */ 2048

#define ROS_TOPIC_LEFT_FRONT_WHEEL_CONTROLLER /*  */ "/omnibot_robot/left_front_mecanum_controller/command"
#define ROS_TOPIC_RIGHT_FRONT_WHEEL_CONTROLLER /* */ "/omnibot_robot/right_front_mecanum_controller/command"
#define ROS_TOPIC_LEFT_BACK_WHEEL_CONTROLLER /*   */ "/omnibot_robot/left_back_mecanum_controller/command"
#define ROS_TOPIC_RIGHT_BACK_WHEEL_CONTROLLER /*  */ "/omnibot_robot/right_back_mecanum_controller/command"

/**
 * ROS PARAMS
 *
 */

#define ROS_NODE__WORK_RATE_HZ 50.0

/**
 * SERIAL SETUP
 *
 */

#define COM_SERIAL__BAUDRATE 115200

/**
 * COMMON INTERFACE PARAMS
 *
 */
uint32_t blink_last_time = 0;
#define CONNECTION_LED_BLINK_RATE_HZ 10.0
#define DISCONNECTION_LED_BLINK_RATE_HZ 2.0
/**
 * SERIAL PARAMS
 *
 */

/**
 * MOTOR PARAMS
 *
 */

// -- COMMON MOTOR PARAMS - //
#define ACTUATORS_COUNT /*                     */ 4
enum
{
    LEFT_FRONT,
    RIGHT_FRONT,
    LEFT_BACK,
    RIGHT_BACK
} actuator_enumrator;

#define MAX_PWM /*                             */ 255
#define MIN_PWM /*                             */ 0
#define START_DEADZONE_PWM /*                  */ 50
#define REDUCTION /*                           */ (double)(1 / 100)
#define MAX_VELOCITY /*                        */ 110.0 // rev/min
#define MIN_VELOCITY /*                        */ 0.0   // rev/min
#define MIN_PWM_VELOCITY /*                    */ 0.0   // rev/min
#define PID_HZ /*                              */ 5.0   // HZ
#define PID_KP /*                              */ 1.0
#define PID_KI /*                              */ 0.0
#define PID_KD /*                              */ 0.0
#define ENCODER_WORK_MODE /*                   */ RISING
#define ENCODER_TIKS_PER_REVOLUTION /*          */ 100

//  LEFT FRONT MOTOR CONFIG  //
#define LEFT_FRONT__MOTOR_PIN1 /*              */ 6
#define LEFT_FRONT__MOTOR_PIN2 /*              */ 7
#define LEFT_FRONT__ENCODER_PIN_A /*           */ 0
#define LEFT_FRONT__ENCODER_PIN_B /*           */ 1
#define LEFT_FRONT__ENCODER_WORK_MODE /*       */ ENCODER_WORK_MODE
#define LEFT_FRONT__INPIN_WORK_MODE /*         */ INPUT_PULLDOWN
#define LEFT_FRONT__MOTOR_REVERSE /*           */ true
#define LEFT_FRONT__ENCODER_REVERSE /*         */ false
#define LEFT_FRONT__KP /*                      */ PID_KP
#define LEFT_FRONT__KI /*                      */ PID_KI
#define LEFT_FRONT__KD /*                      */ PID_KD
#define LEFT_FRONT__PID_RATE_HZ /*             */ PID_HZ
#define LEFT_FRONT__MOTOR_REDUCTION /*         */ REDUCTION
#define LEFT_FRONT__MAX_PWM /*                 */ MAX_PWM
#define LEFT_FRONT__MIN_PWM /*                 */ MIN_PWM
#define LEFT_FRONT__MIN_WORK_PWM /*            */ START_DEADZONE_PWM
#define LEFT_FRONT__MAX_VELOCITY /*            */ MAX_VELOCITY
#define LEFT_FRONT__MIN_VELOCITY /*            */ MIN_VELOCITY
#define LEFT_FRONT__MIN_WORK_VELOCITY /*       */ MIN_PWM_VELOCITY

//  RIGHT FRONT MOTOR CONFIG  //

#define RIGHT_FRONT__MOTOR_PIN1 /*              */ 4
#define RIGHT_FRONT__MOTOR_PIN2 /*              */ 5
#define RIGHT_FRONT__ENCODER_PIN_A /*           */ 16
#define RIGHT_FRONT__ENCODER_PIN_B /*           */ 17
#define RIGHT_FRONT__ENCODER_WORK_MODE /*       */ ENCODER_WORK_MODE
#define RIGHT_FRONT__INPIN_WORK_MODE /*         */ INPUT_PULLDOWN
#define RIGHT_FRONT__MOTOR_REVERSE /*           */ false
#define RIGHT_FRONT__ENCODER_REVERSE /*         */ false
#define RIGHT_FRONT__KP /*                      */ PID_KP
#define RIGHT_FRONT__KI /*                      */ PID_KI
#define RIGHT_FRONT__KD /*                      */ PID_KD
#define RIGHT_FRONT__PID_RATE_HZ /*             */ PID_HZ
#define RIGHT_FRONT__MOTOR_REDUCTION /*         */ REDUCTION
#define RIGHT_FRONT__MAX_PWM /*                 */ MAX_PWM
#define RIGHT_FRONT__MIN_PWM /*                 */ MIN_PWM
#define RIGHT_FRONT__MIN_WORK_PWM /*            */ START_DEADZONE_PWM
#define RIGHT_FRONT__MAX_VELOCITY /*            */ MAX_VELOCITY
#define RIGHT_FRONT__MIN_VELOCITY /*            */ MIN_VELOCITY
#define RIGHT_FRONT__MIN_WORK_VELOCITY /*       */ MIN_PWM_VELOCITY

//  LEFT BACK MOTOR CONFIG  //
#define LEFT_BACK__MOTOR_PIN1 /*                */ 8
#define LEFT_BACK__MOTOR_PIN2 /*                */ 9
#define LEFT_BACK__ENCODER_PIN_A /*             */ 11
#define LEFT_BACK__ENCODER_PIN_B /*             */ 12
#define LEFT_BACK__ENCODER_WORK_MODE /*         */ ENCODER_WORK_MODE
#define LEFT_BACK__INPIN_WORK_MODE /*           */ INPUT_PULLDOWN
#define LEFT_BACK__MOTOR_REVERSE /*             */ true
#define LEFT_BACK__ENCODER_REVERSE /*           */ false
#define LEFT_BACK__KP /*                        */ PID_KP
#define LEFT_BACK__KI /*                        */ PID_KI
#define LEFT_BACK__KD /*                        */ PID_KD
#define LEFT_BACK__PID_RATE_HZ /*               */ PID_HZ
#define LEFT_BACK__MOTOR_REDUCTION /*           */ REDUCTION
#define LEFT_BACK__MAX_PWM /*                   */ MAX_PWM
#define LEFT_BACK__MIN_PWM /*                   */ MIN_PWM
#define LEFT_BACK__MIN_WORK_PWM /*              */ START_DEADZONE_PWM
#define LEFT_BACK__MAX_VELOCITY /*              */ MAX_VELOCITY
#define LEFT_BACK__MIN_VELOCITY /*              */ MIN_VELOCITY
#define LEFT_BACK__MIN_WORK_VELOCITY /*         */ MIN_PWM_VELOCITY

//  RIGHT BACK MOTOR CONFIG  //
#define RIGHT_BACK__MOTOR_PIN1 /*               */ 2
#define RIGHT_BACK__MOTOR_PIN2 /*               */ 3
#define RIGHT_BACK__ENCODER_PIN_A /*            */ 18
#define RIGHT_BACK__ENCODER_PIN_B /*            */ 19
#define RIGHT_BACK__ENCODER_WORK_MODE /*        */ ENCODER_WORK_MODE
#define RIGHT_BACK__INPIN_WORK_MODE /*          */ INPUT_PULLDOWN
#define RIGHT_BACK__MOTOR_REVERSE /*            */ true
#define RIGHT_BACK__ENCODER_REVERSE /*          */ false
#define RIGHT_BACK__KP /*                       */ PID_KP
#define RIGHT_BACK__KI /*                       */ PID_KI
#define RIGHT_BACK__KD /*                       */ PID_KD
#define RIGHT_BACK__PID_RATE_HZ /*              */ PID_HZ
#define RIGHT_BACK__MOTOR_REDUCTION /*          */ REDUCTION
#define RIGHT_BACK__MAX_PWM /*                  */ MAX_PWM
#define RIGHT_BACK__MIN_PWM /*                  */ MIN_PWM
#define RIGHT_BACK__MIN_WORK_PWM /*             */ START_DEADZONE_PWM
#define RIGHT_BACK__MAX_VELOCITY /*             */ MAX_VELOCITY
#define RIGHT_BACK__MIN_VELOCITY /*             */ MIN_VELOCITY
#define RIGHT_BACK__MIN_WORK_VELOCITY /*        */ MIN_PWM_VELOCITY

actuator_config_t _actuator_config[ACTUATORS_COUNT] = {

    //  LEFT FRONT MOTOR CONFIG  //
    {
        .motor_pin1 = /*                    */ LEFT_FRONT__MOTOR_PIN1,
        .motor_pin2 = /*                    */ LEFT_FRONT__MOTOR_PIN2,
        .encoder_pin_A = /*                 */ LEFT_FRONT__ENCODER_PIN_A,
        .encoder_pin_B = /*                 */ LEFT_FRONT__ENCODER_PIN_B,
        .inpin_work_mode = /*               */ LEFT_FRONT__INPIN_WORK_MODE,
        .motor_reverse = /*                 */ LEFT_FRONT__MOTOR_REVERSE,
        .encoder_reverse = /*               */ LEFT_FRONT__ENCODER_REVERSE,
        .kp = /*                            */ LEFT_FRONT__KP,
        .ki = /*                            */ LEFT_FRONT__KI,
        .kd = /*                            */ LEFT_FRONT__KD,
        .pid_rate_hz = /*                   */ LEFT_FRONT__PID_RATE_HZ,
        .motor_reduction = /*               */ LEFT_FRONT__MOTOR_REDUCTION,
        .max_pwm = /*                       */ LEFT_FRONT__MAX_PWM,
        .min_pwm = /*                       */ LEFT_FRONT__MIN_PWM,
        .min_work_pwm = /*                  */ LEFT_FRONT__MIN_WORK_PWM,
        .max_velocity = /*                  */ LEFT_FRONT__MAX_VELOCITY,
        .min_velocity = /*                  */ LEFT_FRONT__MIN_VELOCITY,
        .min_work_velocity = /*             */ LEFT_FRONT__MIN_WORK_VELOCITY,
    },

    //  RIGHT FRONT MOTOR CONFIG  //
    {
        .motor_pin1 = /*                    */ RIGHT_FRONT__MOTOR_PIN1,
        .motor_pin2 = /*                    */ RIGHT_FRONT__MOTOR_PIN2,
        .encoder_pin_A = /*                 */ RIGHT_FRONT__ENCODER_PIN_A,
        .encoder_pin_B = /*                 */ RIGHT_FRONT__ENCODER_PIN_B,
        .inpin_work_mode = /*               */ RIGHT_FRONT__INPIN_WORK_MODE,
        .motor_reverse = /*                 */ RIGHT_FRONT__MOTOR_REVERSE,
        .encoder_reverse = /*               */ RIGHT_FRONT__ENCODER_REVERSE,
        .kp = /*                            */ RIGHT_FRONT__KP,
        .ki = /*                            */ RIGHT_FRONT__KI,
        .kd = /*                            */ RIGHT_FRONT__KD,
        .pid_rate_hz = /*                   */ RIGHT_FRONT__PID_RATE_HZ,
        .motor_reduction = /*               */ RIGHT_FRONT__MOTOR_REDUCTION,
        .max_pwm = /*                       */ RIGHT_FRONT__MAX_PWM,
        .min_pwm = /*                       */ RIGHT_FRONT__MIN_PWM,
        .min_work_pwm = /*                  */ RIGHT_FRONT__MIN_WORK_PWM,
        .max_velocity = /*                  */ RIGHT_FRONT__MAX_VELOCITY,
        .min_velocity = /*                  */ RIGHT_FRONT__MIN_VELOCITY,
        .min_work_velocity = /*             */ RIGHT_FRONT__MIN_WORK_VELOCITY,
    },
    //  LEFT BACK MOTOR CONFIG  //
    {
        .motor_pin1 = /*                    */ LEFT_BACK__MOTOR_PIN1,
        .motor_pin2 = /*                    */ LEFT_BACK__MOTOR_PIN2,
        .encoder_pin_A = /*                 */ LEFT_BACK__ENCODER_PIN_A,
        .encoder_pin_B = /*                 */ LEFT_BACK__ENCODER_PIN_B,
        .inpin_work_mode = /*               */ LEFT_BACK__INPIN_WORK_MODE,
        .motor_reverse = /*                 */ LEFT_BACK__MOTOR_REVERSE,
        .encoder_reverse = /*               */ LEFT_BACK__ENCODER_REVERSE,
        .kp = /*                            */ LEFT_BACK__KP,
        .ki = /*                            */ LEFT_BACK__KI,
        .kd = /*                            */ LEFT_BACK__KD,
        .pid_rate_hz = /*                   */ LEFT_BACK__PID_RATE_HZ,
        .motor_reduction = /*               */ LEFT_BACK__MOTOR_REDUCTION,
        .max_pwm = /*                       */ LEFT_BACK__MAX_PWM,
        .min_pwm = /*                       */ LEFT_BACK__MIN_PWM,
        .min_work_pwm = /*                  */ LEFT_BACK__MIN_WORK_PWM,
        .max_velocity = /*                  */ LEFT_BACK__MAX_VELOCITY,
        .min_velocity = /*                  */ LEFT_BACK__MIN_VELOCITY,
        .min_work_velocity = /*             */ LEFT_BACK__MIN_WORK_VELOCITY,
    },
    //  RIGHT BACK MOTOR CONFIG  //
    {
        .motor_pin1 = /*                    */ RIGHT_BACK__MOTOR_PIN1,
        .motor_pin2 = /*                    */ RIGHT_BACK__MOTOR_PIN2,
        .encoder_pin_A = /*                 */ RIGHT_BACK__ENCODER_PIN_A,
        .encoder_pin_B = /*                 */ RIGHT_BACK__ENCODER_PIN_B,
        .inpin_work_mode = /*               */ RIGHT_BACK__INPIN_WORK_MODE,
        .motor_reverse = /*                 */ RIGHT_BACK__MOTOR_REVERSE,
        .encoder_reverse = /*               */ RIGHT_BACK__ENCODER_REVERSE,
        .kp = /*                            */ RIGHT_BACK__KP,
        .ki = /*                            */ RIGHT_BACK__KI,
        .kd = /*                            */ RIGHT_BACK__KD,
        .pid_rate_hz = /*                   */ RIGHT_BACK__PID_RATE_HZ,
        .motor_reduction = /*               */ RIGHT_BACK__MOTOR_REDUCTION,
        .max_pwm = /*                       */ RIGHT_BACK__MAX_PWM,
        .min_pwm = /*                       */ RIGHT_BACK__MIN_PWM,
        .min_work_pwm = /*                  */ RIGHT_BACK__MIN_WORK_PWM,
        .max_velocity = /*                  */ RIGHT_BACK__MAX_VELOCITY,
        .min_velocity = /*                  */ RIGHT_BACK__MIN_VELOCITY,
        .min_work_velocity = /*             */ RIGHT_BACK__MIN_WORK_VELOCITY,
    },
};

ACTUATOR actuators[ACTUATORS_COUNT];

void LEFT_FRONT_ENC_A_ISR()
{
    actuators[LEFT_FRONT].encA_ISR();
}
void RIGHT_FRONT_ENC_A_ISR()
{
    actuators[RIGHT_FRONT].encA_ISR();
}
void LEFT_BACK_ENC_A_ISR()
{
    actuators[LEFT_BACK].encA_ISR();
}
void RIGHT_BACK_ENC_A_ISR()
{
    actuators[RIGHT_BACK].encA_ISR();
}
void (*functptr_enc_A[])() = {
        LEFT_FRONT_ENC_A_ISR,
        RIGHT_FRONT_ENC_A_ISR,
        LEFT_BACK_ENC_A_ISR,
        RIGHT_BACK_ENC_A_ISR,
};

void LEFT_FRONT_ENC_B_ISR()
{
    actuators[LEFT_FRONT].encB_ISR();
}
void RIGHT_FRONT_ENC_B_ISR()
{
    actuators[RIGHT_FRONT].encB_ISR();
}
void LEFT_BACK_ENC_B_ISR()
{
    actuators[LEFT_BACK].encB_ISR();
}
void RIGHT_BACK_ENC_B_ISR()
{
    actuators[RIGHT_BACK].encB_ISR();
}
void (*functptr_enc_B[])() = {
        LEFT_FRONT_ENC_B_ISR,
        RIGHT_FRONT_ENC_B_ISR,
        LEFT_BACK_ENC_B_ISR,
        RIGHT_BACK_ENC_B_ISR,
};
