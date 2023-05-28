#include "actuator.h"
// ##################################################################### //
// ############################### MACROS ############################## //
// ##################################################################### //

#define RPM_2_RADS(rpm) (double)(rpm * (PI / 30))
#define RADS_2_RPM(rads) (double)((rads * 30) / PI)
// ##################################################################### //
// ############################ WORK PARAMS ############################ //
// ##################################################################### //
// #define WORK_MODE__SERIAL
#define ROS_WORK_MODE
// ##################################################################### //
// ######################## COMMUNICATION PARAMS ####################### //
// ##################################################################### //
uint32_t blink_last_time = 0;
#define CONNECTION_LED_BLINK_RATE_HZ 10.0
#define DISCONNECTION_LED_BLINK_RATE_HZ 2.0

// ##################################################################### //
// ########################### SERIAL PARAMS ########################### //
// ##################################################################### //
#define COM_SERIAL__BAUDRATE 115200

#ifdef WORK_MODE__SERIAL
/**/ #define ENABLE_OUTPUT
#endif // WORK_MODE__SERIAL

#ifdef ENABLE_OUTPUT
/**/ #define OUTPUT_PID_ANALAYZER
// /**/ #define OUTPUT_ECODER_TIKS
// /**/ #define OUTPUT_ACTUATORS_ANGULAR_VELOCITY
// /**/ #define OUTPUT_ACTUATORS_RPM
#endif // ENABLE_OUTPUT

#ifdef OUTPUT_PID_ANALAYZER
// /**/ #define ACTUATOR_NUMBER_FOR_PID_ANALAYZER LEFT_FRONT
/**/ #define ACTUATOR_NUMBER_FOR_PID_ANALAYZER RIGHT_FRONT
// /**/ #define ACTUATOR_NUMBER_FOR_PID_ANALAYZER LEFT_BACK
// /**/ #define ACTUATOR_NUMBER_FOR_PID_ANALAYZER RIGHT_BACK
#endif // OUTPUT_PID_ANALAYZER



#define OUTPUT_SEND_FREQ 20.0
#define OUTPUT_SEND_PERIOD (uint32_t)(1000.0 / OUTPUT_SEND_FREQ)
uint32_t last_send_time = 0;
// ##################################################################### //
// ############################# ROS PARAMS ############################ //
// ##################################################################### //
#define ROS_SERIAL__BAUDRATE /*                   */ 115200
#define ROS_SERIAL__HARDWARE /*                   */ ArduinoHardware
#define ROS_SERIAL__MAX_PUBLISHERS /*             */ 25
#define ROS_SERIAL__MAX_SUBSCRIBERS /*            */ 25
#define ROS_SERIAL__MAX_INPUT_BUFFER_SIZE /*      */ 2048
#define ROS_SERIAL__MAX_OUTPUT_BUFFER_SIZE /*     */ 2048

#define ROS_TOPIC_LEFT_FRONT_WHEEL_CONTROLLER /*  */     "/omnibot_robot_platform_controller/left_front_mecanum_controller/command"
#define ROS_TOPIC_RIGHT_FRONT_WHEEL_CONTROLLER /* */     "/omnibot_robot_platform_controller/right_front_mecanum_controller/command"
#define ROS_TOPIC_LEFT_BACK_WHEEL_CONTROLLER /*   */     "/omnibot_robot_platform_controller/left_back_mecanum_controller/command"
#define ROS_TOPIC_RIGHT_BACK_WHEEL_CONTROLLER /*  */     "/omnibot_robot_platform_controller/right_back_mecanum_controller/command"
#define ROS_NODE__WORK_RATE_HZ /*                 */ 25.0

// ##################################################################### //
// ############################ PROTECTIONS ############################ //
// ##################################################################### //
#ifdef WORK_MODE__SERIAL
/**/ #undef ROS_WORK_MODE
#warning WORK IN SERIAL_MODE, DISABLING ALL ROS COMMUNICATION!!!
#endif // WORK_MODE__SERIAL

#ifdef ROS_WORK_MODE
#warning WORK IN ROS_MODE, DISABLING ALL SERIAL OUTPUTS!!!
/**/ #undef ENABLE_OUTPUT
/**/ #undef WORK_MODE__SERIAL
#endif // ROS_WORK_MODE

#ifdef OUTPUT_PID_ANALAYZER
/**/ #undef OUTPUT_ECODER_TIKS
/**/ #undef OUTPUT_ACTUATORS_ANGULAR_VELOCITY
/**/ #undef OUTPUT_ACTUATORS_RPM
#warning WORK IN SERIAL_PID_ANALAYZER_MODE, DISABLING ALL ROS COMMUNICATION AND ANOTHER SERIAL OUTPUTS!!!
#endif // OUTPUT_PID_ANALAYZER

#if defined(OUTPUT_PID_ANALAYZER) && !defined(ACTUATOR_NUMBER_FOR_PID_ANALAYZER)
/**/ #error Not choose ACTUATOR_NUMBER_FOR_PID_ANALAYZER!!!
#endif // !ACTUATOR_NUMBER_FOR_PID_ANALAYZER
// ##################################################################### //
// ###################### COMMON ACTUATORS PARAMS ###################### //
// ##################################################################### //

#define ACTUATORS_COUNT 4 // actuators count

enum actuator_enum // actuator enumerator
{
    LEFT_FRONT,
    RIGHT_FRONT,
    LEFT_BACK,
    RIGHT_BACK
};

// ~~~~~~~~~~~~ CONTROL PARAMS ~~~~~~~~~~~ //
#define CONTROLLER_FREQ /*              */ 1000.0
#define CONTROLLER_PERIOD /*            */ (uint32_t)(1000.0 / CONTROLLER_FREQ)

// ~~~~~~~~~~ COMMON PWM PARAMS ~~~~~~~~~~ //
#define MAX_PWM /*                      */ 255
#define MIN_WORK_PWM /*                 */ 50
#define MIN_PWM /*                      */ 0

// ~~~~~~~~~ COMMON MOTOR PARAMS ~~~~~~~~~ //
#define MOTOR_REDUCTION_VALUE /*        */ 90

// ~~~~~~~~~~ COMMON RPM PARAMS ~~~~~~~~~~ //
#define MAX_RPM /*                      */ 110.0
#define MIN_WORK_RPM /*                 */ 10.0
#define MIN_RPM /*                      */ 0.0

// ~~~~ COMMON ANGULAR VELOCITY PARAMS ~~~ //
#define MAX_ANGULAR_VELOCITY /*         */ RPM_2_RADS(MAX_RPM)
#define MIN_WORK_ANGULAR_VELOCITY /*    */ RPM_2_RADS(MIN_WORK_RPM)
#define MIN_ANGULAR_VELOCITY /*         */ RPM_2_RADS(MIN_RPM)

// ~~~~~~~~ COMMON ENCODERS PARAMS ~~~~~~~ //
#define ENCODER_WORK_MODE /*            */ RISING // (RISING, FALLING, CHANGE)
#define ENCODER_PINS_MODE /*            */ INPUT
#define ENCODER_TIKS_PER_REVOLUTION /*  */ 11

// ~~~~~~~~~ COMMON WHEELS PARAMS ~~~~~~~~ //
#define WHEEL_DEAMETER /*               */ 0.135
#define WHEEL_RADIUS /*                 */ (double)(WHEEL_DEAMETER / 2)

// ~~~~~~~~~~ COMMON PID PARAMAS ~~~~~~~~~ //
#define PID_Kp /*                       */ 30.0 // 30 20
#define PID_Ki /*                       */ 15.0  // 15 8
#define PID_Kd /*                       */ 0.6


// ##################################################################### //
// ########################## ACTUATORS PARAMS ######################### //
// ##################################################################### //

// ====================================================== //
// ============= LEFT FRONT ACTUATOR PARAMS ============= //
// ====================================================== //
#define LEFT_FRONT__MOTOR_PIN_A /*                   */ 6
#define LEFT_FRONT__MOTOR_PIN_B /*                   */ 7
#define LEFT_FRONT__MOTOR_REVERSE /*                 */ FALSE
#define LEFT_FRONT__MOTOR_REDUCTION_VALUE /*         */ MOTOR_REDUCTION_VALUE
#define LEFT_FRONT__ENCODER_PIN_A /*                 */ 1
#define LEFT_FRONT__ENCODER_PIN_B /*                 */ 0
#define LEFT_FRONT__ENCODER_WORK_MODE /*             */ ENCODER_WORK_MODE
#define LEFT_FRONT__ENCODER_PINS_MODE /*             */ ENCODER_PINS_MODE
#define LEFT_FRONT__ENCODER_REVERSE /*               */ FALSE
#define LEFT_FRONT__ENCODER_TIKS_PER_REVOLUTION /*   */ ENCODER_TIKS_PER_REVOLUTION
#define LEFT_FRONT__KP /*                            */ PID_Kp
#define LEFT_FRONT__KI /*                            */ PID_Ki
#define LEFT_FRONT__KD /*                            */ PID_Kd
#define LEFT_FRONT__CONTROLLER_PERIOD /*                    */ CONTROLLER_PERIOD
#define LEFT_FRONT__MAX_PWM /*                       */ MAX_PWM
#define LEFT_FRONT__MIN_PWM /*                       */ MIN_PWM
#define LEFT_FRONT__MIN_WORK_PWM /*                  */ MIN_WORK_PWM
#define LEFT_FRONT__MAX_ANGULAR_VELOCITY /*          */ MAX_ANGULAR_VELOCITY
#define LEFT_FRONT__MIN_ANGULAR_VELOCITY /*          */ MIN_ANGULAR_VELOCITY
#define LEFT_FRONT__MIN_WORK_ANGULAR_VELOCITY /*     */ MIN_WORK_ANGULAR_VELOCITY
#define LEFT_FRONT__MAX_RPM /*                       */ MAX_RPM
#define LEFT_FRONT__MIN_RPM /*                       */ MIN_RPM
#define LEFT_FRONT__MIN_WORK_RPM /*                  */ MIN_WORK_RPM

// ====================================================== //
// ============= RIGHT FRONT ACTUATOR PARAMS ============ //
// ====================================================== //

#define RIGHT_FRONT__MOTOR_PIN_A /*                  */ 4
#define RIGHT_FRONT__MOTOR_PIN_B /*                  */ 5
#define RIGHT_FRONT__MOTOR_REVERSE /*                */ TRUE
#define RIGHT_FRONT__MOTOR_REDUCTION_VALUE /*        */ MOTOR_REDUCTION_VALUE
#define RIGHT_FRONT__ENCODER_PIN_A /*                */ 16
#define RIGHT_FRONT__ENCODER_PIN_B /*                */ 17
#define RIGHT_FRONT__ENCODER_WORK_MODE /*            */ ENCODER_WORK_MODE
#define RIGHT_FRONT__ENCODER_PINS_MODE /*            */ ENCODER_PINS_MODE
#define RIGHT_FRONT__ENCODER_REVERSE /*              */ TRUE
#define RIGHT_FRONT__ENCODER_TIKS_PER_REVOLUTION /*  */ ENCODER_TIKS_PER_REVOLUTION
#define RIGHT_FRONT__KP /*                           */ PID_Kp
#define RIGHT_FRONT__KI /*                           */ PID_Ki
#define RIGHT_FRONT__KD /*                           */ PID_Kd
#define RIGHT_FRONT__CONTROLLER_PERIOD /*                   */ CONTROLLER_PERIOD
#define RIGHT_FRONT__MAX_PWM /*                      */ MAX_PWM
#define RIGHT_FRONT__MIN_PWM /*                      */ MIN_PWM
#define RIGHT_FRONT__MIN_WORK_PWM /*                 */ MIN_WORK_PWM
#define RIGHT_FRONT__MAX_ANGULAR_VELOCITY /*         */ MAX_ANGULAR_VELOCITY
#define RIGHT_FRONT__MIN_ANGULAR_VELOCITY /*         */ MIN_ANGULAR_VELOCITY
#define RIGHT_FRONT__MIN_WORK_ANGULAR_VELOCITY /*    */ MIN_WORK_ANGULAR_VELOCITY
#define RIGHT_FRONT__MAX_RPM /*                      */ MAX_RPM
#define RIGHT_FRONT__MIN_RPM /*                      */ MIN_RPM
#define RIGHT_FRONT__MIN_WORK_RPM /*                 */ MIN_WORK_RPM

// ====================================================== //
// ============== LEFT BACK ACTUATOR PARAMS ============= //
// ====================================================== //
#define LEFT_BACK__MOTOR_PIN_A /*                    */ 8
#define LEFT_BACK__MOTOR_PIN_B /*                    */ 9
#define LEFT_BACK__MOTOR_REVERSE /*                  */ FALSE
#define LEFT_BACK__MOTOR_REDUCTION_VALUE /*          */ MOTOR_REDUCTION_VALUE
#define LEFT_BACK__ENCODER_PIN_A /*                  */ 11
#define LEFT_BACK__ENCODER_PIN_B /*                  */ 12
#define LEFT_BACK__ENCODER_WORK_MODE /*              */ ENCODER_WORK_MODE
#define LEFT_BACK__ENCODER_PINS_MODE /*              */ ENCODER_PINS_MODE
#define LEFT_BACK__ENCODER_REVERSE /*                */ TRUE
#define LEFT_BACK__ENCODER_TIKS_PER_REVOLUTION /*    */ ENCODER_TIKS_PER_REVOLUTION
#define LEFT_BACK__KP /*                             */ PID_Kp
#define LEFT_BACK__KI /*                             */ PID_Ki
#define LEFT_BACK__KD /*                             */ PID_Kd
#define LEFT_BACK__CONTROLLER_PERIOD /*                     */ CONTROLLER_PERIOD
#define LEFT_BACK__MAX_PWM /*                        */ MAX_PWM
#define LEFT_BACK__MIN_PWM /*                        */ MIN_PWM
#define LEFT_BACK__MIN_WORK_PWM /*                   */ MIN_WORK_PWM
#define LEFT_BACK__MAX_ANGULAR_VELOCITY /*           */ MAX_ANGULAR_VELOCITY
#define LEFT_BACK__MIN_ANGULAR_VELOCITY /*           */ MIN_ANGULAR_VELOCITY
#define LEFT_BACK__MIN_WORK_ANGULAR_VELOCITY /*      */ MIN_WORK_ANGULAR_VELOCITY
#define LEFT_BACK__MAX_RPM /*                        */ MAX_RPM
#define LEFT_BACK__MIN_RPM /*                        */ MIN_RPM
#define LEFT_BACK__MIN_WORK_RPM /*                   */ MIN_WORK_RPM

// ====================================================== //
// ============= RIGHT BACK ACTUATOR PARAMS ============= //
// ====================================================== //
#define RIGHT_BACK__MOTOR_PIN_A /*                   */ 2
#define RIGHT_BACK__MOTOR_PIN_B /*                   */ 3
#define RIGHT_BACK__MOTOR_REVERSE /*                 */ FALSE
#define RIGHT_BACK__MOTOR_REDUCTION_VALUE /*         */ MOTOR_REDUCTION_VALUE
#define RIGHT_BACK__ENCODER_PIN_A /*                 */ 18
#define RIGHT_BACK__ENCODER_PIN_B /*                 */ 19
#define RIGHT_BACK__ENCODER_WORK_MODE /*             */ ENCODER_WORK_MODE
#define RIGHT_BACK__ENCODER_PINS_MODE /*             */ ENCODER_PINS_MODE
#define RIGHT_BACK__ENCODER_REVERSE /*               */ TRUE
#define RIGHT_BACK__ENCODER_TIKS_PER_REVOLUTION /*   */ ENCODER_TIKS_PER_REVOLUTION
#define RIGHT_BACK__KP /*                            */ PID_Kp
#define RIGHT_BACK__KI /*                            */ PID_Ki
#define RIGHT_BACK__KD /*                            */ PID_Kd
#define RIGHT_BACK__CONTROLLER_PERIOD /*                    */ CONTROLLER_PERIOD
#define RIGHT_BACK__MAX_PWM /*                       */ MAX_PWM
#define RIGHT_BACK__MIN_PWM /*                       */ MIN_PWM
#define RIGHT_BACK__MIN_WORK_PWM /*                  */ MIN_WORK_PWM
#define RIGHT_BACK__MAX_ANGULAR_VELOCITY /*          */ MAX_ANGULAR_VELOCITY
#define RIGHT_BACK__MIN_ANGULAR_VELOCITY /*          */ MIN_ANGULAR_VELOCITY
#define RIGHT_BACK__MIN_WORK_ANGULAR_VELOCITY /*     */ MIN_WORK_ANGULAR_VELOCITY
#define RIGHT_BACK__MAX_RPM /*                       */ MAX_RPM
#define RIGHT_BACK__MIN_RPM /*                       */ MIN_RPM
#define RIGHT_BACK__MIN_WORK_RPM /*                  */ MIN_WORK_RPM

ACTUATOR actuators_list[ACTUATORS_COUNT];
actuator_cfg_t _actuator_config[ACTUATORS_COUNT] = {
    // ##################################################################### //
    // ######################## LEFT FRONT ACTUATOR ######################## //
    // ##################################################################### //
    {
        .motor_pin_A = /*                    */ LEFT_FRONT__MOTOR_PIN_A,
        .motor_pin_B = /*                    */ LEFT_FRONT__MOTOR_PIN_B,
        .motor_reverse = /*                  */ LEFT_FRONT__MOTOR_REVERSE,
        .motor_reduction = /*                */ LEFT_FRONT__MOTOR_REDUCTION_VALUE,
        .encoder_pin_A = /*                  */ LEFT_FRONT__ENCODER_PIN_A,
        .encoder_pin_B = /*                  */ LEFT_FRONT__ENCODER_PIN_B,
        .encoder_work_mode = /*              */ LEFT_FRONT__ENCODER_WORK_MODE,
        .encoder_pins_mode = /*              */ LEFT_FRONT__ENCODER_PINS_MODE,
        .encoder_reverse = /*                */ LEFT_FRONT__ENCODER_REVERSE,
        .encoder_tiks_per_revolution = /*    */ LEFT_FRONT__ENCODER_TIKS_PER_REVOLUTION,
        .Kp = /*                             */ LEFT_FRONT__KP,
        .Ki = /*                             */ LEFT_FRONT__KI,
        .Kd = /*                             */ LEFT_FRONT__KD,
        .period = /*                         */ LEFT_FRONT__CONTROLLER_PERIOD,
        .max_pwm = /*                        */ LEFT_FRONT__MAX_PWM,
        .min_pwm = /*                        */ LEFT_FRONT__MIN_PWM,
        .min_work_pwm = /*                   */ LEFT_FRONT__MIN_WORK_PWM,
        .max_w = /*                          */ LEFT_FRONT__MAX_ANGULAR_VELOCITY,
        .min_w = /*                          */ LEFT_FRONT__MIN_ANGULAR_VELOCITY,
        .min_work_w = /*                     */ LEFT_FRONT__MIN_WORK_ANGULAR_VELOCITY,
        .max_rpm = /*                        */ LEFT_FRONT__MAX_RPM,
        .min_rpm = /*                        */ LEFT_FRONT__MIN_RPM,
        .min_work_rpm = /*                   */ LEFT_FRONT__MIN_WORK_RPM,
    },
    // ##################################################################### //
    // ######################## RIGHT FRONT ACTUATOR ####################### //
    // ##################################################################### //
    {
        .motor_pin_A = /*                    */ RIGHT_FRONT__MOTOR_PIN_A,
        .motor_pin_B = /*                    */ RIGHT_FRONT__MOTOR_PIN_B,
        .motor_reverse = /*                  */ RIGHT_FRONT__MOTOR_REVERSE,
        .motor_reduction = /*                */ RIGHT_FRONT__MOTOR_REDUCTION_VALUE,
        .encoder_pin_A = /*                  */ RIGHT_FRONT__ENCODER_PIN_A,
        .encoder_pin_B = /*                  */ RIGHT_FRONT__ENCODER_PIN_B,
        .encoder_work_mode = /*              */ RIGHT_FRONT__ENCODER_WORK_MODE,
        .encoder_pins_mode = /*              */ RIGHT_FRONT__ENCODER_PINS_MODE,
        .encoder_reverse = /*                */ RIGHT_FRONT__ENCODER_REVERSE,
        .encoder_tiks_per_revolution = /*    */ RIGHT_FRONT__ENCODER_TIKS_PER_REVOLUTION,
        .Kp = /*                             */ RIGHT_FRONT__KP,
        .Ki = /*                             */ RIGHT_FRONT__KI,
        .Kd = /*                             */ RIGHT_FRONT__KD,
        .period = /*                         */ RIGHT_FRONT__CONTROLLER_PERIOD,
        .max_pwm = /*                        */ RIGHT_FRONT__MAX_PWM,
        .min_pwm = /*                        */ RIGHT_FRONT__MIN_PWM,
        .min_work_pwm = /*                   */ RIGHT_FRONT__MIN_WORK_PWM,
        .max_w = /*                          */ RIGHT_FRONT__MAX_ANGULAR_VELOCITY,
        .min_w = /*                          */ RIGHT_FRONT__MIN_ANGULAR_VELOCITY,
        .min_work_w = /*                     */ RIGHT_FRONT__MIN_WORK_ANGULAR_VELOCITY,
        .max_rpm = /*                        */ RIGHT_FRONT__MAX_RPM,
        .min_rpm = /*                        */ RIGHT_FRONT__MIN_RPM,
        .min_work_rpm = /*                   */ RIGHT_FRONT__MIN_WORK_RPM,
    },
    // ##################################################################### //
    // ######################### LEFT BACK ACTUATOR ######################## //
    // ##################################################################### //

    {
        .motor_pin_A = /*                    */ LEFT_BACK__MOTOR_PIN_A,
        .motor_pin_B = /*                    */ LEFT_BACK__MOTOR_PIN_B,
        .motor_reverse = /*                  */ LEFT_BACK__MOTOR_REVERSE,
        .motor_reduction = /*                */ LEFT_BACK__MOTOR_REDUCTION_VALUE,
        .encoder_pin_A = /*                  */ LEFT_BACK__ENCODER_PIN_A,
        .encoder_pin_B = /*                  */ LEFT_BACK__ENCODER_PIN_B,
        .encoder_work_mode = /*              */ LEFT_BACK__ENCODER_WORK_MODE,
        .encoder_pins_mode = /*              */ LEFT_BACK__ENCODER_PINS_MODE,
        .encoder_reverse = /*                */ LEFT_BACK__ENCODER_REVERSE,
        .encoder_tiks_per_revolution = /*    */ LEFT_BACK__ENCODER_TIKS_PER_REVOLUTION,
        .Kp = /*                             */ LEFT_BACK__KP,
        .Ki = /*                             */ LEFT_BACK__KI,
        .Kd = /*                             */ LEFT_BACK__KD,
        .period = /*                         */ LEFT_BACK__CONTROLLER_PERIOD,
        .max_pwm = /*                        */ LEFT_BACK__MAX_PWM,
        .min_pwm = /*                        */ LEFT_BACK__MIN_PWM,
        .min_work_pwm = /*                   */ LEFT_BACK__MIN_WORK_PWM,
        .max_w = /*                          */ LEFT_BACK__MAX_ANGULAR_VELOCITY,
        .min_w = /*                          */ LEFT_BACK__MIN_ANGULAR_VELOCITY,
        .min_work_w = /*                     */ LEFT_BACK__MIN_WORK_ANGULAR_VELOCITY,
        .max_rpm = /*                        */ LEFT_BACK__MAX_RPM,
        .min_rpm = /*                        */ LEFT_BACK__MIN_RPM,
        .min_work_rpm = /*                   */ LEFT_BACK__MIN_WORK_RPM,
    },
    // ##################################################################### //
    // ######################## RIGHT BACK ACTUATOR ######################## //
    // ##################################################################### //

    {
        .motor_pin_A = /*                    */ RIGHT_BACK__MOTOR_PIN_A,
        .motor_pin_B = /*                    */ RIGHT_BACK__MOTOR_PIN_B,
        .motor_reverse = /*                  */ RIGHT_BACK__MOTOR_REVERSE,
        .motor_reduction = /*                */ RIGHT_BACK__MOTOR_REDUCTION_VALUE,
        .encoder_pin_A = /*                  */ RIGHT_BACK__ENCODER_PIN_A,
        .encoder_pin_B = /*                  */ RIGHT_BACK__ENCODER_PIN_B,
        .encoder_work_mode = /*              */ RIGHT_BACK__ENCODER_WORK_MODE,
        .encoder_pins_mode = /*              */ RIGHT_BACK__ENCODER_PINS_MODE,
        .encoder_reverse = /*                */ RIGHT_BACK__ENCODER_REVERSE,
        .encoder_tiks_per_revolution = /*    */ RIGHT_BACK__ENCODER_TIKS_PER_REVOLUTION,
        .Kp = /*                             */ RIGHT_BACK__KP,
        .Ki = /*                             */ RIGHT_BACK__KI,
        .Kd = /*                             */ RIGHT_BACK__KD,
        .period = /*                         */ RIGHT_BACK__CONTROLLER_PERIOD,
        .max_pwm = /*                        */ RIGHT_BACK__MAX_PWM,
        .min_pwm = /*                        */ RIGHT_BACK__MIN_PWM,
        .min_work_pwm = /*                   */ RIGHT_BACK__MIN_WORK_PWM,
        .max_w = /*                          */ RIGHT_BACK__MAX_ANGULAR_VELOCITY,
        .min_w = /*                          */ RIGHT_BACK__MIN_ANGULAR_VELOCITY,
        .min_work_w = /*                     */ RIGHT_BACK__MIN_WORK_ANGULAR_VELOCITY,
        .max_rpm = /*                        */ RIGHT_BACK__MAX_RPM,
        .min_rpm = /*                        */ RIGHT_BACK__MIN_RPM,
        .min_work_rpm = /*                   */ RIGHT_BACK__MIN_WORK_RPM,
    },
};
// ###################################################################### //
// ################## EXT INTERRUPTS ENCODER A FUNCTIONS ################ //
// ###################################################################### //
void LEFT_FRONT_ENC_A_ISR()
{
    actuators_list[LEFT_FRONT].enc_A_ISR();
}
void RIGHT_FRONT_ENC_A_ISR()
{
    actuators_list[RIGHT_FRONT].enc_A_ISR();
}
void LEFT_BACK_ENC_A_ISR()
{
    actuators_list[LEFT_BACK].enc_A_ISR();
}
void RIGHT_BACK_ENC_A_ISR()
{
    actuators_list[RIGHT_BACK].enc_A_ISR();
}
void (*functptr_enc_A[])() = {
    LEFT_FRONT_ENC_A_ISR,
    RIGHT_FRONT_ENC_A_ISR,
    LEFT_BACK_ENC_A_ISR,
    RIGHT_BACK_ENC_A_ISR,
};

// ##################################################################### //
// ################# EXT INTERRUPTS ENCODER B FUNCTIONS ################ //
// ##################################################################### //
void LEFT_FRONT_ENC_B_ISR()
{
    actuators_list[LEFT_FRONT].enc_B_ISR();
}
void RIGHT_FRONT_ENC_B_ISR()
{
    actuators_list[RIGHT_FRONT].enc_B_ISR();
}
void LEFT_BACK_ENC_B_ISR()
{
    actuators_list[LEFT_BACK].enc_B_ISR();
}
void RIGHT_BACK_ENC_B_ISR()
{
    actuators_list[RIGHT_BACK].enc_B_ISR();
}
void (*functptr_enc_B[])() = {
    LEFT_FRONT_ENC_B_ISR,
    RIGHT_FRONT_ENC_B_ISR,
    LEFT_BACK_ENC_B_ISR,
    RIGHT_BACK_ENC_B_ISR,
};