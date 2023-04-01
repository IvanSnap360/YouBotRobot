#ifndef __ACTUATOR_H__
#define __ACTUATOR_H__
#include <stdint.h>
#include <Arduino.h>
#define RPM_2_RADS(rpm) (double)(rpm * (PI / 30))
#define RADS_2_RPM(rads) (double)((rads * 30) / PI)
typedef struct actuator_cfg_s
{
    // ====================================================== //
    // ==================== MOTOR PARAMS ==================== //
    // ====================================================== //
    uint32_t motor_pin_A; /*                                  */ // motor pin 1
    uint32_t motor_pin_B; /*                                  */ // motor pin 2
    bool motor_reverse; /*                                    */ // enable motor reverse
    double motor_reduction; /*                                */ // motor reductor value (e.g. 1 / 100)
    // ====================================================== //
    // =================== ENCODER PARAMS =================== //
    // ====================================================== //
    uint32_t encoder_pin_A; /*                                */ // encoder A pin 
    uint32_t encoder_pin_B; /*                                */ // encoder B pin
    uint32_t encoder_work_mode; /*                            */ // encoder work mode(RAISE, FALLING, CHANGE)
    uint32_t encoder_pins_mode; /*                            */ // encoder pins mode (INPUT_PULLDOWN, INPUT_PULLUP)
    bool encoder_reverse; /*                                  */ // enable encoder reverse
    uint32_t encoder_tiks_per_revolution; /*                  */ // encoder tiks per one motor shaft revolution
    // ====================================================== //
    // ===================== PID PARAMS ===================== //
    // ====================================================== //
    double Kp; /*                                             */ // PID proportional coef
    double Ki; /*                                             */ // PID integral coef
    double Kd; /*                                             */ // PID differential coef
    uint32_t period; /*                                       */ // compute period 
    // ====================================================== //
    // ===================== PWM PARAMS ===================== //
    // ====================================================== //
    uint32_t max_pwm; /*                                      */ // max pwm for motor
    uint32_t min_pwm; /*                                      */ // min pwm for motor
    uint32_t min_work_pwm; /*                                 */ // min work pwm value for motor shaft start rotating
    // ====================================================== //
    // =============== ANGULAR VELOCITY PARAMS ============== //
    // ====================================================== //
    double max_w; /*                                          */ // max angular velocity for motor shaft
    double min_w; /*                                          */ // min angular velocity for motor shaft
    double min_work_w; /*                                     */ // min work angular velocity for motor shaft start rotating
    // ====================================================== //
    // ===================== RPM PARAMS ===================== //
    // ====================================================== //
    double max_rpm; /*                                        */ // max rpm for motor for motor shaft
    double min_rpm; /*                                        */ // min rpm for motor for motor shaft
    double min_work_rpm; /*                                   */ // min work rpm for motor shaft start rotating
    // ====================================================== //
    // ====================================================== //
    // ====================================================== //
} actuator_cfg_t;

class ACTUATOR
{
private:
    actuator_cfg_t *_cfg;
    double _lastErr, _lastI;
    double _target_velocity;
    double _current_velocity;

    uint32_t _last_tick_time_val;
    uint32_t _lastflash;
    int counter;


public:
    ACTUATOR();
    void setConfig(actuator_cfg_t *cfg);
    const actuator_cfg_t *getConfig();
    void setVelocity(double velocity);
    double getVelocity();
    void tick();

    void enc_A_ISR();
    void enc_B_ISR();

    

    ~ACTUATOR();
};

#endif // __ACTUATOR_H__