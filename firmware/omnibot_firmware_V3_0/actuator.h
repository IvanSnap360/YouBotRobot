#ifndef __ACTUATOR_H__
#define __ACTUATOR_H__
#include <Arduino.h>
typedef struct
{
    uint32_t motor_pin1, motor_pin2;
    uint32_t encoder_pin_A, encoder_pin_B;
    uint32_t inpin_work_mode;

    bool motor_reverse;
    bool encoder_reverse;

    double kp, ki, kd;
    double pid_rate_hz;

    double motor_reduction;
    uint8_t max_pwm, min_pwm, min_work_pwm;
    double max_velocity,min_velocity, min_work_velocity;
} actuator_config_t;

class ACTUATOR
{
private:
uint32_t _last_compute_time;
    actuator_config_t *_cfg;

    double _target_velocity;
    double _current_velocity;
    double _compute_velocity;


    double _integral, _prevErr;

    double computePID(double input, double setpoint, double kp, double ki, double kd, double dt, double minOut, double maxOut);
    void setMotor(double val);
    double _pid_dt;


    bool _enc_A_state,_enc_B_state;


    uint32_t _encoder_tick;  
public:
    ACTUATOR();
    void setConfig(actuator_config_t *config);
    void setVelocity(double vel);
    double getVelocity();

    void encA_ISR();
    void encB_ISR();

    void tick();

    ~ACTUATOR();
};

#endif // __ACTUATOR_H__
