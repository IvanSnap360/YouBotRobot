#ifndef __ACTUATOR_H__
#define __ACTUATOR_H__
#include <Arduino.h>

typedef struct 
{
    uint32_t motor_pinA,  motor_pinB;
    uint32_t encoder_pinA,encoder_pinB;

    bool reverse;
    double reduction;

    uint8_t start_deadzone_pwm_val;
    uint8_t max_pwm;

    double  max_velocity, min_velocity, min_pwm_velocity;

    double pid_hz;
    

}actuator_cfg_t;


class ACTUATOR
{
private:
    actuator_cfg_t* _config;
    double _kp = 0.0, _ki = 0.0, _kd = 0.0;

    signed long _raw_encoder_val = 0;

    double _target_velocity;
    double _current_velocity;
    float _integral = 0, _prevErr = 0;
    double _dt;

    uint32_t _last_tick_time = 0;

    int computePID(float input, float setpoint, float kp, float ki, float kd, float dt, int minOut, int maxOut) 
    {
        float err = setpoint - input;
        _integral = constrain(_integral + (float)err * dt * ki, minOut, maxOut);
        float D = (err - _prevErr) / dt;
        _prevErr = err;
        return constrain(err * kp + _integral + D * kd, minOut, maxOut);
    }


public:
    ACTUATOR();

    void setConfig(actuator_cfg_t* cfg);

    void setPID(double kp, double ki, double kd);
    void setVelocity(double vel);

    signed long getRawEncoderVal();

    double getVelocity();

    void tick();
    static void encA_ISR();
    static void encB_ISR();

    ~ACTUATOR();
};


#endif // __ACTUATOR_H__