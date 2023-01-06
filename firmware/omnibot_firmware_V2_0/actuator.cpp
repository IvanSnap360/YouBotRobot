#include "actuator.h"

void ACTUATOR::setConfig(actuator_cfg_t *cfg)
{
    _config = cfg;
    _dt = (uint32_t)(1000.0 / _config->pid_hz);

    pinMode(_config->motor_pinA, OUTPUT);
    pinMode(_config->motor_pinB, OUTPUT);

    pinMode(digitalPinToInterrupt(_config->encoder_pinA), INPUT_PULLDOWN);
    pinMode(digitalPinToInterrupt(_config->encoder_pinB), INPUT_PULLDOWN);
}

void ACTUATOR::setPID(double kp, double ki, double kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void ACTUATOR::setVelocity(double vel)
{
    _target_velocity = vel;
}

signed long ACTUATOR::getRawEncoderVal()
{
    return _raw_encoder_val;
}

double ACTUATOR::getVelocity()
{
    return _current_velocity;
}

void ACTUATOR::tick()
{
    if (millis() - _last_tick_time > _dt)
    {
        double vel = computePID(_current_velocity, _target_velocity, _kp, _ki, _kd, _dt, _config->min_pwm_velocity, _config->max_velocity);

        int pwm = map(vel, _config->min_velocity, _config->max_velocity, -_config->max_pwm, _config->max_pwm);

        if (_config->max_pwm > 0)
        {
            analogWrite((_config->reverse)  ? _config->motor_pinA : _config->motor_pinB, pwm);
            analogWrite((!_config->reverse) ? _config->motor_pinA : _config->motor_pinB, 000);
        }
        else
        {
            analogWrite((!_config->reverse) ? _config->motor_pinA : _config->motor_pinB, pwm);
            analogWrite((_config->reverse)  ? _config->motor_pinA : _config->motor_pinB, 000);
        }

        _last_tick_time = millis();
    }
}

void ACTUATOR::encA_ISR()
{
    
}

void ACTUATOR::encB_ISR()
{
    
}

ACTUATOR::ACTUATOR()
{
}
ACTUATOR::~ACTUATOR()
{
}
