#include "actuator.h"
void ACTUATOR::setConfig(actuator_config_t *config)
{
    _cfg = config;

    pinMode(_cfg->motor_pin1, OUTPUT);
    pinMode(_cfg->motor_pin2, OUTPUT);

    pinMode(_cfg->encoder_pin_A, _cfg->inpin_work_mode);
    pinMode(_cfg->encoder_pin_B, _cfg->inpin_work_mode);

    _pid_dt = 1000.0 / _cfg->pid_rate_hz;

    _enc_A_state = digitalRead(_cfg->encoder_pin_A);
    _enc_B_state = digitalRead(_cfg->encoder_pin_B);
}

void ACTUATOR::setVelocity(double vel)
{
    _target_velocity = vel;
}

double ACTUATOR::getVelocity()
{
    return _current_velocity;
}

void ACTUATOR::encA_ISR()
{
    _current_velocity = 60 / ((double)(micros() - _last_encoder_flash_time) / 1000000);
    _last_encoder_flash_time = micros();

    _enc_A_state = digitalRead(_cfg->encoder_pin_A);
    _enc_A_state == _enc_B_state ? _relative_encoder_tick++ : _relative_encoder_tick--;
}

void ACTUATOR::encB_ISR()
{
    _enc_B_state = digitalRead(_cfg->encoder_pin_B);
    _enc_A_state == _enc_B_state ? _relative_encoder_tick++ : _relative_encoder_tick--;
}

void ACTUATOR::tick()
{

    if ((micros() - _last_encoder_flash_time) > 500e3)
    {
        _current_velocity = 0;
    }

    if (millis() - _last_compute_time > (uint32_t)_pid_dt)
    {
        if (_target_velocity == 0.0)
        {
            _prevErr = 0.0;
            _integral = 0.0;
        }

        _compute_velocity = this->computePID(_current_velocity, _target_velocity,
                                             _cfg->kp, _cfg->ki, _cfg->kd,
                                             _pid_dt,
                                             -_cfg->max_pwm, _cfg->max_pwm);
    }

    this->setMotor(_compute_velocity);
}

ACTUATOR::~ACTUATOR()
{
}

ACTUATOR::ACTUATOR()
{
}

double ACTUATOR::computePID(double input, double setpoint, double kp, double ki, double kd, double dt, double minOut, double maxOut)
{
    double err = setpoint - input;
    _integral = 0, _prevErr = 0;
    _integral = constrain(_integral + (float)err * dt * ki, minOut, maxOut);
    float D = (err - _prevErr) / dt;
    _prevErr = err;
    return constrain(err * kp + _integral + D * kd, minOut, maxOut);
}

void ACTUATOR::setMotor(double val)
{
    if (val > 0 && val <= _cfg->min_work_pwm)
        val = _cfg->min_work_pwm;
    if (val < 0 && val >= _cfg->min_work_pwm)
        val = -_cfg->min_work_pwm;

    if (val > 0)
    {
        analogWrite(_cfg->motor_reverse ? _cfg->motor_pin2 : _cfg->motor_pin1, 0);
        analogWrite(_cfg->motor_reverse ? _cfg->motor_pin1 : _cfg->motor_pin2, (uint32_t)abs(val));
    }
    else if (val > 0)
    {
        analogWrite(_cfg->motor_reverse ? _cfg->motor_pin2 : _cfg->motor_pin1, (uint32_t)abs(val));
        analogWrite(_cfg->motor_reverse ? _cfg->motor_pin1 : _cfg->motor_pin2, 0);
    }
    else
    {
        analogWrite(_cfg->motor_pin1, 0);
        analogWrite(_cfg->motor_pin2, 0);
    }
}
