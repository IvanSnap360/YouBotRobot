#include "actuator.h"

ACTUATOR::ACTUATOR()
{
}

void ACTUATOR::setConfig(actuator_cfg_t *cfg)
{
    _cfg = cfg;

    pinMode(_cfg->motor_pin_A, OUTPUT);
    pinMode(_cfg->motor_pin_B, OUTPUT);
    analogWrite(_cfg->motor_pin_A, 0);
    analogWrite(_cfg->motor_pin_B, 0);

    pinMode(_cfg->encoder_pin_A, _cfg->encoder_pins_mode);
    pinMode(_cfg->encoder_pin_B, _cfg->encoder_pins_mode);
}

const actuator_cfg_t* ACTUATOR::getConfig()
{
    return (const actuator_cfg_t*)_cfg;
}

void ACTUATOR::setVelocity(double velocity)
{
    _target_velocity = velocity;
}

double ACTUATOR::getVelocity()
{
    return _current_velocity;
}

void ACTUATOR::tick()
{
    if (millis() - _last_tick_time_val > _cfg->period)
    {
        if (_target_velocity == 0.0)
        {
            _lastI = 0.0;
            _lastErr = 0.0;
        }

        double _err = _target_velocity - _current_velocity;

        double P = _cfg->Kp * _err;
        double I = _lastI + (_cfg->Ki * _err);
        double D = _cfg->Kd * (_err - _lastErr);
        double val = round(P + I + D);

        _lastI = I;
        _lastErr = _err;



        val = constrain(val, -_cfg->max_pwm,_cfg->max_pwm);

        if (val < 0 && val >= -_cfg->min_work_pwm)
            val = -_cfg->min_work_pwm;

        if (val < 0 && val >= _cfg->min_work_pwm)
            val = _cfg->min_work_pwm;

        if (val == 0)
        {
            analogWrite(_cfg->motor_reverse ? _cfg->motor_pin_B : _cfg->motor_pin_A, 0);
            analogWrite(_cfg->motor_reverse ? _cfg->motor_pin_A : _cfg->motor_pin_B, 0);
        }
        else if (val > 0)
        {
            analogWrite(_cfg->motor_reverse ? _cfg->motor_pin_B : _cfg->motor_pin_A, abs(val));
            analogWrite(_cfg->motor_reverse ? _cfg->motor_pin_A : _cfg->motor_pin_B, 0);
        }
        else if (val < 0)
        {
            analogWrite(_cfg->motor_reverse ? _cfg->motor_pin_B : _cfg->motor_pin_A, 0);
            analogWrite(_cfg->motor_reverse ? _cfg->motor_pin_A : _cfg->motor_pin_B, abs(val));
        }


        _last_tick_time_val = millis();
    }
    if ((micros() - _lastflash) > 1000000)
        _current_velocity = 0;
}

void ACTUATOR::enc_A_ISR()
{
    if (counter == 0)
    {
        _current_velocity = RPM_2_RADS((60/((float)(micros()-_lastflash)/1000000) / _cfg->motor_reduction));
        _lastflash = micros();
        counter = _cfg->encoder_tiks_per_revolution;
    }
    else
    {
        counter--;
    }
}

void ACTUATOR::enc_B_ISR()
{
}

ACTUATOR::~ACTUATOR()
{
}
