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

    _encA_flg = digitalRead(_cfg->encoder_pin_A);
    _encB_flg = digitalRead(_cfg->encoder_pin_B);
}

actuator_cfg_t *ACTUATOR::getConfig()
{
    return _cfg;
}

void ACTUATOR::setVelocity(double velocity)
{
    _target_velocity = velocity;
}

double ACTUATOR::getVelocity()
{
    return _current_velocity;
}

void ACTUATOR::setRPM(int rpm)
{
    _target_rpm = rpm;
    setVelocity(RPM_2_RADS(rpm));
}

int ACTUATOR::getRPM()
{
    return _current_rpm;
}

int ACTUATOR::getEncoderTiks()
{
    return _enc_ticks;
}

int ACTUATOR::computePID(float input, float setpoint, float kp, float ki, float kd, float dt, int minOut, int maxOut)
{
    float err = setpoint - input;
    _integral = _integral + (float)err * ki;
    float D = (err - _prevErr) / dt;
    _prevErr = err;
    return (err * kp + _integral + D * kd);
}
void ACTUATOR::setPWM(int val)
{
    _cntrl_val = val;
}



int ACTUATOR::getPWM()
{
    return _cntrl_val;
}
void ACTUATOR::tick()
{

    if (millis() - _last_tick_time_val > _cfg->period)
    {
        _current_rpm = _tacho.getRPM() / _cfg->motor_reduction;
        _current_velocity = RPM_2_RADS(_current_rpm); // current rpm to rads/s
        _cntrl_val = computePID(_current_rpm, _target_rpm,
                                //
                                (float)_cfg->Kp,
                                (float)_cfg->Ki,
                                (float)_cfg->Kd,
                                //
                                _cfg->period,
                                //
                                _cfg->min_work_pwm,
                                _cfg->max_pwm);

        if (_cntrl_val < 0 && _cntrl_val >= -_cfg->min_work_pwm)
            _cntrl_val = -_cfg->min_work_pwm;

        if (_cntrl_val > 0 && _cntrl_val <= _cfg->min_work_pwm)
            _cntrl_val = _cfg->min_work_pwm;

        if (_cntrl_val < 0 && _cntrl_val <= -_cfg->max_pwm)
            _cntrl_val = -_cfg->max_pwm;

        if (_cntrl_val > 0 && _cntrl_val >= _cfg->max_pwm)
            _cntrl_val = _cfg->max_pwm;

        if (_cntrl_val == 0)
        {
            analogWrite(_cfg->motor_reverse ? _cfg->motor_pin_B : _cfg->motor_pin_A, 0);
            analogWrite(_cfg->motor_reverse ? _cfg->motor_pin_A : _cfg->motor_pin_B, 0);
        }
        else if (_cntrl_val > 0)
        {
            analogWrite(_cfg->motor_reverse ? _cfg->motor_pin_B : _cfg->motor_pin_A, abs(_cntrl_val));
            analogWrite(_cfg->motor_reverse ? _cfg->motor_pin_A : _cfg->motor_pin_B, 0);
        }
        else if (_cntrl_val < 0)
        {
            analogWrite(_cfg->motor_reverse ? _cfg->motor_pin_B : _cfg->motor_pin_A, 0);
            analogWrite(_cfg->motor_reverse ? _cfg->motor_pin_A : _cfg->motor_pin_B, abs(_cntrl_val));
        }

        _last_tick_time_val = millis();
    }
}

void ACTUATOR::enc_A_ISR()
{
    // ~~~~~~~~~ GYVER OLD TACHOMETR ~~~~~~~~~ //
    // _current_rpm = (60/((float)(micros()-_lastflash)/1000000)) / (_cfg->encoder_tiks_per_revolution * _cfg->motor_reduction);
    // _lastflash = micros();
    //
    // ~~~~~ MY RELATIVE ENCODER COUNTER ~~~~~ //
    //    _encA_flg = digitalReadFast(_cfg->encoder_pin_A);
    //    _encA_flg != _encB_flg ? (_cfg->encoder_reverse ? _enc_ticks-- : _enc_ticks++) : (_cfg->encoder_reverse ? _enc_ticks++ : _enc_ticks--);
    //
    // ~~~~~~~~~ GYVER NEW TACHOMETR ~~~~~~~~~ //
    if (_enc_ticks == 0)
    {
        _tacho.tick();
        _enc_ticks = _cfg->encoder_tiks_per_revolution;
    }
    else
    {
        _enc_ticks--;
    }
        
}

void ACTUATOR::enc_B_ISR()
{
    // _encB_flg = digitalReadFast(_cfg->encoder_pin_B);
    // _encA_flg == _encB_flg ? (_cfg->encoder_reverse ? _enc_ticks-- : _enc_ticks++) : (_cfg->encoder_reverse ? _enc_ticks++ : _enc_ticks--);
}

ACTUATOR::~ACTUATOR()
{
}
