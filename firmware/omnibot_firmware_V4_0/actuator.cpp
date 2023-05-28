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
    _integral = _integral + (float)err * dt * ki;
    float D = (err - _prevErr) / dt;
    _prevErr = err;
    return constrain(err * kp + _integral + D * kd, minOut, maxOut);
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
        _current_rpm = 0;
        if(micros() - _lastflash > 10000 && (_tacho_overflow || _tacho_values_ptr > 0)) {
            for(int i = 0; i < TACHO_AMOUNT; i++)
                _tacho_values[i] = 0;
            _tacho_values_ptr = 0;
            _tacho_overflow = false;
        }
        if(_tacho_overflow || _tacho_values_ptr > 0) {
            double tacho_sum = 0;
            for(int i = 0; i < (_tacho_overflow ? TACHO_AMOUNT : _tacho_values_ptr); i++)
                tacho_sum += _tacho_values[i];
            _current_rpm = tacho_sum/(_tacho_overflow ? TACHO_AMOUNT : _tacho_values_ptr); 
        }
        // _current_rpm = _tacho.guetRPM() / _cfg->motor_reduction;
        _current_velocity = RPM_2_RADS(_current_rpm); // current rpm to rads/s

        if (_target_rpm == 0)
        {
            _integral = 0;
            _prevErr = 0;
        }

        _cntrl_val = computePID(_current_rpm, abs(_target_rpm),
                                //
                                (float)_cfg->Kp,
                                (float)_cfg->Ki,
                                (float)_cfg->Kd,
                                //
                                _cfg->period / 1000.0,
                                //
                                _cfg->min_pwm,
                                _cfg->max_pwm) * (_target_rpm < 0 ? -1 : 1);
        /*
        if (_cntrl_val < 0 && _cntrl_val >= -_cfg->min_work_pwm)
            _cntrl_val = -_cfg->min_work_pwm;

        if (_cntrl_val > 0 && _cntrl_val <= _cfg->min_work_pwm)
            _cntrl_val = _cfg->min_work_pwm;

        if (_cntrl_val < 0 && _cntrl_val <= -_cfg->max_pwm)
            _cntrl_val = -_cfg->max_pwm;

        if (_cntrl_val > 0 && _cntrl_val >= _cfg->max_pwm)
            _cntrl_val = _cfg->max_pwm;
        */
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
    uint8_t v = (60/((float)(micros()-_lastflash)/1000000)) / (_cfg->encoder_tiks_per_revolution * _cfg->motor_reduction);
    _tacho_values[_tacho_values_ptr] = v;
    _tacho_values_ptr++;
    if(_tacho_values_ptr >= TACHO_AMOUNT) {
        _tacho_overflow = true;
        _tacho_values_ptr = 0;
    }
      
    _lastflash = micros();

    // ~~~~~~~~~ GYVER OLD TACHOMETR ~~~~~~~~~ //
    //_current_rpm = (60/((float)(micros()-_lastflash)/1000000)) / (_cfg->encoder_tiks_per_revolution * _cfg->motor_reduction);
    //_lastflash = micros();
    //
    // ~~~~~ MY RELATIVE ENCODER COUNTER ~~~~~ //
    //    _encA_flg = digitalReadFast(_cfg->encoder_pin_A);
    //    _encA_flg != _encB_flg ? (_cfg->encoder_reverse ? _enc_ticks-- : _enc_ticks++) : (_cfg->encoder_reverse ? _enc_ticks++ : _enc_ticks--);
    //
    // ~~~~~~~~~ GYVER NEW TACHOMETR ~~~~~~~~~ //
    // if (_enc_ticks == 0)
    // {
    //     _tacho.tick();
    //     _enc_ticks = _cfg->encoder_tiks_per_revolution;
    // }
    // else
    // {
    //     _enc_ticks--;
    // }
        
}

void ACTUATOR::enc_B_ISR()
{
    // _encB_flg = digitalReadFast(_cfg->encoder_pin_B);
    // _encA_flg == _encB_flg ? (_cfg->encoder_reverse ? _enc_ticks-- : _enc_ticks++) : (_cfg->encoder_reverse ? _enc_ticks++ : _enc_ticks--);
}

ACTUATOR::~ACTUATOR()
{
}
