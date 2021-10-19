#include "joystick.h"
JOYSTICK::JOYSTICK()
{

}

void JOYSTICK::init(int pinx,int piny, int pinb,int maxout,int minout)
{
    _pinx = pinx;
    _piny = piny;
    _pinb = pinb;
    _maxout = maxout;
    _minout = minout;
    pinMode(_pinb, INPUT);
}

int JOYSTICK::getXMapValue()
{
    return map(analogRead(_pinx),0,1023,_minout,_maxout);
}
int JOYSTICK::getYMapValue()
{
    return map(analogRead(_piny),0,1023,_minout,_maxout);
}

int JOYSTICK::getXRawValue()
{
    return analogRead(_pinx);
}
int JOYSTICK::getYRawValue()
{
    return analogRead(_piny);
}

int JOYSTICK::getButValue()
{
    if(digitalRead(_pinb) == true) 
        return 1.0;
    else 
        return -1.0;
}