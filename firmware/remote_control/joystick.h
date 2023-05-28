#ifndef __JOYSTICK_H__
#define __JOYSTICK_H__
#include <Arduino.h>
class JOYSTICK
{
private:
    int _pinx,_piny,_pinb;
    int _maxout,_minout;
public:
    JOYSTICK();
    void init(int pinx,int piny, int pinb,int maxout,int minout);
    int getXMapValue();
    int getYMapValue();
    int getXRawValue();
    int getYRawValue();
    int getButValue();
};



#endif // __JOYSTICK_H__