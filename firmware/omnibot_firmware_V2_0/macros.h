#include <Arduino.h>


#define EVERY_MS(x,tmr_lt) \
  bool flag = millis() - tmr_lt >= (x);\
  if (flag) tmr_lt = millis();\
  if (flag)