#include <ros.h>
#include "config.h"
#include <std_msgs/Float32.h>
#define msg_type std_msgs::Float32
ros::NodeHandle nh;

void FL_cb(const msg_type &data);
void FR_cb(const msg_type &data);
void BL_cb(const msg_type &data);
void BR_cb(const msg_type &data);

ros::Subscriber<msg_type> FL_sub(FL_WHEEL_SUBTOPIC_NAME, &FL_cb);
ros::Subscriber<msg_type> FR_sub(FR_WHEEL_SUBTOPIC_NAME, &FR_cb);
ros::Subscriber<msg_type> BL_sub(BL_WHEEL_SUBTOPIC_NAME, &BL_cb);
ros::Subscriber<msg_type> BR_sub(BR_WHEEL_SUBTOPIC_NAME, &BR_cb);

unsigned long long last_time = 0;

void setup()
{

  nh.initNode();
  nh.subscribe(FL_sub);
  nh.subscribe(FR_sub);
  nh.subscribe(BL_sub);
  nh.subscribe(BR_sub);

  pinMode(FL_MOTOR_PIN_1, OUTPUT);
  pinMode(FL_MOTOR_PIN_2, OUTPUT);
  pinMode(FR_MOTOR_PIN_1, OUTPUT);
  pinMode(FR_MOTOR_PIN_2, OUTPUT);
  pinMode(BL_MOTOR_PIN_1, OUTPUT);
  pinMode(BL_MOTOR_PIN_2, OUTPUT);
  pinMode(BR_MOTOR_PIN_1, OUTPUT);
  pinMode(BR_MOTOR_PIN_2, OUTPUT);

  pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
  if (nh.connected())
  {

    if (millis() - last_time > 50)
    {
      toggleLED();
      last_time = millis();
    }
  }
  else
  {
    if (millis() - last_time > 100)
    {
      toggleLED();
      last_time = millis();
    }
    digitalWrite(FL_MOTOR_PIN_1, 0);
    digitalWrite(FL_MOTOR_PIN_2, 0);
    digitalWrite(FR_MOTOR_PIN_1, 0);
    digitalWrite(FR_MOTOR_PIN_2, 0);
    digitalWrite(BL_MOTOR_PIN_1, 0);
    digitalWrite(BL_MOTOR_PIN_2, 0);
    digitalWrite(BR_MOTOR_PIN_1, 0);
    digitalWrite(BR_MOTOR_PIN_2, 0);
  }
  nh.spinOnce();
}

void FL_cb(const msg_type &data)
{
  if (data.data > 0)
  {
    analogWrite(FL_MOTOR_PIN_1, abs(data.data));
    analogWrite(FL_MOTOR_PIN_2, 0);
  }
  else if (data.data < 0)
  {
    analogWrite(FL_MOTOR_PIN_1, 0);
    analogWrite(FL_MOTOR_PIN_2, abs(data.data));
  }
  else
  {
    analogWrite(FL_MOTOR_PIN_1, 0);
    analogWrite(FL_MOTOR_PIN_2, 0);
  }
}
void FR_cb(const msg_type &data)
{
  if (data.data > 0)
  {
    analogWrite(FR_MOTOR_PIN_1, abs(data.data));
    analogWrite(FR_MOTOR_PIN_2, 0);
  }
  else if (data.data < 0)
  {
    analogWrite(FR_MOTOR_PIN_1, 0);
    analogWrite(FR_MOTOR_PIN_2, abs(data.data));
  }
  else
  {
    analogWrite(FR_MOTOR_PIN_1, 0);
    analogWrite(FR_MOTOR_PIN_2, 0);
  }
}
void BL_cb(const msg_type &data)
{
  if (data.data > 0)
  {
    analogWrite(BL_MOTOR_PIN_1, abs(data.data));
    analogWrite(BL_MOTOR_PIN_2, 0);
  }
  else if (data.data < 0)
  {
    analogWrite(BL_MOTOR_PIN_1, 0);
    analogWrite(BL_MOTOR_PIN_2, abs(data.data));
  }
  else
  {
    analogWrite(BL_MOTOR_PIN_1, 0);
    analogWrite(BL_MOTOR_PIN_2, 0);
  }
}
void BR_cb(const msg_type &data)
{
  if (data.data > 0)
  {
    analogWrite(BR_MOTOR_PIN_1, abs(data.data));
    analogWrite(BR_MOTOR_PIN_2, 0);
  }
  else if (data.data < 0)
  {
    analogWrite(BR_MOTOR_PIN_1, 0);
    analogWrite(BR_MOTOR_PIN_2, abs(data.data));
  }
  else
  {
    analogWrite(BR_MOTOR_PIN_1, 0);
    analogWrite(BR_MOTOR_PIN_2, 0);
  }
}
