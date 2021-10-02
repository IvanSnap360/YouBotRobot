#include <ros.h>
#include "config.h"
#include <std_msgs/Int16.h>

ros::NodeHandle nh;

void FL_cb(const std_msgs::Int16 & data);
void FR_cb(const std_msgs::Int16 & data);
void BL_cb(const std_msgs::Int16 & data);
void BR_cb(const std_msgs::Int16 & data);

ros::Subscriber<std_msgs::Int16> FL_sub(FL_WHEEL_SUBTOPIC_NAME,&FL_cb);
ros::Subscriber<std_msgs::Int16> FR_sub(FR_WHEEL_SUBTOPIC_NAME,&FR_cb);
ros::Subscriber<std_msgs::Int16> BL_sub(BL_WHEEL_SUBTOPIC_NAME,&BL_cb);
ros::Subscriber<std_msgs::Int16> BR_sub(BR_WHEEL_SUBTOPIC_NAME,&BR_cb);

unsigned long long last_time = 0;

void setup()
{
    nh.initNode();

    pinMode(FL_MOTOR_PIN_1,OUTPUT);
    pinMode(FL_MOTOR_PIN_2,OUTPUT);
    pinMode(FR_MOTOR_PIN_1,OUTPUT);
    pinMode(FR_MOTOR_PIN_2,OUTPUT);
    pinMode(BL_MOTOR_PIN_1,OUTPUT);
    pinMode(BL_MOTOR_PIN_2,OUTPUT);
    pinMode(BR_MOTOR_PIN_1,OUTPUT);
    pinMode(BR_MOTOR_PIN_2,OUTPUT);
    pinMode(LED_BUILTIN,OUTPUT);
}

void loop()
{
    if (nh.connected())
    {

        if (millis() - last_time > 1000)
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
    }
    nh.spinOnce();
}