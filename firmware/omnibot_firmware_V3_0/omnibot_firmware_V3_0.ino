#include "actuator.h"
#include "config.h"

#ifdef WORK_MODE__SERIAL
#endif // WORK_MODE__SERIAL

#ifdef WORK_MODE__ROS
#include <ros.h>
#include <std_msgs/Float64.h>
#endif // WORK_MODE__ROS

#ifdef WORK_MODE__ROS
ros::NodeHandle_<
    ROS_SERIAL__HARDWARE,
    ROS_SERIAL__MAX_SUBSCRIBERS,
    ROS_SERIAL__MAX_PUBLISHERS,
    ROS_SERIAL__MAX_INPUT_BUFFER_SIZE,
    ROS_SERIAL__MAX_OUTPUT_BUFFER_SIZE>
    nh;

std_msgs::Float64 wheel_target_velocity[ACTUATORS_COUNT];

void left_front_controller_sub_cb_f(const std_msgs::Float64 &val);
void right_front_controller_sub_cb_f(const std_msgs::Float64 &val);
void left_back_controller_sub_cb_f(const std_msgs::Float64 &val);
void right_back_controller_sub_cb_f(const std_msgs::Float64 &val);

ros::Subscriber<std_msgs::Float64> left_front_controller_sub /* */ (ROS_TOPIC_LEFT_FRONT_WHEEL_CONTROLLER, /* */ left_front_controller_sub_cb_f);
ros::Subscriber<std_msgs::Float64> right_front_controller_sub /**/ (ROS_TOPIC_RIGHT_FRONT_WHEEL_CONTROLLER, /**/ right_front_controller_sub_cb_f);
ros::Subscriber<std_msgs::Float64> left_back_controller_sub /*  */ (ROS_TOPIC_LEFT_BACK_WHEEL_CONTROLLER, /*  */ left_back_controller_sub_cb_f);
ros::Subscriber<std_msgs::Float64> right_back_controller_sub /* */ (ROS_TOPIC_RIGHT_BACK_WHEEL_CONTROLLER, /* */ right_back_controller_sub_cb_f);
#endif // WORK_MODE__ROS
void setup()
{
#ifdef WORK_MODE__ROS
    nh.getHardware()->setBaud(ROS_SERIAL__BAUDRATE);
    nh.subscribe(left_front_controller_sub);
    nh.subscribe(right_front_controller_sub);
    nh.subscribe(left_back_controller_sub);
    nh.subscribe(right_back_controller_sub);
    nh.initNode();
#endif // WORK_MODE__ROS

#ifdef WORK_MODE__SERIAL
    Serial.begin(COM_SERIAL__BAUDRATE);

    while (!Serial)
    {
        if (millis() - blink_last_time > (uint32_t)(1000.0 / DISCONNECTION_LED_BLINK_RATE_HZ))
        {
            toggleLED();
            blink_last_time = millis();
        }
    }
#endif // WORK_MODE__SERIAL

    for (int i = 0; i < ACTUATORS_COUNT; i++)
    {
        actuators[i].setConfig(&_actuator_config[i]);
        attachInterrupt(_actuator_config->encoder_pin_A, (*functptr_enc_A[i]), ENCODER_WORK_MODE);
        attachInterrupt(_actuator_config->encoder_pin_B, (*functptr_enc_B[i]), ENCODER_WORK_MODE);
    }
}

void loop()
{
   
    for (auto &actuator : actuators)
        actuator.tick();

#ifdef WORK_MODE__ROS
    if (nh.connected())
    {
        if (millis() - blink_last_time > (uint32_t)(1000.0 / CONNECTION_LED_BLINK_RATE_HZ))
        {
            toggleLED();
            blink_last_time = millis();
        }
    }
    else
    {
        if (millis() - blink_last_time > (uint32_t)(1000.0 / DISCONNECTION_LED_BLINK_RATE_HZ))
        {
            toggleLED();
            blink_last_time = millis();
        }
    }
    nh.spinOnce();
#endif // WORK_MODE__ROS
}

#ifdef WORK_MODE__ROS
void left_front_controller_sub_cb_f(const std_msgs::Float64 &val)
{
    actuators[LEFT_FRONT].setVelocity((double)val.data);
}
void right_front_controller_sub_cb_f(const std_msgs::Float64 &val)
{
    actuators[RIGHT_FRONT].setVelocity((double)val.data);
}
void left_back_controller_sub_cb_f(const std_msgs::Float64 &val)
{
    actuators[LEFT_BACK].setVelocity((double)val.data);
}
void right_back_controller_sub_cb_f(const std_msgs::Float64 &val)
{
    actuators[RIGHT_BACK].setVelocity((double)val.data);
}
#endif // WORK_MODE__ROS

#ifdef WORK_MODE__SERIAL
void serial_pid_setup()
{
    static int actuator = -1;
    if (Serial.available() > 1)
    {
        actuator = Serial.parseInt();
        char incoming = Serial.read();
        float value = Serial.parseFloat();
        switch (incoming)
        {
        case 'p':
            actuators[actuator].setPID_KOEF(pid_enum::Kp, value);
        case 'i':
            actuators[actuator].setPID_KOEF(pid_enum::Ki, value);
        case 'd':
            actuators[actuator].setPID_KOEF(pid_enum::Kd, value);
        case 's':
            actuators[actuator].setVelocity(value);
        }
        Serial.println(incoming);
        Serial.println(value);
    }
    if (actuator > -1 && actuator < ACTUATORS_COUNT)
    {
        String actuator_name = "";
        switch (actuator)
        {
        case LEFT_FRONT:
            actuator_name = "LEFT_FRONT";
        case RIGHT_FRONT:
            actuator_name = "RIGHT_FRONT";
        case LEFT_BACK:
            actuator_name = "LEFT_BACK";
        case RIGHT_BACK:
            actuator_name = "RIGHT_BACK";
        }
        Serial.println("Actuator " + actuator_name + " velocity");
        Serial.println(actuators[actuator].getVelocity());
    }
}
#endif // WORK_MODE__SERIAL