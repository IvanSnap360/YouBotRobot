#include "config.h"

#ifdef ROS_MODE
#include <ros.h>
#include <sensor_msgs/JointState.h>
#endif

#include "actuator.h"

#ifdef ROS_MODE
ros::NodeHandle nh;
#endif

unsigned long led_blink_lt = 0;
bool led_state = false;

ACTUATOR actuator[WHEELS_COUNT];

#ifdef ROS_MODE
char *joints_names[WHEELS_COUNT] = {
    "left_front_omniwheel_joint",
    "right_front_omniwheel_joint",
    "left_back_omniwheel_joint",
    "right_back_omniwheel_joint",
};

double joints_pos[WHEELS_COUNT] = {0.0, 0.0, 0.0, 0.0};
double joints_vel[WHEELS_COUNT] = {0.0, 0.0, 0.0, 0.0};
double joints_eff[WHEELS_COUNT] = {0.0, 0.0, 0.0, 0.0};


void joint_cntrl_sub_cb_f(const sensor_msgs::JointState &js);


ros::Subscriber<sensor_msgs::JointState> joint_cntrl_sub(WHEELS_JOINTS_CONTROL_TOPIC_NAME, joint_cntrl_sub_cb_f);

sensor_msgs::JointState joints_state_msg;
ros::Publisher joint_state_pub(WHEELS_JOINTS_STATE_TOPIC_NAME, &joints_state_msg);
unsigned long joints_state_pub_lt = 0;
#endif

void setup()
{
    pinMode(LED, OUTPUT);

#ifndef ROS_MODE
    Serial.begin(SERIAL_BAUDRATE);
#endif

    for (int i = 0; i < WHEELS_COUNT; i++)
    {
        actuator[i].setConfig(&actuators_config[i]);

    }

#ifndef ROS_MODE
    while (!Serial)
    {
        if (millis() - led_blink_lt > (uint32_t)(1000.0 / SERIAL_DISCONNECT_LED_BLINK_HZ))
        {
            led_state = !led_state;
            digitalWrite(LED, led_state);
            led_blink_lt = millis();
        }
    }

#endif
}

void loop()
{
#ifdef ROS_MODE
    if (nh.connected())
    {

        if (millis() - led_blink_lt > (uint32_t)(1000.0 / ROS_CONNECT_LED_BLINK_HZ))
        {
            led_state = !led_state;
            digitalWrite(LED, led_state);
            led_blink_lt = millis();
        }

        if (millis() - joints_state_pub_lt > (uint32_t)(1000.0 / JOINT_STATE_PUB_RATE_HZ))
        {
            joints_vel[LEFT_FRONT]  = actuator[LEFT_FRONT].getVelocity();
            joints_vel[RIGHT_FRONT] = actuator[RIGHT_FRONT].getVelocity();
            joints_vel[LEFT_BACK]   = actuator[LEFT_BACK].getVelocity();
            joints_vel[RIGHT_BACK]  = actuator[RIGHT_BACK].getVelocity();

            joints_state_pub_lt = millis();
        }
    }
    else
    {
        actuator[LEFT_FRONT].setVelocity(0.0);
        actuator[RIGHT_FRONT].setVelocity(0.0);
        actuator[LEFT_BACK].setVelocity(0.0);
        actuator[RIGHT_BACK].setVelocity(0.0);
        if (millis() - led_blink_lt > (uint32_t)(1000.0 / ROS_DISCONNECT_LED_BLINK_HZ))
        {
            led_state = !led_state;
            digitalWrite(LED, led_state);
            led_blink_lt = millis();
        }
    }
    nh.spinOnce();
#endif

#if (!defined(ROS_MODE) && defined(ENCODER_DEBUG) && defined(ENCODER_RAW_VALS) && !defined(ENCODER_VELOCITY_VALS))
    Serial.print("LEFT FRONT WHEEL ENCODER VAL: ");
    Serial.print(actuator[LEFT_FRONT].getRawEncoderVal());
    Serial.print("  ;RIGHT FRONT WHEEL ENCODER VAL: ");
    Serial.print(actuator[RIGHT_FRONT].getRawEncoderVal());
    Serial.print("  ;LEFT BACK WHEEL ENCODER VAL: ");
    Serial.print(actuator[LEFT_BACK].getRawEncoderVal());
    Serial.print("  ;RIGHT BACk WHEEL ENCODER VAL: ");
    Serial.println(actuator[RIGHT_BACK].getRawEncoderVal());
#endif

#if (!defined(ROS_MODE) && defined(ENCODER_DEBUG) && defined(ENCODER_VELOCITY_VALS) && !defined(ENCODER_RAW_VALS))
    Serial.print("LEFT FRONT WHEEL ENCODER VELOCITY: ");
    Serial.print(actuator[LEFT_FRONT].getVelocity());
    Serial.print("  ;RIGHT FRONT WHEEL ENCODER VELOCITY: ");
    Serial.print(actuator[RIGHT_FRONT].getVelocity());
    Serial.print("  ;LEFT BACK WHEEL ENCODER VELOCITY: ");
    Serial.print(actuator[LEFT_BACK].getVelocity());
    Serial.print("  ;RIGHT BACk WHEEL ENCODER VELOCITY: ");
    Serial.println(actuator[RIGHT_BACK].getVelocity());
#endif

#ifndef ROS_MODE
    if (millis() - led_blink_lt > (uint32_t)(1000.0 / SERIAL_CONNECT_LED_BLINK_HZ))
    {
        led_state = !led_state;
        digitalWrite(LED, led_state);
        led_blink_lt = millis();
    }
#endif

    actuator[LEFT_FRONT].tick();
    actuator[RIGHT_FRONT].tick();
    actuator[LEFT_BACK].tick();
    actuator[RIGHT_BACK].tick();
}

#ifdef ROS_MODE
void joint_cntrl_sub_cb_f(const sensor_msgs::JointState &js)
{
    actuator[LEFT_FRONT].setVelocity(js.velocity[LEFT_FRONT]);
    actuator[RIGHT_FRONT].setVelocity(js.velocity[RIGHT_FRONT]);
    actuator[LEFT_BACK].setVelocity(js.velocity[LEFT_BACK]);
    actuator[RIGHT_BACK].setVelocity(js.velocity[RIGHT_BACK]);
}

#endif