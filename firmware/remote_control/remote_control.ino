#include "joystick.h"
#include <ros.h>
#include <geometry_msgs/Pose.h>
// #define DEBUG
#define ROS
// ros::NodeHandle_ <ArduinoHardware,5,5,512,512> nh;
#ifdef ROS
ros::NodeHandle nh;
#endif // DEBUG

#define LJ_X_PIN A1
#define LJ_Y_PIN A0
#define LJ_B_PIN 7

#define RJ_X_PIN A2
#define RJ_Y_PIN A3
#define RJ_B_PIN 2

#define CONNECT_LED_PIN 6
#define UNCONNECT_LED_PIN 5
#define LED_VAL 127

JOYSTICK left_joy;
JOYSTICK right_joy;

geometry_msgs::Pose left_joy_msg;
geometry_msgs::Pose right_joy_msg;

ros::Publisher left_joy_pub("/remote_cotroller/left_joy/value", &left_joy_msg);
ros::Publisher right_joy_pub("/remote_cotroller/right_joy/value", &right_joy_msg);

unsigned long long last_pub_time;

void setup()
{
#ifdef DEBUG
  Serial.begin(9600);
#endif // DEBUG
#ifdef ROS
  nh.getHardware()->setBaud(115200);
  nh.initNode();
  nh.advertise(left_joy_pub);
  nh.advertise(right_joy_pub);

#endif // DEBUG

  pinMode(CONNECT_LED_PIN, OUTPUT);
  pinMode(UNCONNECT_LED_PIN, OUTPUT);

  digitalWrite(CONNECT_LED_PIN, 0);
  digitalWrite(UNCONNECT_LED_PIN, 0);

  left_joy.init(LJ_X_PIN, LJ_Y_PIN, LJ_B_PIN, -1000, 1000);
  right_joy.init(RJ_X_PIN, RJ_Y_PIN, RJ_B_PIN, -1000, 1000);
}

void loop()
{
  #ifdef ROS
  
  if (nh.connected())
  {
    if (millis() - last_pub_time > 50)
    {
      left_joy_msg.position.x = float(-left_joy.getYMapValue());
      left_joy_msg.position.y = float(-left_joy.getXMapValue());
      left_joy_msg.position.z = float(left_joy.getButValue());

      right_joy_msg.position.x = float(-right_joy.getXMapValue());
      right_joy_msg.position.y = float(-right_joy.getYMapValue());
      right_joy_msg.position.z = right_joy.getButValue();

      left_joy_pub.publish(&left_joy_msg);
      right_joy_pub.publish(&right_joy_msg);
      last_pub_time = millis();
    }
    analogWrite(CONNECT_LED_PIN, LED_VAL);
    analogWrite(UNCONNECT_LED_PIN, 0);
  }
  else
  {
    analogWrite(UNCONNECT_LED_PIN, LED_VAL);
    analogWrite(CONNECT_LED_PIN, 0);
  }
  nh.spinOnce();
  #endif // ROS

  #ifdef DEBUG
  Serial.print(left_joy.getXMapValue());
  Serial.print("  ");
  Serial.print(left_joy.getYMapValue());
  Serial.print("  ");
  Serial.println();
  #endif // DEBUG
}
