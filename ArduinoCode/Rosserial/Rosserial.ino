/*
 * Licence: BSD
 * From: http://wiki.ros.org/rosserial_arduino
 *       http://wiki.ros.org/rosserial_arduino/Tutorials/Servo%20Controller  
 *
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <Servo.h> 
#include <ros.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Int16.h>
#include <math.h>
ros::NodeHandle  nh;


Servo servo;
Servo rudderservo;
void servo_cb( const std_msgs::UInt16& cmd_msg){
  float pwm;
  pwm = 400*((float)cmd_msg.data/630) + 1500;
  servo.writeMicroseconds(pwm); //set servo angle, should be from 0-180  
  //digitalWrite(13, HIGH-digitalRead(13));  //toggle led 
  //str_msg.data = pwm;
}

void rudder_servo(const std_msgs::Int16& cmd_msg){
 rudderservo.write(cmd_msg.data + 90);
}


ros::Subscriber<std_msgs::UInt16> sub1("sail_servo", servo_cb);
ros::Subscriber<std_msgs::Int16> sub2("rudder_control",rudder_servo);

void setup(){
  nh.getHardware()->setBaud(9600);
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  
  servo.attach(9);  // sail servo attached to pin 9
  rudderservo.attach(10); // rudder servo attached to pin 10
}

void loop(){
  nh.spinOnce();
  delay(1);
}
