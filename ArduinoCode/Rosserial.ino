/*
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
#include <math.h>
ros::NodeHandle  nh;


Servo servo;
void servo_cb( const std_msgs::UInt16& cmd_msg){
  float pwm;
  pwm = 400*((float)cmd_msg.data/630) + 1500;
  servo.writeMicroseconds(pwm); //set servo angle, should be from 0-180  
  //digitalWrite(13, HIGH-digitalRead(13));  //toggle led 
  //str_msg.data = pwm;
}


ros::Subscriber<std_msgs::UInt16> sub("rudder_control", servo_cb);

void setup(){
  pinMode(13, OUTPUT);

  nh.initNode();
  nh.subscribe(sub);
  
  servo.attach(9); //attach it to pin 9
  
}

void loop(){
  nh.spinOnce();
  delay(1);
}
