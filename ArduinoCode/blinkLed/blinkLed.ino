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


#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
//#include <std_msgs/String.h>
#include <Adafruit_NeoPixel.h>
ros::NodeHandle  nh;

#define PIN 6
#define NUMPIXELS 30
#define DELAY 20

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUMPIXELS, PIN);
int gps_satellites = 0;



void set_up_to_pixel(int number, int red, int green, int blue){
  int i = 0;
  while(i<number){
    strip.setPixelColor(i, red, green, blue);
    i++;
  }
}


void set_all_msg_colour(long allColours){
  int red = (int)((float)allColours/90000);
  int green = (allColours - red*90000)/300;
  int blue = allColours - red*90000 - green*300;
  set_up_to_pixel(NUMPIXELS, red, green, blue);
}

//void blink_sailing_state(const std_msgs::String& cmd_msg){
//  const char * sailing_state = cmd_msg.data;
//    if (sailing_state[0] != 'n'){
      // BLINK WHEN SAILING STATE IS NOT 'normal'
      // for some reason sending more than 10 characters breaks this
      // maybe dynamic memory????
//      set_up_to_pixel(NUMPIXELS, 255,0,0);
//      strip.show();
//      delay(DELAY);
//      set_up_to_pixel(NUMPIXELS,0,0,0);
//      strip.show();
//    }
    //if (sailing_state == "tack_to_stbd_tack"){
    //  set_up_to_pixel(NUMPIXELS, 0,255,0);
    //  strip.show();
    //  delay(DELAY);
    //  set_up_to_pixel(NUMPIXELS,0,0,0);
    //  strip.show();
    //}
//}

void blink_colour(const std_msgs::Int32& cmd_msg){
  set_all_msg_colour(cmd_msg.data);
  strip.show();
  delay(DELAY);
  set_up_to_pixel(NUMPIXELS,0,0,0);
  strip.show();
}

void blink_gps_satellites(const std_msgs::Int16& cmd_msg){
  if(cmd_msg.data != gps_satellites){
    gps_satellites = cmd_msg.data;
    set_up_to_pixel(gps_satellites, 0,0,255);
    strip.show();
    delay(DELAY);
    set_up_to_pixel(gps_satellites, 0,0,0);
    strip.show();
  }
}

void blink_waypoint_reached(const std_msgs::Float32& cmd_msg){
  int representDistance = (int)(10*cmd_msg.data);
  if(representDistance<NUMPIXELS){
    set_up_to_pixel(representDistance,255,100,0);
    strip.show();
    delay(DELAY);
    set_up_to_pixel(representDistance, 0,0,0);
    strip.show();
  }
}

//ros::Subscriber<std_msgs::String> sub1("sailing_state", blink_sailing_state);
ros::Subscriber<std_msgs::Int16> sub2("gps_satellites", blink_gps_satellites);
ros::Subscriber<std_msgs::Float32> sub3("distance_to_waypoint", blink_waypoint_reached);
ros::Subscriber<std_msgs::Int32> sub4("led_blink", blink_colour);

void setup(){
  nh.getHardware()->setBaud(9600);;

  nh.initNode();
  //nh.subscribe(sub1);
  nh.subscribe(sub2);
  nh.subscribe(sub3);
  nh.subscribe(sub4);
  
  strip.setBrightness(30);  
  strip.begin();
  strip.show();

}

void loop(){
  nh.spinOnce();
}
