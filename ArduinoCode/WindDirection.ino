/*
  AnalogReadSerial
  Reads an analog input on pin 0, prints the result to the serial monitor.
  Graphical representation is available using serial plotter (Tools > Serial Plotter menu)
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.
  
  This example code is in the public domain.
 * 
 * Debug commands:
 * #roscore
 * #rosrun rosserial_python serial_node.py /dev/ttyACM0
 * #rostopic echo chatter
 */

#include <ros.h>
#include <stdlib.h>
#include<std_msgs/Float32.h>

ros::NodeHandle  nh;
std_msgs::Float32 wind_direction_angle;
ros::Publisher chatter("wind_direction_apparent", &wind_direction_angle);


void setup()
{
  Serial.begin(9600);
  nh.initNode();  
  nh.advertise(chatter);
//  nh.getHardware()->setBaud(57600);
}

void loop()
{
  int   i; // count number
  int   sensorValue1 =analogRead(A0);  // Pin A0
  int   Reference[16] = {  422,   487, 516,  603, 707,  780, 818,   891, 915,   958, 965,   980, 994,   1001, 1009, 1016};
  float Direction[16] = {112.5, 157.5, 135, 67.5,  90, 22.5,  45, 202.5, 180, 337.5,   0, 247.5, 225,  292.5,  270,  315};
  int   position=-1,thread=100;
  
  // Update angle
  for(i=0;i<=15;i++){
        if(abs(Reference[i]-sensorValue1)<thread){
            thread=abs(Reference[i]-sensorValue1);
            position=i;
        }
  }
  
  wind_direction_angle.data = Direction[position];
  chatter.publish( &wind_direction_angle );
  nh.spinOnce();
  delay(100);
}
