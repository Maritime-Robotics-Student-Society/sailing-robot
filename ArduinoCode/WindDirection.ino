/*
  AnalogReadSerial
  Reads an analog input on pin 0, prints the result to the serial monitor.
  Graphical representation is available using serial plotter (Tools > Serial Plotter menu)
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.
*/
#include<ros.h>
#include<std_msgs/UInt16.h>
#include <stdlib.h>

ros::NodeHandle nh;
// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

std_msgs::UInt16 wind_angle;
ros::Publisher p("wind_direction_apparent", &wind_angle);

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  nh.initNode();
  int i; // count number
  int sensorValue1 = analogRead(A0);
  int Reference[8] = {516,707,818,915,965,994,1009,1016};
  int Direction[8] = {135, 90, 45,180,  0,225, 270, 315};
  int position=-1,thread=100;

  // print out the value you read:
  for(i=0;i<=7;i++){
        if(abs(Reference[i]-sensorValue1)<thread){
            thread=abs(Reference[i]-sensorValue1);
            position=i;
        }
  wind_angle.data = Direction[position];
  p.publish(&wind_angle);
  delay(50);        // delay in between reads for stability
  nh.spinOnce();
}
