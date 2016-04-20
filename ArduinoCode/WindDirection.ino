/*
  AnalogReadSerial
  Reads an analog input on pin 0, prints the result to the serial monitor.
  Graphical representation is available using serial plotter (Tools > Serial Plotter menu)
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.
*/
#include<ros.h>
#include<std_msgs/UInt16.h>

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
  int angle;
  int sensorValue1 = analogRead(A0);
  // print out the value you read:
  switch(sensorValue1)
  { case 965:
       angle = 0;
       break;
    case 818:
       angle = 45;
       break;
    case 707:
       angle = 90;
       break;
    case 516:
       angle = 135;
       break;
    case 915:
       angle = 180;
       break;
    case 994:
       angle = 225;
       break;
    case 1009:
       angle = 270;
       break;
    case 1016:
       angle = 315;
       break;
  }
  wind_angle.data = angle;
  p.publish(&wind_angle);
  delay(50);        // delay in between reads for stability
  nh.spinOnce();
}
