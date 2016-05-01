/*
 * rosserial publisher for wind sensors
 *
 * publishing the apparent windspeed in [m/s]
 * based on rotation counts from the wind anemometer
 *
 * publishing the apparent wind direction in [degrees]
 * based on the resistance measured with the wind vane
 */

#include <ros.h>
#include <stdlib.h>
#include <std_msgs/Float64.h>

ros::NodeHandle nh;

std_msgs::Float64 apparent_wind_speed;
std_msgs::Float64 apparent_wind_direction;
ros::Publisher publishApparentWindSpeed("/wind_speed_apparent", &apparent_wind_speed);
ros::Publisher publishApparentWindAngle("/wind_direction_apparent", &apparent_wind_direction);

float wind_speed = 0;   // Initialise wind speed
int prevtime = 1;  

int PIN = 3;            // For the wind speed sensor, pin 2 or 3 only

int   i;                // count number (for loop)

int led = 13; //pin for the LED

// read raw sensor data from the analog PINs
// Initialisation for the wind directions sensor
int   Reference[16] = {786, 406, 461, 84,   92,  66,    184, 127 ,   287, 244,   631,  600,   946, 827,  979, 702};
float Direction[16] = {0,  22.5, 45,  67.5, 90,  112.5, 135, 157.5, 180, 202.5, 225,  247.5, 270, 292.5, 315, 337.5};
int   position=-1,thread=100;
int   sensorValue1; 


void arduino_anemometer()
{
 // measure wind speed
 wind_speed = 666.66/(millis() - prevtime);
 prevtime = millis();

 digitalWrite(led, !digitalRead(led));
}

void setup()
{
  nh.getHardware()->setBaud(9600);
  nh.initNode();
  nh.advertise(publishApparentWindSpeed);
  nh.advertise(publishApparentWindAngle);
  attachInterrupt(digitalPinToInterrupt(PIN) , arduino_anemometer, RISING);
  
  pinMode(led, OUTPUT);
}


void loop()
{
  // Publish previously measured wind speed
  apparent_wind_speed.data = wind_speed;
  publishApparentWindSpeed.publish( &apparent_wind_speed );
  nh.spinOnce();

 
  thread=100;
  sensorValue1 = analogRead(A0);  // Pin A0 for the wind direction
  // Update apparent wind angle
  for(i=0;i<=15;i++){
        if(abs(Reference[i]-sensorValue1)<thread){
            thread=abs(Reference[i]-sensorValue1);
            position=i;
        }
  }
  
  apparent_wind_direction.data =  Direction[position]; //5*sensorValue1/1024.0;//
  publishApparentWindAngle.publish(&apparent_wind_direction);

  delay(100);
}
