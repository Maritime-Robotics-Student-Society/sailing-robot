/*
 * rosserial publisher for wind speed sensor
 *
 * publishing the apparent windspeed in [m/s]
 * based on rotation counts from the wind anemometer
 */

#include <ros.h>
#include <std_msgs/Float64.h>

ros::NodeHandle nh;

std_msgs::Float64 apparent_wind_speed;
ros::Publisher publishApparentWindSpeed("/apparent_wind_speed", &apparent_wind_speed);

float wind_speed = 0;

unsigned int duration;

unsigned long timePrevious;

int PIN = 2;

int count = 0;

void arduino_anemometer()
{
 count++;
}

void setup()
{
  nh.initNode();
  nh.advertise(publishApparentWindSpeed);
  attachInterrupt(digitalPinToInterrupt(PIN) , arduino_anemometer, RISING);

  count = 0;
  duration = 0;
  timePrevious = 0; 

}


void loop()
{
  /*
   * Wind calculations here
   */
 
  duration = (millis() - timePrevious);
  timePrevious = millis();
  count = 0;
  wind_speed = (666.66/duration);

  apparent_wind_speed.data = wind_speed;
  publishApparentWindSpeed.publish( &apparent_wind_speed );
  nh.spinOnce();
  delay(100);
}
