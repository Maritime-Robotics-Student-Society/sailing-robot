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

void setup()
{
  nh.initNode();
  nh.advertise(publishApparentWindSpeed);
}


void loop()
{
  /*
   * Wind calculations here
   */
  wind_speed = wind_speed + 1;
  apparent_wind_speed.data = wind_speed;
  publishApparentWindSpeed.publish( &apparent_wind_speed );
  nh.spinOnce();
  delay(100);
}
