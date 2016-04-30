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


float wind_speed = 0;  // Initialise wind speed
int timestep = 100;   // timestep in [ms] 

int PIN = 2;
int count = 0;

void arduino_anemometer()
{
 count++;
}

void setup()
{
  nh.getHardware()->setBaud(9600);
  nh.initNode();
  nh.advertise(publishApparentWindSpeed);
  attachInterrupt(PIN , arduino_anemometer, RISING);

}


void loop()
{
  // Publish previously measured wind speed
  apparent_wind_speed.data = wind_speed;
  publishApparentWindSpeed.publish( &apparent_wind_speed );
  nh.spinOnce();

  // measure wind speed over the next time step
  count = 0; 
  delay(timestep);
  wind_speed = (666.66/timestep) * count;
}
