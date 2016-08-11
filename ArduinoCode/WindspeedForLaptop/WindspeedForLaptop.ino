/*
 * Serial print of measured wind speed
 *
 * publishing the apparent windspeed in [m/s]
 * based on rotation counts from the wind anemometer
 *
 */

#include <stdlib.h>

double apparent_wind_direction;

float wind_speed = 0;   // Initialise wind speed
int prevtime = 1;  

int PIN = 3;            // For the wind speed sensor, pin 2 or 3 only
int   i;                // count number (for loop)
int led = 13; //pin for the LED

int thread=100;
int sensorValue1; 


void arduino_anemometer()
{
 // measure wind speed
 int deltat = millis() - prevtime;
 if (deltat > 0){
   wind_speed = 666.66/deltat;
 }
 prevtime = millis();
 digitalWrite(led, !digitalRead(led));
}

void setup()
{
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(PIN) , arduino_anemometer, RISING);
  
  pinMode(led, OUTPUT);

}


void loop()
{
  // Publish previously measured wind speed
  Serial.println(wind_speed);
  Serial.println("in m/s");

  thread=100;
  
  delay(100);
}
