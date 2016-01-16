#ifndef _ROS_opencv_apps_CircleArray_h
#define _ROS_opencv_apps_CircleArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "opencv_apps/Circle.h"

namespace opencv_apps
{

  class CircleArray : public ros::Msg
  {
    public:
      uint8_t circles_length;
      opencv_apps::Circle st_circles;
      opencv_apps::Circle * circles;

    CircleArray():
      circles_length(0), circles(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = circles_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < circles_length; i++){
      offset += this->circles[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t circles_lengthT = *(inbuffer + offset++);
      if(circles_lengthT > circles_length)
        this->circles = (opencv_apps::Circle*)realloc(this->circles, circles_lengthT * sizeof(opencv_apps::Circle));
      offset += 3;
      circles_length = circles_lengthT;
      for( uint8_t i = 0; i < circles_length; i++){
      offset += this->st_circles.deserialize(inbuffer + offset);
        memcpy( &(this->circles[i]), &(this->st_circles), sizeof(opencv_apps::Circle));
      }
     return offset;
    }

    const char * getType(){ return "opencv_apps/CircleArray"; };
    const char * getMD5(){ return "1970b146e338dd024c765e522039a727"; };

  };

}
#endif