#ifndef _ROS_opencv_apps_CircleArrayStamped_h
#define _ROS_opencv_apps_CircleArrayStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "opencv_apps/Circle.h"

namespace opencv_apps
{

  class CircleArrayStamped : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t circles_length;
      opencv_apps::Circle st_circles;
      opencv_apps::Circle * circles;

    CircleArrayStamped():
      header(),
      circles_length(0), circles(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
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
      offset += this->header.deserialize(inbuffer + offset);
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

    const char * getType(){ return "opencv_apps/CircleArrayStamped"; };
    const char * getMD5(){ return "430ffa6c2b0a36b7e81feff1ce79c3c4"; };

  };

}
#endif