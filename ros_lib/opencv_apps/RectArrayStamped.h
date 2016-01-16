#ifndef _ROS_opencv_apps_RectArrayStamped_h
#define _ROS_opencv_apps_RectArrayStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "opencv_apps/Rect.h"

namespace opencv_apps
{

  class RectArrayStamped : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t rects_length;
      opencv_apps::Rect st_rects;
      opencv_apps::Rect * rects;

    RectArrayStamped():
      header(),
      rects_length(0), rects(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = rects_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < rects_length; i++){
      offset += this->rects[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint8_t rects_lengthT = *(inbuffer + offset++);
      if(rects_lengthT > rects_length)
        this->rects = (opencv_apps::Rect*)realloc(this->rects, rects_lengthT * sizeof(opencv_apps::Rect));
      offset += 3;
      rects_length = rects_lengthT;
      for( uint8_t i = 0; i < rects_length; i++){
      offset += this->st_rects.deserialize(inbuffer + offset);
        memcpy( &(this->rects[i]), &(this->st_rects), sizeof(opencv_apps::Rect));
      }
     return offset;
    }

    const char * getType(){ return "opencv_apps/RectArrayStamped"; };
    const char * getMD5(){ return "f29b08b33e061c73d7d9fc35142631d0"; };

  };

}
#endif