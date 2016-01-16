#ifndef _ROS_opencv_apps_Rect_h
#define _ROS_opencv_apps_Rect_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace opencv_apps
{

  class Rect : public ros::Msg
  {
    public:
      float x;
      float y;
      float width;
      float height;

    Rect():
      x(0),
      y(0),
      width(0),
      height(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->x);
      offset += serializeAvrFloat64(outbuffer + offset, this->y);
      offset += serializeAvrFloat64(outbuffer + offset, this->width);
      offset += serializeAvrFloat64(outbuffer + offset, this->height);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->y));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->width));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->height));
     return offset;
    }

    const char * getType(){ return "opencv_apps/Rect"; };
    const char * getMD5(){ return "7048f28f1f0ef51e102638c86d9a7728"; };

  };

}
#endif