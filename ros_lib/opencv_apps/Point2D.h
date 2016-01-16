#ifndef _ROS_opencv_apps_Point2D_h
#define _ROS_opencv_apps_Point2D_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace opencv_apps
{

  class Point2D : public ros::Msg
  {
    public:
      float x;
      float y;

    Point2D():
      x(0),
      y(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->x);
      offset += serializeAvrFloat64(outbuffer + offset, this->y);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->y));
     return offset;
    }

    const char * getType(){ return "opencv_apps/Point2D"; };
    const char * getMD5(){ return "209f516d3eb691f0663e25cb750d67c1"; };

  };

}
#endif