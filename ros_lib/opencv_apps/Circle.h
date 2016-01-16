#ifndef _ROS_opencv_apps_Circle_h
#define _ROS_opencv_apps_Circle_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "opencv_apps/Point2D.h"

namespace opencv_apps
{

  class Circle : public ros::Msg
  {
    public:
      opencv_apps::Point2D center;
      float radius;

    Circle():
      center(),
      radius(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->center.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->radius);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->center.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->radius));
     return offset;
    }

    const char * getType(){ return "opencv_apps/Circle"; };
    const char * getMD5(){ return "4f6847051b4fe493b5af8caad66201d5"; };

  };

}
#endif