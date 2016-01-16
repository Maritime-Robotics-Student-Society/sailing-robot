#ifndef _ROS_opencv_apps_RotatedRect_h
#define _ROS_opencv_apps_RotatedRect_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "opencv_apps/Point2D.h"
#include "opencv_apps/Size.h"

namespace opencv_apps
{

  class RotatedRect : public ros::Msg
  {
    public:
      float angle;
      opencv_apps::Point2D center;
      opencv_apps::Size size;

    RotatedRect():
      angle(0),
      center(),
      size()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->angle);
      offset += this->center.serialize(outbuffer + offset);
      offset += this->size.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->angle));
      offset += this->center.deserialize(inbuffer + offset);
      offset += this->size.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "opencv_apps/RotatedRect"; };
    const char * getMD5(){ return "0ae60505c52f020176686d0689b8d390"; };

  };

}
#endif