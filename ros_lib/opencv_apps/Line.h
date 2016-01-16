#ifndef _ROS_opencv_apps_Line_h
#define _ROS_opencv_apps_Line_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "opencv_apps/Point2D.h"

namespace opencv_apps
{

  class Line : public ros::Msg
  {
    public:
      opencv_apps::Point2D pt1;
      opencv_apps::Point2D pt2;

    Line():
      pt1(),
      pt2()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->pt1.serialize(outbuffer + offset);
      offset += this->pt2.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->pt1.deserialize(inbuffer + offset);
      offset += this->pt2.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "opencv_apps/Line"; };
    const char * getMD5(){ return "a1419010b3fc4549e3f450018363d000"; };

  };

}
#endif