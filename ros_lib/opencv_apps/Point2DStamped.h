#ifndef _ROS_opencv_apps_Point2DStamped_h
#define _ROS_opencv_apps_Point2DStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "opencv_apps/Point2D.h"

namespace opencv_apps
{

  class Point2DStamped : public ros::Msg
  {
    public:
      std_msgs::Header header;
      opencv_apps::Point2D point;

    Point2DStamped():
      header(),
      point()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->point.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->point.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "opencv_apps/Point2DStamped"; };
    const char * getMD5(){ return "9f7db918fde9989a73131d0d083d049d"; };

  };

}
#endif