#ifndef _ROS_opencv_apps_FlowStamped_h
#define _ROS_opencv_apps_FlowStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "opencv_apps/Flow.h"

namespace opencv_apps
{

  class FlowStamped : public ros::Msg
  {
    public:
      std_msgs::Header header;
      opencv_apps::Flow flow;

    FlowStamped():
      header(),
      flow()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->flow.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->flow.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "opencv_apps/FlowStamped"; };
    const char * getMD5(){ return "b55faf909449963372b92417925b68cc"; };

  };

}
#endif