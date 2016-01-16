#ifndef _ROS_geographic_msgs_BoundingBox_h
#define _ROS_geographic_msgs_BoundingBox_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geographic_msgs/GeoPoint.h"

namespace geographic_msgs
{

  class BoundingBox : public ros::Msg
  {
    public:
      geographic_msgs::GeoPoint min_pt;
      geographic_msgs::GeoPoint max_pt;

    BoundingBox():
      min_pt(),
      max_pt()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->min_pt.serialize(outbuffer + offset);
      offset += this->max_pt.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->min_pt.deserialize(inbuffer + offset);
      offset += this->max_pt.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return "geographic_msgs/BoundingBox"; };
    const char * getMD5(){ return "f62e8b5e390a3ac7603250d46e8f8446"; };

  };

}
#endif