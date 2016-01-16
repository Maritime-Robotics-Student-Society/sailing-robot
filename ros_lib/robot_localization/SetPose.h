#ifndef _ROS_SERVICE_SetPose_h
#define _ROS_SERVICE_SetPose_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

namespace robot_localization
{

static const char SETPOSE[] = "robot_localization/SetPose";

  class SetPoseRequest : public ros::Msg
  {
    public:
      geometry_msgs::PoseWithCovarianceStamped pose;

    SetPoseRequest():
      pose()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->pose.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->pose.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return SETPOSE; };
    const char * getMD5(){ return "4f3e0bbe7a24e1f929488cd1970222d3"; };

  };

  class SetPoseResponse : public ros::Msg
  {
    public:

    SetPoseResponse()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
     return offset;
    }

    const char * getType(){ return SETPOSE; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SetPose {
    public:
    typedef SetPoseRequest Request;
    typedef SetPoseResponse Response;
  };

}
#endif
