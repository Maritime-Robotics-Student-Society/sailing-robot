#ifndef _ROS_SERVICE_SetDatum_h
#define _ROS_SERVICE_SetDatum_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geographic_msgs/GeoPose.h"

namespace robot_localization
{

static const char SETDATUM[] = "robot_localization/SetDatum";

  class SetDatumRequest : public ros::Msg
  {
    public:
      geographic_msgs::GeoPose geo_pose;

    SetDatumRequest():
      geo_pose()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->geo_pose.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->geo_pose.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return SETDATUM; };
    const char * getMD5(){ return "fe903ca95d0210defda73a1629604439"; };

  };

  class SetDatumResponse : public ros::Msg
  {
    public:

    SetDatumResponse()
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

    const char * getType(){ return SETDATUM; };
    const char * getMD5(){ return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SetDatum {
    public:
    typedef SetDatumRequest Request;
    typedef SetDatumResponse Response;
  };

}
#endif
