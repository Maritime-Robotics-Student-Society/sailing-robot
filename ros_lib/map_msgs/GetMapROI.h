#ifndef _ROS_SERVICE_GetMapROI_h
#define _ROS_SERVICE_GetMapROI_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "nav_msgs/OccupancyGrid.h"

namespace map_msgs
{

static const char GETMAPROI[] = "map_msgs/GetMapROI";

  class GetMapROIRequest : public ros::Msg
  {
    public:
      float x;
      float y;
      float l_x;
      float l_y;

    GetMapROIRequest():
      x(0),
      y(0),
      l_x(0),
      l_y(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->x);
      offset += serializeAvrFloat64(outbuffer + offset, this->y);
      offset += serializeAvrFloat64(outbuffer + offset, this->l_x);
      offset += serializeAvrFloat64(outbuffer + offset, this->l_y);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->y));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->l_x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->l_y));
     return offset;
    }

    const char * getType(){ return GETMAPROI; };
    const char * getMD5(){ return "43c2ff8f45af555c0eaf070c401e9a47"; };

  };

  class GetMapROIResponse : public ros::Msg
  {
    public:
      nav_msgs::OccupancyGrid sub_map;

    GetMapROIResponse():
      sub_map()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->sub_map.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->sub_map.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return GETMAPROI; };
    const char * getMD5(){ return "4d1986519c00d81967d2891a606b234c"; };

  };

  class GetMapROI {
    public:
    typedef GetMapROIRequest Request;
    typedef GetMapROIResponse Response;
  };

}
#endif
