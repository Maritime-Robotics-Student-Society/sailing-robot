#ifndef _ROS_SERVICE_GetPointMapROI_h
#define _ROS_SERVICE_GetPointMapROI_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "sensor_msgs/PointCloud2.h"

namespace map_msgs
{

static const char GETPOINTMAPROI[] = "map_msgs/GetPointMapROI";

  class GetPointMapROIRequest : public ros::Msg
  {
    public:
      float x;
      float y;
      float z;
      float r;
      float l_x;
      float l_y;
      float l_z;

    GetPointMapROIRequest():
      x(0),
      y(0),
      z(0),
      r(0),
      l_x(0),
      l_y(0),
      l_z(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->x);
      offset += serializeAvrFloat64(outbuffer + offset, this->y);
      offset += serializeAvrFloat64(outbuffer + offset, this->z);
      offset += serializeAvrFloat64(outbuffer + offset, this->r);
      offset += serializeAvrFloat64(outbuffer + offset, this->l_x);
      offset += serializeAvrFloat64(outbuffer + offset, this->l_y);
      offset += serializeAvrFloat64(outbuffer + offset, this->l_z);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->y));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->z));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->r));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->l_x));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->l_y));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->l_z));
     return offset;
    }

    const char * getType(){ return GETPOINTMAPROI; };
    const char * getMD5(){ return "895f7e437a9a6dd225316872b187a303"; };

  };

  class GetPointMapROIResponse : public ros::Msg
  {
    public:
      sensor_msgs::PointCloud2 sub_map;

    GetPointMapROIResponse():
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

    const char * getType(){ return GETPOINTMAPROI; };
    const char * getMD5(){ return "313769f8b0e724525c6463336cbccd63"; };

  };

  class GetPointMapROI {
    public:
    typedef GetPointMapROIRequest Request;
    typedef GetPointMapROIResponse Response;
  };

}
#endif
