#ifndef _ROS_geographic_msgs_GeoPoint_h
#define _ROS_geographic_msgs_GeoPoint_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace geographic_msgs
{

  class GeoPoint : public ros::Msg
  {
    public:
      float latitude;
      float longitude;
      float altitude;

    GeoPoint():
      latitude(0),
      longitude(0),
      altitude(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->latitude);
      offset += serializeAvrFloat64(outbuffer + offset, this->longitude);
      offset += serializeAvrFloat64(outbuffer + offset, this->altitude);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->latitude));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->longitude));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->altitude));
     return offset;
    }

    const char * getType(){ return "geographic_msgs/GeoPoint"; };
    const char * getMD5(){ return "c48027a852aeff972be80478ff38e81a"; };

  };

}
#endif