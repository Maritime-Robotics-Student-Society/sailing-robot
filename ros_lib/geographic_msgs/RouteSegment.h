#ifndef _ROS_geographic_msgs_RouteSegment_h
#define _ROS_geographic_msgs_RouteSegment_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "uuid_msgs/UniqueID.h"
#include "geographic_msgs/KeyValue.h"

namespace geographic_msgs
{

  class RouteSegment : public ros::Msg
  {
    public:
      uuid_msgs::UniqueID id;
      uuid_msgs::UniqueID start;
      uuid_msgs::UniqueID end;
      uint8_t props_length;
      geographic_msgs::KeyValue st_props;
      geographic_msgs::KeyValue * props;

    RouteSegment():
      id(),
      start(),
      end(),
      props_length(0), props(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->id.serialize(outbuffer + offset);
      offset += this->start.serialize(outbuffer + offset);
      offset += this->end.serialize(outbuffer + offset);
      *(outbuffer + offset++) = props_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < props_length; i++){
      offset += this->props[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->id.deserialize(inbuffer + offset);
      offset += this->start.deserialize(inbuffer + offset);
      offset += this->end.deserialize(inbuffer + offset);
      uint8_t props_lengthT = *(inbuffer + offset++);
      if(props_lengthT > props_length)
        this->props = (geographic_msgs::KeyValue*)realloc(this->props, props_lengthT * sizeof(geographic_msgs::KeyValue));
      offset += 3;
      props_length = props_lengthT;
      for( uint8_t i = 0; i < props_length; i++){
      offset += this->st_props.deserialize(inbuffer + offset);
        memcpy( &(this->props[i]), &(this->st_props), sizeof(geographic_msgs::KeyValue));
      }
     return offset;
    }

    const char * getType(){ return "geographic_msgs/RouteSegment"; };
    const char * getMD5(){ return "8583d1e2ddf1891c3934a5d2ed9a799c"; };

  };

}
#endif