#ifndef _ROS_geographic_msgs_RoutePath_h
#define _ROS_geographic_msgs_RoutePath_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "uuid_msgs/UniqueID.h"
#include "geographic_msgs/KeyValue.h"

namespace geographic_msgs
{

  class RoutePath : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uuid_msgs::UniqueID network;
      uint8_t segments_length;
      uuid_msgs::UniqueID st_segments;
      uuid_msgs::UniqueID * segments;
      uint8_t props_length;
      geographic_msgs::KeyValue st_props;
      geographic_msgs::KeyValue * props;

    RoutePath():
      header(),
      network(),
      segments_length(0), segments(NULL),
      props_length(0), props(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->network.serialize(outbuffer + offset);
      *(outbuffer + offset++) = segments_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < segments_length; i++){
      offset += this->segments[i].serialize(outbuffer + offset);
      }
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
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->network.deserialize(inbuffer + offset);
      uint8_t segments_lengthT = *(inbuffer + offset++);
      if(segments_lengthT > segments_length)
        this->segments = (uuid_msgs::UniqueID*)realloc(this->segments, segments_lengthT * sizeof(uuid_msgs::UniqueID));
      offset += 3;
      segments_length = segments_lengthT;
      for( uint8_t i = 0; i < segments_length; i++){
      offset += this->st_segments.deserialize(inbuffer + offset);
        memcpy( &(this->segments[i]), &(this->st_segments), sizeof(uuid_msgs::UniqueID));
      }
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

    const char * getType(){ return "geographic_msgs/RoutePath"; };
    const char * getMD5(){ return "0aa2dd809a8091bdb4466dfefecbb8cf"; };

  };

}
#endif