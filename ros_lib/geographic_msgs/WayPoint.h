#ifndef _ROS_geographic_msgs_WayPoint_h
#define _ROS_geographic_msgs_WayPoint_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "uuid_msgs/UniqueID.h"
#include "geographic_msgs/GeoPoint.h"
#include "geographic_msgs/KeyValue.h"

namespace geographic_msgs
{

  class WayPoint : public ros::Msg
  {
    public:
      uuid_msgs::UniqueID id;
      geographic_msgs::GeoPoint position;
      uint8_t props_length;
      geographic_msgs::KeyValue st_props;
      geographic_msgs::KeyValue * props;

    WayPoint():
      id(),
      position(),
      props_length(0), props(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->id.serialize(outbuffer + offset);
      offset += this->position.serialize(outbuffer + offset);
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
      offset += this->position.deserialize(inbuffer + offset);
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

    const char * getType(){ return "geographic_msgs/WayPoint"; };
    const char * getMD5(){ return "ef04f823aef332455a49eaec3f1761b7"; };

  };

}
#endif