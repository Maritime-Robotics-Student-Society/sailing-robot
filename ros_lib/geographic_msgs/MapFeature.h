#ifndef _ROS_geographic_msgs_MapFeature_h
#define _ROS_geographic_msgs_MapFeature_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "uuid_msgs/UniqueID.h"
#include "geographic_msgs/KeyValue.h"

namespace geographic_msgs
{

  class MapFeature : public ros::Msg
  {
    public:
      uuid_msgs::UniqueID id;
      uint8_t components_length;
      uuid_msgs::UniqueID st_components;
      uuid_msgs::UniqueID * components;
      uint8_t props_length;
      geographic_msgs::KeyValue st_props;
      geographic_msgs::KeyValue * props;

    MapFeature():
      id(),
      components_length(0), components(NULL),
      props_length(0), props(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->id.serialize(outbuffer + offset);
      *(outbuffer + offset++) = components_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < components_length; i++){
      offset += this->components[i].serialize(outbuffer + offset);
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
      offset += this->id.deserialize(inbuffer + offset);
      uint8_t components_lengthT = *(inbuffer + offset++);
      if(components_lengthT > components_length)
        this->components = (uuid_msgs::UniqueID*)realloc(this->components, components_lengthT * sizeof(uuid_msgs::UniqueID));
      offset += 3;
      components_length = components_lengthT;
      for( uint8_t i = 0; i < components_length; i++){
      offset += this->st_components.deserialize(inbuffer + offset);
        memcpy( &(this->components[i]), &(this->st_components), sizeof(uuid_msgs::UniqueID));
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

    const char * getType(){ return "geographic_msgs/MapFeature"; };
    const char * getMD5(){ return "e2505ace5e8da8a15b610eaf62bdefae"; };

  };

}
#endif