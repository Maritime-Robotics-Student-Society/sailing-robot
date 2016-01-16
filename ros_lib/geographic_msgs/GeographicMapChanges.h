#ifndef _ROS_geographic_msgs_GeographicMapChanges_h
#define _ROS_geographic_msgs_GeographicMapChanges_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "geographic_msgs/GeographicMap.h"
#include "uuid_msgs/UniqueID.h"

namespace geographic_msgs
{

  class GeographicMapChanges : public ros::Msg
  {
    public:
      std_msgs::Header header;
      geographic_msgs::GeographicMap diffs;
      uint8_t deletes_length;
      uuid_msgs::UniqueID st_deletes;
      uuid_msgs::UniqueID * deletes;

    GeographicMapChanges():
      header(),
      diffs(),
      deletes_length(0), deletes(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->diffs.serialize(outbuffer + offset);
      *(outbuffer + offset++) = deletes_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < deletes_length; i++){
      offset += this->deletes[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->diffs.deserialize(inbuffer + offset);
      uint8_t deletes_lengthT = *(inbuffer + offset++);
      if(deletes_lengthT > deletes_length)
        this->deletes = (uuid_msgs::UniqueID*)realloc(this->deletes, deletes_lengthT * sizeof(uuid_msgs::UniqueID));
      offset += 3;
      deletes_length = deletes_lengthT;
      for( uint8_t i = 0; i < deletes_length; i++){
      offset += this->st_deletes.deserialize(inbuffer + offset);
        memcpy( &(this->deletes[i]), &(this->st_deletes), sizeof(uuid_msgs::UniqueID));
      }
     return offset;
    }

    const char * getType(){ return "geographic_msgs/GeographicMapChanges"; };
    const char * getMD5(){ return "4fd027f54298203ec12aa1c4b20e6cb8"; };

  };

}
#endif