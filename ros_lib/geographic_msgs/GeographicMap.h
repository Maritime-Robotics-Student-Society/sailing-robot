#ifndef _ROS_geographic_msgs_GeographicMap_h
#define _ROS_geographic_msgs_GeographicMap_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "uuid_msgs/UniqueID.h"
#include "geographic_msgs/BoundingBox.h"
#include "geographic_msgs/WayPoint.h"
#include "geographic_msgs/MapFeature.h"
#include "geographic_msgs/KeyValue.h"

namespace geographic_msgs
{

  class GeographicMap : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uuid_msgs::UniqueID id;
      geographic_msgs::BoundingBox bounds;
      uint8_t points_length;
      geographic_msgs::WayPoint st_points;
      geographic_msgs::WayPoint * points;
      uint8_t features_length;
      geographic_msgs::MapFeature st_features;
      geographic_msgs::MapFeature * features;
      uint8_t props_length;
      geographic_msgs::KeyValue st_props;
      geographic_msgs::KeyValue * props;

    GeographicMap():
      header(),
      id(),
      bounds(),
      points_length(0), points(NULL),
      features_length(0), features(NULL),
      props_length(0), props(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->id.serialize(outbuffer + offset);
      offset += this->bounds.serialize(outbuffer + offset);
      *(outbuffer + offset++) = points_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < points_length; i++){
      offset += this->points[i].serialize(outbuffer + offset);
      }
      *(outbuffer + offset++) = features_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < features_length; i++){
      offset += this->features[i].serialize(outbuffer + offset);
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
      offset += this->id.deserialize(inbuffer + offset);
      offset += this->bounds.deserialize(inbuffer + offset);
      uint8_t points_lengthT = *(inbuffer + offset++);
      if(points_lengthT > points_length)
        this->points = (geographic_msgs::WayPoint*)realloc(this->points, points_lengthT * sizeof(geographic_msgs::WayPoint));
      offset += 3;
      points_length = points_lengthT;
      for( uint8_t i = 0; i < points_length; i++){
      offset += this->st_points.deserialize(inbuffer + offset);
        memcpy( &(this->points[i]), &(this->st_points), sizeof(geographic_msgs::WayPoint));
      }
      uint8_t features_lengthT = *(inbuffer + offset++);
      if(features_lengthT > features_length)
        this->features = (geographic_msgs::MapFeature*)realloc(this->features, features_lengthT * sizeof(geographic_msgs::MapFeature));
      offset += 3;
      features_length = features_lengthT;
      for( uint8_t i = 0; i < features_length; i++){
      offset += this->st_features.deserialize(inbuffer + offset);
        memcpy( &(this->features[i]), &(this->st_features), sizeof(geographic_msgs::MapFeature));
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

    const char * getType(){ return "geographic_msgs/GeographicMap"; };
    const char * getMD5(){ return "0f4ce6d2ebf9ac9c7c4f3308f6ae0731"; };

  };

}
#endif