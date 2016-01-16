#ifndef _ROS_geographic_msgs_RouteNetwork_h
#define _ROS_geographic_msgs_RouteNetwork_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "uuid_msgs/UniqueID.h"
#include "geographic_msgs/BoundingBox.h"
#include "geographic_msgs/WayPoint.h"
#include "geographic_msgs/RouteSegment.h"
#include "geographic_msgs/KeyValue.h"

namespace geographic_msgs
{

  class RouteNetwork : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uuid_msgs::UniqueID id;
      geographic_msgs::BoundingBox bounds;
      uint8_t points_length;
      geographic_msgs::WayPoint st_points;
      geographic_msgs::WayPoint * points;
      uint8_t segments_length;
      geographic_msgs::RouteSegment st_segments;
      geographic_msgs::RouteSegment * segments;
      uint8_t props_length;
      geographic_msgs::KeyValue st_props;
      geographic_msgs::KeyValue * props;

    RouteNetwork():
      header(),
      id(),
      bounds(),
      points_length(0), points(NULL),
      segments_length(0), segments(NULL),
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
      uint8_t segments_lengthT = *(inbuffer + offset++);
      if(segments_lengthT > segments_length)
        this->segments = (geographic_msgs::RouteSegment*)realloc(this->segments, segments_lengthT * sizeof(geographic_msgs::RouteSegment));
      offset += 3;
      segments_length = segments_lengthT;
      for( uint8_t i = 0; i < segments_length; i++){
      offset += this->st_segments.deserialize(inbuffer + offset);
        memcpy( &(this->segments[i]), &(this->st_segments), sizeof(geographic_msgs::RouteSegment));
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

    const char * getType(){ return "geographic_msgs/RouteNetwork"; };
    const char * getMD5(){ return "fd717c0a34a7c954deed32c6847f30a8"; };

  };

}
#endif