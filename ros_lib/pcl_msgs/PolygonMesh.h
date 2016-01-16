#ifndef _ROS_pcl_msgs_PolygonMesh_h
#define _ROS_pcl_msgs_PolygonMesh_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_msgs/Vertices.h"

namespace pcl_msgs
{

  class PolygonMesh : public ros::Msg
  {
    public:
      std_msgs::Header header;
      sensor_msgs::PointCloud2 cloud;
      uint8_t polygons_length;
      pcl_msgs::Vertices st_polygons;
      pcl_msgs::Vertices * polygons;

    PolygonMesh():
      header(),
      cloud(),
      polygons_length(0), polygons(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += this->cloud.serialize(outbuffer + offset);
      *(outbuffer + offset++) = polygons_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < polygons_length; i++){
      offset += this->polygons[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += this->cloud.deserialize(inbuffer + offset);
      uint8_t polygons_lengthT = *(inbuffer + offset++);
      if(polygons_lengthT > polygons_length)
        this->polygons = (pcl_msgs::Vertices*)realloc(this->polygons, polygons_lengthT * sizeof(pcl_msgs::Vertices));
      offset += 3;
      polygons_length = polygons_lengthT;
      for( uint8_t i = 0; i < polygons_length; i++){
      offset += this->st_polygons.deserialize(inbuffer + offset);
        memcpy( &(this->polygons[i]), &(this->st_polygons), sizeof(pcl_msgs::Vertices));
      }
     return offset;
    }

    const char * getType(){ return "pcl_msgs/PolygonMesh"; };
    const char * getMD5(){ return "45a5fc6ad2cde8489600a790acc9a38a"; };

  };

}
#endif