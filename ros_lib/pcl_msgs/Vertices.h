#ifndef _ROS_pcl_msgs_Vertices_h
#define _ROS_pcl_msgs_Vertices_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace pcl_msgs
{

  class Vertices : public ros::Msg
  {
    public:
      uint8_t vertices_length;
      uint32_t st_vertices;
      uint32_t * vertices;

    Vertices():
      vertices_length(0), vertices(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = vertices_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < vertices_length; i++){
      *(outbuffer + offset + 0) = (this->vertices[i] >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->vertices[i] >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->vertices[i] >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->vertices[i] >> (8 * 3)) & 0xFF;
      offset += sizeof(this->vertices[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t vertices_lengthT = *(inbuffer + offset++);
      if(vertices_lengthT > vertices_length)
        this->vertices = (uint32_t*)realloc(this->vertices, vertices_lengthT * sizeof(uint32_t));
      offset += 3;
      vertices_length = vertices_lengthT;
      for( uint8_t i = 0; i < vertices_length; i++){
      this->st_vertices =  ((uint32_t) (*(inbuffer + offset)));
      this->st_vertices |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->st_vertices |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->st_vertices |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->st_vertices);
        memcpy( &(this->vertices[i]), &(this->st_vertices), sizeof(uint32_t));
      }
     return offset;
    }

    const char * getType(){ return "pcl_msgs/Vertices"; };
    const char * getMD5(){ return "39bd7b1c23763ddd1b882b97cb7cfe11"; };

  };

}
#endif