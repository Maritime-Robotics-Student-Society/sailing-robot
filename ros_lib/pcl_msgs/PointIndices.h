#ifndef _ROS_pcl_msgs_PointIndices_h
#define _ROS_pcl_msgs_PointIndices_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace pcl_msgs
{

  class PointIndices : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t indices_length;
      int32_t st_indices;
      int32_t * indices;

    PointIndices():
      header(),
      indices_length(0), indices(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = indices_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < indices_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_indicesi;
      u_indicesi.real = this->indices[i];
      *(outbuffer + offset + 0) = (u_indicesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_indicesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_indicesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_indicesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->indices[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint8_t indices_lengthT = *(inbuffer + offset++);
      if(indices_lengthT > indices_length)
        this->indices = (int32_t*)realloc(this->indices, indices_lengthT * sizeof(int32_t));
      offset += 3;
      indices_length = indices_lengthT;
      for( uint8_t i = 0; i < indices_length; i++){
      union {
        int32_t real;
        uint32_t base;
      } u_st_indices;
      u_st_indices.base = 0;
      u_st_indices.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_indices.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_indices.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_indices.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_indices = u_st_indices.real;
      offset += sizeof(this->st_indices);
        memcpy( &(this->indices[i]), &(this->st_indices), sizeof(int32_t));
      }
     return offset;
    }

    const char * getType(){ return "pcl_msgs/PointIndices"; };
    const char * getMD5(){ return "458c7998b7eaf99908256472e273b3d4"; };

  };

}
#endif