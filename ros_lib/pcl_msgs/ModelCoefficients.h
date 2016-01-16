#ifndef _ROS_pcl_msgs_ModelCoefficients_h
#define _ROS_pcl_msgs_ModelCoefficients_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace pcl_msgs
{

  class ModelCoefficients : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t values_length;
      float st_values;
      float * values;

    ModelCoefficients():
      header(),
      values_length(0), values(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = values_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < values_length; i++){
      union {
        float real;
        uint32_t base;
      } u_valuesi;
      u_valuesi.real = this->values[i];
      *(outbuffer + offset + 0) = (u_valuesi.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_valuesi.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_valuesi.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_valuesi.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->values[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint8_t values_lengthT = *(inbuffer + offset++);
      if(values_lengthT > values_length)
        this->values = (float*)realloc(this->values, values_lengthT * sizeof(float));
      offset += 3;
      values_length = values_lengthT;
      for( uint8_t i = 0; i < values_length; i++){
      union {
        float real;
        uint32_t base;
      } u_st_values;
      u_st_values.base = 0;
      u_st_values.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_st_values.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_st_values.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_st_values.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->st_values = u_st_values.real;
      offset += sizeof(this->st_values);
        memcpy( &(this->values[i]), &(this->st_values), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "pcl_msgs/ModelCoefficients"; };
    const char * getMD5(){ return "ca27dea75e72cb894cd36f9e5005e93e"; };

  };

}
#endif