#ifndef _ROS_SERVICE_GetPolledImage_h
#define _ROS_SERVICE_GetPolledImage_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "sensor_msgs/RegionOfInterest.h"
#include "ros/duration.h"
#include "ros/time.h"

namespace polled_camera
{

static const char GETPOLLEDIMAGE[] = "polled_camera/GetPolledImage";

  class GetPolledImageRequest : public ros::Msg
  {
    public:
      const char* response_namespace;
      ros::Duration timeout;
      uint32_t binning_x;
      uint32_t binning_y;
      sensor_msgs::RegionOfInterest roi;

    GetPolledImageRequest():
      response_namespace(""),
      timeout(),
      binning_x(0),
      binning_y(0),
      roi()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      uint32_t length_response_namespace = strlen(this->response_namespace);
      memcpy(outbuffer + offset, &length_response_namespace, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->response_namespace, length_response_namespace);
      offset += length_response_namespace;
      *(outbuffer + offset + 0) = (this->timeout.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timeout.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timeout.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timeout.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timeout.sec);
      *(outbuffer + offset + 0) = (this->timeout.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->timeout.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->timeout.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->timeout.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->timeout.nsec);
      *(outbuffer + offset + 0) = (this->binning_x >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->binning_x >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->binning_x >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->binning_x >> (8 * 3)) & 0xFF;
      offset += sizeof(this->binning_x);
      *(outbuffer + offset + 0) = (this->binning_y >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->binning_y >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->binning_y >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->binning_y >> (8 * 3)) & 0xFF;
      offset += sizeof(this->binning_y);
      offset += this->roi.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint32_t length_response_namespace;
      memcpy(&length_response_namespace, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_response_namespace; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_response_namespace-1]=0;
      this->response_namespace = (char *)(inbuffer + offset-1);
      offset += length_response_namespace;
      this->timeout.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->timeout.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timeout.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timeout.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->timeout.sec);
      this->timeout.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->timeout.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->timeout.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->timeout.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->timeout.nsec);
      this->binning_x =  ((uint32_t) (*(inbuffer + offset)));
      this->binning_x |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->binning_x |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->binning_x |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->binning_x);
      this->binning_y =  ((uint32_t) (*(inbuffer + offset)));
      this->binning_y |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->binning_y |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->binning_y |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->binning_y);
      offset += this->roi.deserialize(inbuffer + offset);
     return offset;
    }

    const char * getType(){ return GETPOLLEDIMAGE; };
    const char * getMD5(){ return "c77ed43e530fd48e9e7a2a93845e154c"; };

  };

  class GetPolledImageResponse : public ros::Msg
  {
    public:
      bool success;
      const char* status_message;
      ros::Time stamp;

    GetPolledImageResponse():
      success(0),
      status_message(""),
      stamp()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      uint32_t length_status_message = strlen(this->status_message);
      memcpy(outbuffer + offset, &length_status_message, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->status_message, length_status_message);
      offset += length_status_message;
      *(outbuffer + offset + 0) = (this->stamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.sec);
      *(outbuffer + offset + 0) = (this->stamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.nsec);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
      uint32_t length_status_message;
      memcpy(&length_status_message, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_status_message; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_status_message-1]=0;
      this->status_message = (char *)(inbuffer + offset-1);
      offset += length_status_message;
      this->stamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.sec);
      this->stamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.nsec);
     return offset;
    }

    const char * getType(){ return GETPOLLEDIMAGE; };
    const char * getMD5(){ return "dbf1f851bc511800e6129ccd5a3542ab"; };

  };

  class GetPolledImage {
    public:
    typedef GetPolledImageRequest Request;
    typedef GetPolledImageResponse Response;
  };

}
#endif
