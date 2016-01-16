#ifndef _ROS_opencv_apps_MomentArrayStamped_h
#define _ROS_opencv_apps_MomentArrayStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "opencv_apps/Moment.h"

namespace opencv_apps
{

  class MomentArrayStamped : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t moments_length;
      opencv_apps::Moment st_moments;
      opencv_apps::Moment * moments;

    MomentArrayStamped():
      header(),
      moments_length(0), moments(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = moments_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < moments_length; i++){
      offset += this->moments[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint8_t moments_lengthT = *(inbuffer + offset++);
      if(moments_lengthT > moments_length)
        this->moments = (opencv_apps::Moment*)realloc(this->moments, moments_lengthT * sizeof(opencv_apps::Moment));
      offset += 3;
      moments_length = moments_lengthT;
      for( uint8_t i = 0; i < moments_length; i++){
      offset += this->st_moments.deserialize(inbuffer + offset);
        memcpy( &(this->moments[i]), &(this->st_moments), sizeof(opencv_apps::Moment));
      }
     return offset;
    }

    const char * getType(){ return "opencv_apps/MomentArrayStamped"; };
    const char * getMD5(){ return "28ac0beb70b037acf76c3bed71b679a9"; };

  };

}
#endif