#ifndef _ROS_opencv_apps_ContourArrayStamped_h
#define _ROS_opencv_apps_ContourArrayStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "opencv_apps/Contour.h"

namespace opencv_apps
{

  class ContourArrayStamped : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t contours_length;
      opencv_apps::Contour st_contours;
      opencv_apps::Contour * contours;

    ContourArrayStamped():
      header(),
      contours_length(0), contours(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset++) = contours_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < contours_length; i++){
      offset += this->contours[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint8_t contours_lengthT = *(inbuffer + offset++);
      if(contours_lengthT > contours_length)
        this->contours = (opencv_apps::Contour*)realloc(this->contours, contours_lengthT * sizeof(opencv_apps::Contour));
      offset += 3;
      contours_length = contours_lengthT;
      for( uint8_t i = 0; i < contours_length; i++){
      offset += this->st_contours.deserialize(inbuffer + offset);
        memcpy( &(this->contours[i]), &(this->st_contours), sizeof(opencv_apps::Contour));
      }
     return offset;
    }

    const char * getType(){ return "opencv_apps/ContourArrayStamped"; };
    const char * getMD5(){ return "6bcf2733566be102cf11fc89685fd962"; };

  };

}
#endif