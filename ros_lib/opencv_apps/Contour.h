#ifndef _ROS_opencv_apps_Contour_h
#define _ROS_opencv_apps_Contour_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "opencv_apps/Point2D.h"

namespace opencv_apps
{

  class Contour : public ros::Msg
  {
    public:
      uint8_t points_length;
      opencv_apps::Point2D st_points;
      opencv_apps::Point2D * points;

    Contour():
      points_length(0), points(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = points_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < points_length; i++){
      offset += this->points[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t points_lengthT = *(inbuffer + offset++);
      if(points_lengthT > points_length)
        this->points = (opencv_apps::Point2D*)realloc(this->points, points_lengthT * sizeof(opencv_apps::Point2D));
      offset += 3;
      points_length = points_lengthT;
      for( uint8_t i = 0; i < points_length; i++){
      offset += this->st_points.deserialize(inbuffer + offset);
        memcpy( &(this->points[i]), &(this->st_points), sizeof(opencv_apps::Point2D));
      }
     return offset;
    }

    const char * getType(){ return "opencv_apps/Contour"; };
    const char * getMD5(){ return "8f02263beef99aa03117a577a3eb879d"; };

  };

}
#endif