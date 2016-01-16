#ifndef _ROS_opencv_apps_Face_h
#define _ROS_opencv_apps_Face_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "opencv_apps/Rect.h"

namespace opencv_apps
{

  class Face : public ros::Msg
  {
    public:
      opencv_apps::Rect face;
      uint8_t eyes_length;
      opencv_apps::Rect st_eyes;
      opencv_apps::Rect * eyes;

    Face():
      face(),
      eyes_length(0), eyes(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->face.serialize(outbuffer + offset);
      *(outbuffer + offset++) = eyes_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < eyes_length; i++){
      offset += this->eyes[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->face.deserialize(inbuffer + offset);
      uint8_t eyes_lengthT = *(inbuffer + offset++);
      if(eyes_lengthT > eyes_length)
        this->eyes = (opencv_apps::Rect*)realloc(this->eyes, eyes_lengthT * sizeof(opencv_apps::Rect));
      offset += 3;
      eyes_length = eyes_lengthT;
      for( uint8_t i = 0; i < eyes_length; i++){
      offset += this->st_eyes.deserialize(inbuffer + offset);
        memcpy( &(this->eyes[i]), &(this->st_eyes), sizeof(opencv_apps::Rect));
      }
     return offset;
    }

    const char * getType(){ return "opencv_apps/Face"; };
    const char * getMD5(){ return "0c2547d2eaf71219898bf5c25e36907e"; };

  };

}
#endif