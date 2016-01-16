#ifndef _ROS_opencv_apps_FaceArray_h
#define _ROS_opencv_apps_FaceArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "opencv_apps/Face.h"

namespace opencv_apps
{

  class FaceArray : public ros::Msg
  {
    public:
      uint8_t faces_length;
      opencv_apps::Face st_faces;
      opencv_apps::Face * faces;

    FaceArray():
      faces_length(0), faces(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = faces_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < faces_length; i++){
      offset += this->faces[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t faces_lengthT = *(inbuffer + offset++);
      if(faces_lengthT > faces_length)
        this->faces = (opencv_apps::Face*)realloc(this->faces, faces_lengthT * sizeof(opencv_apps::Face));
      offset += 3;
      faces_length = faces_lengthT;
      for( uint8_t i = 0; i < faces_length; i++){
      offset += this->st_faces.deserialize(inbuffer + offset);
        memcpy( &(this->faces[i]), &(this->st_faces), sizeof(opencv_apps::Face));
      }
     return offset;
    }

    const char * getType(){ return "opencv_apps/FaceArray"; };
    const char * getMD5(){ return "40b464276ad8e3c5012f7a3a93eed2a4"; };

  };

}
#endif