#ifndef _ROS_opencv_apps_FaceArrayStamped_h
#define _ROS_opencv_apps_FaceArrayStamped_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "opencv_apps/Face.h"

namespace opencv_apps
{

  class FaceArrayStamped : public ros::Msg
  {
    public:
      std_msgs::Header header;
      uint8_t faces_length;
      opencv_apps::Face st_faces;
      opencv_apps::Face * faces;

    FaceArrayStamped():
      header(),
      faces_length(0), faces(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
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
      offset += this->header.deserialize(inbuffer + offset);
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

    const char * getType(){ return "opencv_apps/FaceArrayStamped"; };
    const char * getMD5(){ return "bf258edc868c139ea6c94254d9ab51e5"; };

  };

}
#endif