#ifndef _ROS_opencv_apps_Moment_h
#define _ROS_opencv_apps_Moment_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "opencv_apps/Point2D.h"

namespace opencv_apps
{

  class Moment : public ros::Msg
  {
    public:
      float m00;
      float m10;
      float m01;
      float m20;
      float m11;
      float m02;
      float m30;
      float m21;
      float m12;
      float m03;
      float mu20;
      float mu11;
      float mu02;
      float mu30;
      float mu21;
      float mu12;
      float mu03;
      float nu20;
      float nu11;
      float nu02;
      float nu30;
      float nu21;
      float nu12;
      float nu03;
      opencv_apps::Point2D center;
      float length;
      float area;

    Moment():
      m00(0),
      m10(0),
      m01(0),
      m20(0),
      m11(0),
      m02(0),
      m30(0),
      m21(0),
      m12(0),
      m03(0),
      mu20(0),
      mu11(0),
      mu02(0),
      mu30(0),
      mu21(0),
      mu12(0),
      mu03(0),
      nu20(0),
      nu11(0),
      nu02(0),
      nu30(0),
      nu21(0),
      nu12(0),
      nu03(0),
      center(),
      length(0),
      area(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->m00);
      offset += serializeAvrFloat64(outbuffer + offset, this->m10);
      offset += serializeAvrFloat64(outbuffer + offset, this->m01);
      offset += serializeAvrFloat64(outbuffer + offset, this->m20);
      offset += serializeAvrFloat64(outbuffer + offset, this->m11);
      offset += serializeAvrFloat64(outbuffer + offset, this->m02);
      offset += serializeAvrFloat64(outbuffer + offset, this->m30);
      offset += serializeAvrFloat64(outbuffer + offset, this->m21);
      offset += serializeAvrFloat64(outbuffer + offset, this->m12);
      offset += serializeAvrFloat64(outbuffer + offset, this->m03);
      offset += serializeAvrFloat64(outbuffer + offset, this->mu20);
      offset += serializeAvrFloat64(outbuffer + offset, this->mu11);
      offset += serializeAvrFloat64(outbuffer + offset, this->mu02);
      offset += serializeAvrFloat64(outbuffer + offset, this->mu30);
      offset += serializeAvrFloat64(outbuffer + offset, this->mu21);
      offset += serializeAvrFloat64(outbuffer + offset, this->mu12);
      offset += serializeAvrFloat64(outbuffer + offset, this->mu03);
      offset += serializeAvrFloat64(outbuffer + offset, this->nu20);
      offset += serializeAvrFloat64(outbuffer + offset, this->nu11);
      offset += serializeAvrFloat64(outbuffer + offset, this->nu02);
      offset += serializeAvrFloat64(outbuffer + offset, this->nu30);
      offset += serializeAvrFloat64(outbuffer + offset, this->nu21);
      offset += serializeAvrFloat64(outbuffer + offset, this->nu12);
      offset += serializeAvrFloat64(outbuffer + offset, this->nu03);
      offset += this->center.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->length);
      offset += serializeAvrFloat64(outbuffer + offset, this->area);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->m00));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->m10));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->m01));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->m20));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->m11));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->m02));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->m30));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->m21));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->m12));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->m03));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->mu20));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->mu11));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->mu02));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->mu30));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->mu21));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->mu12));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->mu03));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->nu20));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->nu11));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->nu02));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->nu30));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->nu21));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->nu12));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->nu03));
      offset += this->center.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->length));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->area));
     return offset;
    }

    const char * getType(){ return "opencv_apps/Moment"; };
    const char * getMD5(){ return "560ee3fabfffb4ed4155742d6db8a03c"; };

  };

}
#endif