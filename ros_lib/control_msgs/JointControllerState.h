#ifndef _ROS_control_msgs_JointControllerState_h
#define _ROS_control_msgs_JointControllerState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace control_msgs
{

  class JointControllerState : public ros::Msg
  {
    public:
      std_msgs::Header header;
      float set_point;
      float process_value;
      float process_value_dot;
      float error;
      float time_step;
      float command;
      float p;
      float i;
      float d;
      float i_clamp;

    JointControllerState():
      header(),
      set_point(0),
      process_value(0),
      process_value_dot(0),
      error(0),
      time_step(0),
      command(0),
      p(0),
      i(0),
      d(0),
      i_clamp(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      offset += serializeAvrFloat64(outbuffer + offset, this->set_point);
      offset += serializeAvrFloat64(outbuffer + offset, this->process_value);
      offset += serializeAvrFloat64(outbuffer + offset, this->process_value_dot);
      offset += serializeAvrFloat64(outbuffer + offset, this->error);
      offset += serializeAvrFloat64(outbuffer + offset, this->time_step);
      offset += serializeAvrFloat64(outbuffer + offset, this->command);
      offset += serializeAvrFloat64(outbuffer + offset, this->p);
      offset += serializeAvrFloat64(outbuffer + offset, this->i);
      offset += serializeAvrFloat64(outbuffer + offset, this->d);
      offset += serializeAvrFloat64(outbuffer + offset, this->i_clamp);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->set_point));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->process_value));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->process_value_dot));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->error));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->time_step));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->command));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->p));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->i));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->d));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->i_clamp));
     return offset;
    }

    const char * getType(){ return "control_msgs/JointControllerState"; };
    const char * getMD5(){ return "c0d034a7bf20aeb1c37f3eccb7992b69"; };

  };

}
#endif