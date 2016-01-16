#ifndef _ROS_gazebo_msgs_ODEJointProperties_h
#define _ROS_gazebo_msgs_ODEJointProperties_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace gazebo_msgs
{

  class ODEJointProperties : public ros::Msg
  {
    public:
      uint8_t damping_length;
      float st_damping;
      float * damping;
      uint8_t hiStop_length;
      float st_hiStop;
      float * hiStop;
      uint8_t loStop_length;
      float st_loStop;
      float * loStop;
      uint8_t erp_length;
      float st_erp;
      float * erp;
      uint8_t cfm_length;
      float st_cfm;
      float * cfm;
      uint8_t stop_erp_length;
      float st_stop_erp;
      float * stop_erp;
      uint8_t stop_cfm_length;
      float st_stop_cfm;
      float * stop_cfm;
      uint8_t fudge_factor_length;
      float st_fudge_factor;
      float * fudge_factor;
      uint8_t fmax_length;
      float st_fmax;
      float * fmax;
      uint8_t vel_length;
      float st_vel;
      float * vel;

    ODEJointProperties():
      damping_length(0), damping(NULL),
      hiStop_length(0), hiStop(NULL),
      loStop_length(0), loStop(NULL),
      erp_length(0), erp(NULL),
      cfm_length(0), cfm(NULL),
      stop_erp_length(0), stop_erp(NULL),
      stop_cfm_length(0), stop_cfm(NULL),
      fudge_factor_length(0), fudge_factor(NULL),
      fmax_length(0), fmax(NULL),
      vel_length(0), vel(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = damping_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < damping_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->damping[i]);
      }
      *(outbuffer + offset++) = hiStop_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < hiStop_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->hiStop[i]);
      }
      *(outbuffer + offset++) = loStop_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < loStop_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->loStop[i]);
      }
      *(outbuffer + offset++) = erp_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < erp_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->erp[i]);
      }
      *(outbuffer + offset++) = cfm_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < cfm_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->cfm[i]);
      }
      *(outbuffer + offset++) = stop_erp_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < stop_erp_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->stop_erp[i]);
      }
      *(outbuffer + offset++) = stop_cfm_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < stop_cfm_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->stop_cfm[i]);
      }
      *(outbuffer + offset++) = fudge_factor_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < fudge_factor_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->fudge_factor[i]);
      }
      *(outbuffer + offset++) = fmax_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < fmax_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->fmax[i]);
      }
      *(outbuffer + offset++) = vel_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < vel_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->vel[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t damping_lengthT = *(inbuffer + offset++);
      if(damping_lengthT > damping_length)
        this->damping = (float*)realloc(this->damping, damping_lengthT * sizeof(float));
      offset += 3;
      damping_length = damping_lengthT;
      for( uint8_t i = 0; i < damping_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_damping));
        memcpy( &(this->damping[i]), &(this->st_damping), sizeof(float));
      }
      uint8_t hiStop_lengthT = *(inbuffer + offset++);
      if(hiStop_lengthT > hiStop_length)
        this->hiStop = (float*)realloc(this->hiStop, hiStop_lengthT * sizeof(float));
      offset += 3;
      hiStop_length = hiStop_lengthT;
      for( uint8_t i = 0; i < hiStop_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_hiStop));
        memcpy( &(this->hiStop[i]), &(this->st_hiStop), sizeof(float));
      }
      uint8_t loStop_lengthT = *(inbuffer + offset++);
      if(loStop_lengthT > loStop_length)
        this->loStop = (float*)realloc(this->loStop, loStop_lengthT * sizeof(float));
      offset += 3;
      loStop_length = loStop_lengthT;
      for( uint8_t i = 0; i < loStop_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_loStop));
        memcpy( &(this->loStop[i]), &(this->st_loStop), sizeof(float));
      }
      uint8_t erp_lengthT = *(inbuffer + offset++);
      if(erp_lengthT > erp_length)
        this->erp = (float*)realloc(this->erp, erp_lengthT * sizeof(float));
      offset += 3;
      erp_length = erp_lengthT;
      for( uint8_t i = 0; i < erp_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_erp));
        memcpy( &(this->erp[i]), &(this->st_erp), sizeof(float));
      }
      uint8_t cfm_lengthT = *(inbuffer + offset++);
      if(cfm_lengthT > cfm_length)
        this->cfm = (float*)realloc(this->cfm, cfm_lengthT * sizeof(float));
      offset += 3;
      cfm_length = cfm_lengthT;
      for( uint8_t i = 0; i < cfm_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_cfm));
        memcpy( &(this->cfm[i]), &(this->st_cfm), sizeof(float));
      }
      uint8_t stop_erp_lengthT = *(inbuffer + offset++);
      if(stop_erp_lengthT > stop_erp_length)
        this->stop_erp = (float*)realloc(this->stop_erp, stop_erp_lengthT * sizeof(float));
      offset += 3;
      stop_erp_length = stop_erp_lengthT;
      for( uint8_t i = 0; i < stop_erp_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_stop_erp));
        memcpy( &(this->stop_erp[i]), &(this->st_stop_erp), sizeof(float));
      }
      uint8_t stop_cfm_lengthT = *(inbuffer + offset++);
      if(stop_cfm_lengthT > stop_cfm_length)
        this->stop_cfm = (float*)realloc(this->stop_cfm, stop_cfm_lengthT * sizeof(float));
      offset += 3;
      stop_cfm_length = stop_cfm_lengthT;
      for( uint8_t i = 0; i < stop_cfm_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_stop_cfm));
        memcpy( &(this->stop_cfm[i]), &(this->st_stop_cfm), sizeof(float));
      }
      uint8_t fudge_factor_lengthT = *(inbuffer + offset++);
      if(fudge_factor_lengthT > fudge_factor_length)
        this->fudge_factor = (float*)realloc(this->fudge_factor, fudge_factor_lengthT * sizeof(float));
      offset += 3;
      fudge_factor_length = fudge_factor_lengthT;
      for( uint8_t i = 0; i < fudge_factor_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_fudge_factor));
        memcpy( &(this->fudge_factor[i]), &(this->st_fudge_factor), sizeof(float));
      }
      uint8_t fmax_lengthT = *(inbuffer + offset++);
      if(fmax_lengthT > fmax_length)
        this->fmax = (float*)realloc(this->fmax, fmax_lengthT * sizeof(float));
      offset += 3;
      fmax_length = fmax_lengthT;
      for( uint8_t i = 0; i < fmax_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_fmax));
        memcpy( &(this->fmax[i]), &(this->st_fmax), sizeof(float));
      }
      uint8_t vel_lengthT = *(inbuffer + offset++);
      if(vel_lengthT > vel_length)
        this->vel = (float*)realloc(this->vel, vel_lengthT * sizeof(float));
      offset += 3;
      vel_length = vel_lengthT;
      for( uint8_t i = 0; i < vel_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_vel));
        memcpy( &(this->vel[i]), &(this->st_vel), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return "gazebo_msgs/ODEJointProperties"; };
    const char * getMD5(){ return "1b744c32a920af979f53afe2f9c3511f"; };

  };

}
#endif