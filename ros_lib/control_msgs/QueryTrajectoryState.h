#ifndef _ROS_SERVICE_QueryTrajectoryState_h
#define _ROS_SERVICE_QueryTrajectoryState_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace control_msgs
{

static const char QUERYTRAJECTORYSTATE[] = "control_msgs/QueryTrajectoryState";

  class QueryTrajectoryStateRequest : public ros::Msg
  {
    public:
      ros::Time time;

    QueryTrajectoryStateRequest():
      time()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->time.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->time.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->time.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->time.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time.sec);
      *(outbuffer + offset + 0) = (this->time.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->time.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->time.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->time.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->time.nsec);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      this->time.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->time.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->time.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->time.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->time.sec);
      this->time.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->time.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->time.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->time.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->time.nsec);
     return offset;
    }

    const char * getType(){ return QUERYTRAJECTORYSTATE; };
    const char * getMD5(){ return "556a4fb76023a469987922359d08a844"; };

  };

  class QueryTrajectoryStateResponse : public ros::Msg
  {
    public:
      uint8_t name_length;
      char* st_name;
      char* * name;
      uint8_t position_length;
      float st_position;
      float * position;
      uint8_t velocity_length;
      float st_velocity;
      float * velocity;
      uint8_t acceleration_length;
      float st_acceleration;
      float * acceleration;

    QueryTrajectoryStateResponse():
      name_length(0), name(NULL),
      position_length(0), position(NULL),
      velocity_length(0), velocity(NULL),
      acceleration_length(0), acceleration(NULL)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset++) = name_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < name_length; i++){
      uint32_t length_namei = strlen(this->name[i]);
      memcpy(outbuffer + offset, &length_namei, sizeof(uint32_t));
      offset += 4;
      memcpy(outbuffer + offset, this->name[i], length_namei);
      offset += length_namei;
      }
      *(outbuffer + offset++) = position_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < position_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->position[i]);
      }
      *(outbuffer + offset++) = velocity_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < velocity_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->velocity[i]);
      }
      *(outbuffer + offset++) = acceleration_length;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      *(outbuffer + offset++) = 0;
      for( uint8_t i = 0; i < acceleration_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->acceleration[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      uint8_t name_lengthT = *(inbuffer + offset++);
      if(name_lengthT > name_length)
        this->name = (char**)realloc(this->name, name_lengthT * sizeof(char*));
      offset += 3;
      name_length = name_lengthT;
      for( uint8_t i = 0; i < name_length; i++){
      uint32_t length_st_name;
      memcpy(&length_st_name, (inbuffer + offset), sizeof(uint32_t));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_name-1]=0;
      this->st_name = (char *)(inbuffer + offset-1);
      offset += length_st_name;
        memcpy( &(this->name[i]), &(this->st_name), sizeof(char*));
      }
      uint8_t position_lengthT = *(inbuffer + offset++);
      if(position_lengthT > position_length)
        this->position = (float*)realloc(this->position, position_lengthT * sizeof(float));
      offset += 3;
      position_length = position_lengthT;
      for( uint8_t i = 0; i < position_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_position));
        memcpy( &(this->position[i]), &(this->st_position), sizeof(float));
      }
      uint8_t velocity_lengthT = *(inbuffer + offset++);
      if(velocity_lengthT > velocity_length)
        this->velocity = (float*)realloc(this->velocity, velocity_lengthT * sizeof(float));
      offset += 3;
      velocity_length = velocity_lengthT;
      for( uint8_t i = 0; i < velocity_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_velocity));
        memcpy( &(this->velocity[i]), &(this->st_velocity), sizeof(float));
      }
      uint8_t acceleration_lengthT = *(inbuffer + offset++);
      if(acceleration_lengthT > acceleration_length)
        this->acceleration = (float*)realloc(this->acceleration, acceleration_lengthT * sizeof(float));
      offset += 3;
      acceleration_length = acceleration_lengthT;
      for( uint8_t i = 0; i < acceleration_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_acceleration));
        memcpy( &(this->acceleration[i]), &(this->st_acceleration), sizeof(float));
      }
     return offset;
    }

    const char * getType(){ return QUERYTRAJECTORYSTATE; };
    const char * getMD5(){ return "1f1a6554ad060f44d013e71868403c1a"; };

  };

  class QueryTrajectoryState {
    public:
    typedef QueryTrajectoryStateRequest Request;
    typedef QueryTrajectoryStateResponse Response;
  };

}
#endif
