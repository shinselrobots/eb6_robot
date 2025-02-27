#ifndef _ROS_dynamixel_msgs_JointStateArray_h
#define _ROS_dynamixel_msgs_JointStateArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"

namespace dynamixel_msgs
{

  class JointStateArray : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t name_length;
      typedef char* _name_type;
      _name_type st_name;
      _name_type * name;
      uint32_t position_length;
      typedef float _position_type;
      _position_type st_position;
      _position_type * position;
      uint32_t velocity_length;
      typedef float _velocity_type;
      _velocity_type st_velocity;
      _velocity_type * velocity;
      uint32_t effort_length;
      typedef float _effort_type;
      _effort_type st_effort;
      _effort_type * effort;
      uint32_t goal_pos_length;
      typedef float _goal_pos_type;
      _goal_pos_type st_goal_pos;
      _goal_pos_type * goal_pos;
      uint32_t error_length;
      typedef float _error_type;
      _error_type st_error;
      _error_type * error;
      uint32_t temp_length;
      typedef float _temp_type;
      _temp_type st_temp;
      _temp_type * temp;

    JointStateArray():
      header(),
      name_length(0), st_name(), name(nullptr),
      position_length(0), st_position(), position(nullptr),
      velocity_length(0), st_velocity(), velocity(nullptr),
      effort_length(0), st_effort(), effort(nullptr),
      goal_pos_length(0), st_goal_pos(), goal_pos(nullptr),
      error_length(0), st_error(), error(nullptr),
      temp_length(0), st_temp(), temp(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->name_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->name_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->name_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->name_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->name_length);
      for( uint32_t i = 0; i < name_length; i++){
      uint32_t length_namei = strlen(this->name[i]);
      varToArr(outbuffer + offset, length_namei);
      offset += 4;
      memcpy(outbuffer + offset, this->name[i], length_namei);
      offset += length_namei;
      }
      *(outbuffer + offset + 0) = (this->position_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->position_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->position_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->position_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->position_length);
      for( uint32_t i = 0; i < position_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->position[i]);
      }
      *(outbuffer + offset + 0) = (this->velocity_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->velocity_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->velocity_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->velocity_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->velocity_length);
      for( uint32_t i = 0; i < velocity_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->velocity[i]);
      }
      *(outbuffer + offset + 0) = (this->effort_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->effort_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->effort_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->effort_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->effort_length);
      for( uint32_t i = 0; i < effort_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->effort[i]);
      }
      *(outbuffer + offset + 0) = (this->goal_pos_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->goal_pos_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->goal_pos_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->goal_pos_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->goal_pos_length);
      for( uint32_t i = 0; i < goal_pos_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->goal_pos[i]);
      }
      *(outbuffer + offset + 0) = (this->error_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->error_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->error_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->error_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->error_length);
      for( uint32_t i = 0; i < error_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->error[i]);
      }
      *(outbuffer + offset + 0) = (this->temp_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->temp_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->temp_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->temp_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->temp_length);
      for( uint32_t i = 0; i < temp_length; i++){
      offset += serializeAvrFloat64(outbuffer + offset, this->temp[i]);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t name_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      name_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      name_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      name_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->name_length);
      if(name_lengthT > name_length)
        this->name = (char**)realloc(this->name, name_lengthT * sizeof(char*));
      name_length = name_lengthT;
      for( uint32_t i = 0; i < name_length; i++){
      uint32_t length_st_name;
      arrToVar(length_st_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_name-1]=0;
      this->st_name = (char *)(inbuffer + offset-1);
      offset += length_st_name;
        memcpy( &(this->name[i]), &(this->st_name), sizeof(char*));
      }
      uint32_t position_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      position_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      position_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      position_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->position_length);
      if(position_lengthT > position_length)
        this->position = (float*)realloc(this->position, position_lengthT * sizeof(float));
      position_length = position_lengthT;
      for( uint32_t i = 0; i < position_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_position));
        memcpy( &(this->position[i]), &(this->st_position), sizeof(float));
      }
      uint32_t velocity_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      velocity_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      velocity_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      velocity_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->velocity_length);
      if(velocity_lengthT > velocity_length)
        this->velocity = (float*)realloc(this->velocity, velocity_lengthT * sizeof(float));
      velocity_length = velocity_lengthT;
      for( uint32_t i = 0; i < velocity_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_velocity));
        memcpy( &(this->velocity[i]), &(this->st_velocity), sizeof(float));
      }
      uint32_t effort_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      effort_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      effort_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      effort_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->effort_length);
      if(effort_lengthT > effort_length)
        this->effort = (float*)realloc(this->effort, effort_lengthT * sizeof(float));
      effort_length = effort_lengthT;
      for( uint32_t i = 0; i < effort_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_effort));
        memcpy( &(this->effort[i]), &(this->st_effort), sizeof(float));
      }
      uint32_t goal_pos_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      goal_pos_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      goal_pos_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      goal_pos_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->goal_pos_length);
      if(goal_pos_lengthT > goal_pos_length)
        this->goal_pos = (float*)realloc(this->goal_pos, goal_pos_lengthT * sizeof(float));
      goal_pos_length = goal_pos_lengthT;
      for( uint32_t i = 0; i < goal_pos_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_goal_pos));
        memcpy( &(this->goal_pos[i]), &(this->st_goal_pos), sizeof(float));
      }
      uint32_t error_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      error_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      error_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      error_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->error_length);
      if(error_lengthT > error_length)
        this->error = (float*)realloc(this->error, error_lengthT * sizeof(float));
      error_length = error_lengthT;
      for( uint32_t i = 0; i < error_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_error));
        memcpy( &(this->error[i]), &(this->st_error), sizeof(float));
      }
      uint32_t temp_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      temp_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      temp_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      temp_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->temp_length);
      if(temp_lengthT > temp_length)
        this->temp = (float*)realloc(this->temp, temp_lengthT * sizeof(float));
      temp_length = temp_lengthT;
      for( uint32_t i = 0; i < temp_length; i++){
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->st_temp));
        memcpy( &(this->temp[i]), &(this->st_temp), sizeof(float));
      }
     return offset;
    }

    virtual const char * getType() override { return "dynamixel_msgs/JointStateArray"; };
    virtual const char * getMD5() override { return "2857eb4ae78ccf339b833d524b1c4439"; };

  };

}
#endif
