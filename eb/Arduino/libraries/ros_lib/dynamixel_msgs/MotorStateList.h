#ifndef _ROS_dynamixel_msgs_MotorStateList_h
#define _ROS_dynamixel_msgs_MotorStateList_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "dynamixel_msgs/MotorState.h"

namespace dynamixel_msgs
{

  class MotorStateList : public ros::Msg
  {
    public:
      uint32_t motor_states_length;
      typedef dynamixel_msgs::MotorState _motor_states_type;
      _motor_states_type st_motor_states;
      _motor_states_type * motor_states;

    MotorStateList():
      motor_states_length(0), st_motor_states(), motor_states(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->motor_states_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->motor_states_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->motor_states_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->motor_states_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->motor_states_length);
      for( uint32_t i = 0; i < motor_states_length; i++){
      offset += this->motor_states[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t motor_states_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      motor_states_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      motor_states_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      motor_states_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->motor_states_length);
      if(motor_states_lengthT > motor_states_length)
        this->motor_states = (dynamixel_msgs::MotorState*)realloc(this->motor_states, motor_states_lengthT * sizeof(dynamixel_msgs::MotorState));
      motor_states_length = motor_states_lengthT;
      for( uint32_t i = 0; i < motor_states_length; i++){
      offset += this->st_motor_states.deserialize(inbuffer + offset);
        memcpy( &(this->motor_states[i]), &(this->st_motor_states), sizeof(dynamixel_msgs::MotorState));
      }
     return offset;
    }

    virtual const char * getType() override { return "dynamixel_msgs/MotorStateList"; };
    virtual const char * getMD5() override { return "9e94ccf6563ca78afce19eb097f9343c"; };

  };

}
#endif
