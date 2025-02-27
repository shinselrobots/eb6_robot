#ifndef _ROS_dynamixel_workbench_msgs_DynamixelStateList_h
#define _ROS_dynamixel_workbench_msgs_DynamixelStateList_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "dynamixel_workbench_msgs/DynamixelState.h"

namespace dynamixel_workbench_msgs
{

  class DynamixelStateList : public ros::Msg
  {
    public:
      uint32_t dynamixel_state_length;
      typedef dynamixel_workbench_msgs::DynamixelState _dynamixel_state_type;
      _dynamixel_state_type st_dynamixel_state;
      _dynamixel_state_type * dynamixel_state;

    DynamixelStateList():
      dynamixel_state_length(0), st_dynamixel_state(), dynamixel_state(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->dynamixel_state_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->dynamixel_state_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->dynamixel_state_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->dynamixel_state_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dynamixel_state_length);
      for( uint32_t i = 0; i < dynamixel_state_length; i++){
      offset += this->dynamixel_state[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t dynamixel_state_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      dynamixel_state_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      dynamixel_state_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      dynamixel_state_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->dynamixel_state_length);
      if(dynamixel_state_lengthT > dynamixel_state_length)
        this->dynamixel_state = (dynamixel_workbench_msgs::DynamixelState*)realloc(this->dynamixel_state, dynamixel_state_lengthT * sizeof(dynamixel_workbench_msgs::DynamixelState));
      dynamixel_state_length = dynamixel_state_lengthT;
      for( uint32_t i = 0; i < dynamixel_state_length; i++){
      offset += this->st_dynamixel_state.deserialize(inbuffer + offset);
        memcpy( &(this->dynamixel_state[i]), &(this->st_dynamixel_state), sizeof(dynamixel_workbench_msgs::DynamixelState));
      }
     return offset;
    }

    virtual const char * getType() override { return "dynamixel_workbench_msgs/DynamixelStateList"; };
    virtual const char * getMD5() override { return "52c1af7cbc10f50d2e78a86519b081ee"; };

  };

}
#endif
