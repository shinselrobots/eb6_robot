#ifndef _ROS_dynamixel_workbench_msgs_DynamixelState_h
#define _ROS_dynamixel_workbench_msgs_DynamixelState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dynamixel_workbench_msgs
{

  class DynamixelState : public ros::Msg
  {
    public:
      typedef const char* _name_type;
      _name_type name;
      typedef uint8_t _id_type;
      _id_type id;
      typedef int32_t _present_position_type;
      _present_position_type present_position;
      typedef int32_t _present_velocity_type;
      _present_velocity_type present_velocity;
      typedef int16_t _present_current_type;
      _present_current_type present_current;

    DynamixelState():
      name(""),
      id(0),
      present_position(0),
      present_velocity(0),
      present_current(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      *(outbuffer + offset + 0) = (this->id >> (8 * 0)) & 0xFF;
      offset += sizeof(this->id);
      union {
        int32_t real;
        uint32_t base;
      } u_present_position;
      u_present_position.real = this->present_position;
      *(outbuffer + offset + 0) = (u_present_position.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_present_position.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_present_position.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_present_position.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->present_position);
      union {
        int32_t real;
        uint32_t base;
      } u_present_velocity;
      u_present_velocity.real = this->present_velocity;
      *(outbuffer + offset + 0) = (u_present_velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_present_velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_present_velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_present_velocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->present_velocity);
      union {
        int16_t real;
        uint16_t base;
      } u_present_current;
      u_present_current.real = this->present_current;
      *(outbuffer + offset + 0) = (u_present_current.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_present_current.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->present_current);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      this->id =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->id);
      union {
        int32_t real;
        uint32_t base;
      } u_present_position;
      u_present_position.base = 0;
      u_present_position.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_present_position.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_present_position.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_present_position.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->present_position = u_present_position.real;
      offset += sizeof(this->present_position);
      union {
        int32_t real;
        uint32_t base;
      } u_present_velocity;
      u_present_velocity.base = 0;
      u_present_velocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_present_velocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_present_velocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_present_velocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->present_velocity = u_present_velocity.real;
      offset += sizeof(this->present_velocity);
      union {
        int16_t real;
        uint16_t base;
      } u_present_current;
      u_present_current.base = 0;
      u_present_current.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_present_current.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->present_current = u_present_current.real;
      offset += sizeof(this->present_current);
     return offset;
    }

    virtual const char * getType() override { return "dynamixel_workbench_msgs/DynamixelState"; };
    virtual const char * getMD5() override { return "c9a7ca26833f8d4ad5b00272b4226f36"; };

  };

}
#endif
