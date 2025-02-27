#ifndef _ROS_SERVICE_DynamixelCommand_h
#define _ROS_SERVICE_DynamixelCommand_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dynamixel_workbench_msgs
{

static const char DYNAMIXELCOMMAND[] = "dynamixel_workbench_msgs/DynamixelCommand";

  class DynamixelCommandRequest : public ros::Msg
  {
    public:
      typedef const char* _command_type;
      _command_type command;
      typedef uint8_t _id_type;
      _id_type id;
      typedef const char* _addr_name_type;
      _addr_name_type addr_name;
      typedef int32_t _value_type;
      _value_type value;

    DynamixelCommandRequest():
      command(""),
      id(0),
      addr_name(""),
      value(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_command = strlen(this->command);
      varToArr(outbuffer + offset, length_command);
      offset += 4;
      memcpy(outbuffer + offset, this->command, length_command);
      offset += length_command;
      *(outbuffer + offset + 0) = (this->id >> (8 * 0)) & 0xFF;
      offset += sizeof(this->id);
      uint32_t length_addr_name = strlen(this->addr_name);
      varToArr(outbuffer + offset, length_addr_name);
      offset += 4;
      memcpy(outbuffer + offset, this->addr_name, length_addr_name);
      offset += length_addr_name;
      union {
        int32_t real;
        uint32_t base;
      } u_value;
      u_value.real = this->value;
      *(outbuffer + offset + 0) = (u_value.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_value.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_value.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_value.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->value);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_command;
      arrToVar(length_command, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_command; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_command-1]=0;
      this->command = (char *)(inbuffer + offset-1);
      offset += length_command;
      this->id =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->id);
      uint32_t length_addr_name;
      arrToVar(length_addr_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_addr_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_addr_name-1]=0;
      this->addr_name = (char *)(inbuffer + offset-1);
      offset += length_addr_name;
      union {
        int32_t real;
        uint32_t base;
      } u_value;
      u_value.base = 0;
      u_value.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_value.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_value.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_value.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->value = u_value.real;
      offset += sizeof(this->value);
     return offset;
    }

    virtual const char * getType() override { return DYNAMIXELCOMMAND; };
    virtual const char * getMD5() override { return "567da0de54717bf69d417bbe8d582ee9"; };

  };

  class DynamixelCommandResponse : public ros::Msg
  {
    public:
      typedef bool _comm_result_type;
      _comm_result_type comm_result;

    DynamixelCommandResponse():
      comm_result(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_comm_result;
      u_comm_result.real = this->comm_result;
      *(outbuffer + offset + 0) = (u_comm_result.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->comm_result);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_comm_result;
      u_comm_result.base = 0;
      u_comm_result.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->comm_result = u_comm_result.real;
      offset += sizeof(this->comm_result);
     return offset;
    }

    virtual const char * getType() override { return DYNAMIXELCOMMAND; };
    virtual const char * getMD5() override { return "f12f56e0367ef7d42085bd5f9c478576"; };

  };

  class DynamixelCommand {
    public:
    typedef DynamixelCommandRequest Request;
    typedef DynamixelCommandResponse Response;
  };

}
#endif
