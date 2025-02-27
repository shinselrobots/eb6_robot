#ifndef _ROS_dynamixel_workbench_msgs_DynamixelLoadInfo_h
#define _ROS_dynamixel_workbench_msgs_DynamixelLoadInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dynamixel_workbench_msgs
{

  class DynamixelLoadInfo : public ros::Msg
  {
    public:
      typedef const char* _device_name_type;
      _device_name_type device_name;
      typedef uint64_t _baud_rate_type;
      _baud_rate_type baud_rate;
      typedef float _protocol_version_type;
      _protocol_version_type protocol_version;

    DynamixelLoadInfo():
      device_name(""),
      baud_rate(0),
      protocol_version(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_device_name = strlen(this->device_name);
      varToArr(outbuffer + offset, length_device_name);
      offset += 4;
      memcpy(outbuffer + offset, this->device_name, length_device_name);
      offset += length_device_name;
      *(outbuffer + offset + 0) = (this->baud_rate >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->baud_rate >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->baud_rate >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->baud_rate >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (this->baud_rate >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (this->baud_rate >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (this->baud_rate >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (this->baud_rate >> (8 * 7)) & 0xFF;
      offset += sizeof(this->baud_rate);
      union {
        float real;
        uint32_t base;
      } u_protocol_version;
      u_protocol_version.real = this->protocol_version;
      *(outbuffer + offset + 0) = (u_protocol_version.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_protocol_version.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_protocol_version.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_protocol_version.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->protocol_version);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_device_name;
      arrToVar(length_device_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_device_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_device_name-1]=0;
      this->device_name = (char *)(inbuffer + offset-1);
      offset += length_device_name;
      this->baud_rate =  ((uint64_t) (*(inbuffer + offset)));
      this->baud_rate |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->baud_rate |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->baud_rate |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->baud_rate |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      this->baud_rate |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      this->baud_rate |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      this->baud_rate |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      offset += sizeof(this->baud_rate);
      union {
        float real;
        uint32_t base;
      } u_protocol_version;
      u_protocol_version.base = 0;
      u_protocol_version.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_protocol_version.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_protocol_version.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_protocol_version.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->protocol_version = u_protocol_version.real;
      offset += sizeof(this->protocol_version);
     return offset;
    }

    virtual const char * getType() override { return "dynamixel_workbench_msgs/DynamixelLoadInfo"; };
    virtual const char * getMD5() override { return "af24d77e0b8329be8d17a782ba80f5e2"; };

  };

}
#endif
