#ifndef _ROS_SERVICE_TorqueEnable_h
#define _ROS_SERVICE_TorqueEnable_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dynamixel_controllers
{

static const char TORQUEENABLE[] = "dynamixel_controllers/TorqueEnable";

  class TorqueEnableRequest : public ros::Msg
  {
    public:
      typedef bool _torque_enable_type;
      _torque_enable_type torque_enable;

    TorqueEnableRequest():
      torque_enable(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_torque_enable;
      u_torque_enable.real = this->torque_enable;
      *(outbuffer + offset + 0) = (u_torque_enable.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->torque_enable);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_torque_enable;
      u_torque_enable.base = 0;
      u_torque_enable.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->torque_enable = u_torque_enable.real;
      offset += sizeof(this->torque_enable);
     return offset;
    }

    virtual const char * getType() override { return TORQUEENABLE; };
    virtual const char * getMD5() override { return "e44dc96db32bd58b5a896c2c5bf316d0"; };

  };

  class TorqueEnableResponse : public ros::Msg
  {
    public:

    TorqueEnableResponse()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
     return offset;
    }

    virtual const char * getType() override { return TORQUEENABLE; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class TorqueEnable {
    public:
    typedef TorqueEnableRequest Request;
    typedef TorqueEnableResponse Response;
  };

}
#endif
