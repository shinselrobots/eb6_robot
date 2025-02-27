#ifndef _ROS_SERVICE_SetTorqueLimit_h
#define _ROS_SERVICE_SetTorqueLimit_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dynamixel_controllers
{

static const char SETTORQUELIMIT[] = "dynamixel_controllers/SetTorqueLimit";

  class SetTorqueLimitRequest : public ros::Msg
  {
    public:
      typedef float _torque_limit_type;
      _torque_limit_type torque_limit;

    SetTorqueLimitRequest():
      torque_limit(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->torque_limit);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->torque_limit));
     return offset;
    }

    virtual const char * getType() override { return SETTORQUELIMIT; };
    virtual const char * getMD5() override { return "7ac67170532bb79d95db2a425915bbd6"; };

  };

  class SetTorqueLimitResponse : public ros::Msg
  {
    public:

    SetTorqueLimitResponse()
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

    virtual const char * getType() override { return SETTORQUELIMIT; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SetTorqueLimit {
    public:
    typedef SetTorqueLimitRequest Request;
    typedef SetTorqueLimitResponse Response;
  };

}
#endif
