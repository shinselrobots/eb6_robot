#ifndef _ROS_SERVICE_SetSpeed_h
#define _ROS_SERVICE_SetSpeed_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dynamixel_controllers
{

static const char SETSPEED[] = "dynamixel_controllers/SetSpeed";

  class SetSpeedRequest : public ros::Msg
  {
    public:
      typedef float _speed_type;
      _speed_type speed;

    SetSpeedRequest():
      speed(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->speed);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->speed));
     return offset;
    }

    virtual const char * getType() override { return SETSPEED; };
    virtual const char * getMD5() override { return "4641bb0523a3557209606d9bd91ce33a"; };

  };

  class SetSpeedResponse : public ros::Msg
  {
    public:

    SetSpeedResponse()
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

    virtual const char * getType() override { return SETSPEED; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SetSpeed {
    public:
    typedef SetSpeedRequest Request;
    typedef SetSpeedResponse Response;
  };

}
#endif
