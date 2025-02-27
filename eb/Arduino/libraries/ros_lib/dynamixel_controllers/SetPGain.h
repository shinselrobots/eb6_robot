#ifndef _ROS_SERVICE_SetPGain_h
#define _ROS_SERVICE_SetPGain_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dynamixel_controllers
{

static const char SETPGAIN[] = "dynamixel_controllers/SetPGain";

  class SetPGainRequest : public ros::Msg
  {
    public:
      typedef uint8_t _p_gain_type;
      _p_gain_type p_gain;

    SetPGainRequest():
      p_gain(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->p_gain >> (8 * 0)) & 0xFF;
      offset += sizeof(this->p_gain);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->p_gain =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->p_gain);
     return offset;
    }

    virtual const char * getType() override { return SETPGAIN; };
    virtual const char * getMD5() override { return "877d7d04bebf9b2008ade5f81fd04f6d"; };

  };

  class SetPGainResponse : public ros::Msg
  {
    public:

    SetPGainResponse()
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

    virtual const char * getType() override { return SETPGAIN; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SetPGain {
    public:
    typedef SetPGainRequest Request;
    typedef SetPGainResponse Response;
  };

}
#endif
