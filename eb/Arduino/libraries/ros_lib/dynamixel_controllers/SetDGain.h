#ifndef _ROS_SERVICE_SetDGain_h
#define _ROS_SERVICE_SetDGain_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dynamixel_controllers
{

static const char SETDGAIN[] = "dynamixel_controllers/SetDGain";

  class SetDGainRequest : public ros::Msg
  {
    public:
      typedef uint8_t _d_gain_type;
      _d_gain_type d_gain;

    SetDGainRequest():
      d_gain(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->d_gain >> (8 * 0)) & 0xFF;
      offset += sizeof(this->d_gain);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->d_gain =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->d_gain);
     return offset;
    }

    virtual const char * getType() override { return SETDGAIN; };
    virtual const char * getMD5() override { return "e11ea9e5dc42b9fc71a6ea92affe961a"; };

  };

  class SetDGainResponse : public ros::Msg
  {
    public:

    SetDGainResponse()
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

    virtual const char * getType() override { return SETDGAIN; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SetDGain {
    public:
    typedef SetDGainRequest Request;
    typedef SetDGainResponse Response;
  };

}
#endif
