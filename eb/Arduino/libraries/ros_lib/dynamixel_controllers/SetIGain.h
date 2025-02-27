#ifndef _ROS_SERVICE_SetIGain_h
#define _ROS_SERVICE_SetIGain_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dynamixel_controllers
{

static const char SETIGAIN[] = "dynamixel_controllers/SetIGain";

  class SetIGainRequest : public ros::Msg
  {
    public:
      typedef uint8_t _i_gain_type;
      _i_gain_type i_gain;

    SetIGainRequest():
      i_gain(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->i_gain >> (8 * 0)) & 0xFF;
      offset += sizeof(this->i_gain);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->i_gain =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->i_gain);
     return offset;
    }

    virtual const char * getType() override { return SETIGAIN; };
    virtual const char * getMD5() override { return "05cc07cc9106270360ef709bb430891a"; };

  };

  class SetIGainResponse : public ros::Msg
  {
    public:

    SetIGainResponse()
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

    virtual const char * getType() override { return SETIGAIN; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SetIGain {
    public:
    typedef SetIGainRequest Request;
    typedef SetIGainResponse Response;
  };

}
#endif
