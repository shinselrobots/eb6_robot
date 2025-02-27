#ifndef _ROS_SERVICE_SetCompliancePunch_h
#define _ROS_SERVICE_SetCompliancePunch_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dynamixel_controllers
{

static const char SETCOMPLIANCEPUNCH[] = "dynamixel_controllers/SetCompliancePunch";

  class SetCompliancePunchRequest : public ros::Msg
  {
    public:
      typedef uint8_t _punch_type;
      _punch_type punch;

    SetCompliancePunchRequest():
      punch(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->punch >> (8 * 0)) & 0xFF;
      offset += sizeof(this->punch);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->punch =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->punch);
     return offset;
    }

    virtual const char * getType() override { return SETCOMPLIANCEPUNCH; };
    virtual const char * getMD5() override { return "6f1db06d3f143058321215213d1c3bef"; };

  };

  class SetCompliancePunchResponse : public ros::Msg
  {
    public:

    SetCompliancePunchResponse()
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

    virtual const char * getType() override { return SETCOMPLIANCEPUNCH; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class SetCompliancePunch {
    public:
    typedef SetCompliancePunchRequest Request;
    typedef SetCompliancePunchResponse Response;
  };

}
#endif
