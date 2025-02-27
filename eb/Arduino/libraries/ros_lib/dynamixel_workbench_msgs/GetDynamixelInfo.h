#ifndef _ROS_SERVICE_GetDynamixelInfo_h
#define _ROS_SERVICE_GetDynamixelInfo_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "dynamixel_workbench_msgs/DynamixelInfo.h"

namespace dynamixel_workbench_msgs
{

static const char GETDYNAMIXELINFO[] = "dynamixel_workbench_msgs/GetDynamixelInfo";

  class GetDynamixelInfoRequest : public ros::Msg
  {
    public:

    GetDynamixelInfoRequest()
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

    virtual const char * getType() override { return GETDYNAMIXELINFO; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class GetDynamixelInfoResponse : public ros::Msg
  {
    public:
      typedef dynamixel_workbench_msgs::DynamixelInfo _dynamixel_info_type;
      _dynamixel_info_type dynamixel_info;

    GetDynamixelInfoResponse():
      dynamixel_info()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->dynamixel_info.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->dynamixel_info.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return GETDYNAMIXELINFO; };
    virtual const char * getMD5() override { return "5075be00278efc6e6ba91f48b43afb53"; };

  };

  class GetDynamixelInfo {
    public:
    typedef GetDynamixelInfoRequest Request;
    typedef GetDynamixelInfoResponse Response;
  };

}
#endif
