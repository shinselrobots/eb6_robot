#ifndef _ROS_behavior_msgs_CommandState_h
#define _ROS_behavior_msgs_CommandState_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace behavior_msgs
{

  class CommandState : public ros::Msg
  {
    public:
      typedef const char* _commandState_type;
      _commandState_type commandState;
      typedef const char* _param1_type;
      _param1_type param1;
      typedef const char* _param2_type;
      _param2_type param2;

    CommandState():
      commandState(""),
      param1(""),
      param2("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_commandState = strlen(this->commandState);
      varToArr(outbuffer + offset, length_commandState);
      offset += 4;
      memcpy(outbuffer + offset, this->commandState, length_commandState);
      offset += length_commandState;
      uint32_t length_param1 = strlen(this->param1);
      varToArr(outbuffer + offset, length_param1);
      offset += 4;
      memcpy(outbuffer + offset, this->param1, length_param1);
      offset += length_param1;
      uint32_t length_param2 = strlen(this->param2);
      varToArr(outbuffer + offset, length_param2);
      offset += 4;
      memcpy(outbuffer + offset, this->param2, length_param2);
      offset += length_param2;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_commandState;
      arrToVar(length_commandState, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_commandState; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_commandState-1]=0;
      this->commandState = (char *)(inbuffer + offset-1);
      offset += length_commandState;
      uint32_t length_param1;
      arrToVar(length_param1, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_param1; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_param1-1]=0;
      this->param1 = (char *)(inbuffer + offset-1);
      offset += length_param1;
      uint32_t length_param2;
      arrToVar(length_param2, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_param2; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_param2-1]=0;
      this->param2 = (char *)(inbuffer + offset-1);
      offset += length_param2;
     return offset;
    }

    virtual const char * getType() override { return "behavior_msgs/CommandState"; };
    virtual const char * getMD5() override { return "8e24e1d61a82f87a1c704bf03f79ce90"; };

  };

}
#endif
