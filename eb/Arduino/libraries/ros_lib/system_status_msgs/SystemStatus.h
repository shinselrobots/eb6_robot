#ifndef _ROS_system_status_msgs_SystemStatus_h
#define _ROS_system_status_msgs_SystemStatus_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace system_status_msgs
{

  class SystemStatus : public ros::Msg
  {
    public:
      typedef const char* _item_type;
      _item_type item;
      typedef const char* _status_type;
      _status_type status;

    SystemStatus():
      item(""),
      status("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_item = strlen(this->item);
      varToArr(outbuffer + offset, length_item);
      offset += 4;
      memcpy(outbuffer + offset, this->item, length_item);
      offset += length_item;
      uint32_t length_status = strlen(this->status);
      varToArr(outbuffer + offset, length_status);
      offset += 4;
      memcpy(outbuffer + offset, this->status, length_status);
      offset += length_status;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_item;
      arrToVar(length_item, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_item; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_item-1]=0;
      this->item = (char *)(inbuffer + offset-1);
      offset += length_item;
      uint32_t length_status;
      arrToVar(length_status, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_status; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_status-1]=0;
      this->status = (char *)(inbuffer + offset-1);
      offset += length_status;
     return offset;
    }

    virtual const char * getType() override { return "system_status_msgs/SystemStatus"; };
    virtual const char * getMD5() override { return "1f7f70b26787bf2b13ec47c3ae516999"; };

  };

}
#endif
