#ifndef _ROS_dynamixel_workbench_msgs_DynamixelInfo_h
#define _ROS_dynamixel_workbench_msgs_DynamixelInfo_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "dynamixel_workbench_msgs/DynamixelLoadInfo.h"

namespace dynamixel_workbench_msgs
{

  class DynamixelInfo : public ros::Msg
  {
    public:
      typedef dynamixel_workbench_msgs::DynamixelLoadInfo _load_info_type;
      _load_info_type load_info;
      typedef const char* _model_name_type;
      _model_name_type model_name;
      typedef uint16_t _model_number_type;
      _model_number_type model_number;
      typedef uint8_t _model_id_type;
      _model_id_type model_id;

    DynamixelInfo():
      load_info(),
      model_name(""),
      model_number(0),
      model_id(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->load_info.serialize(outbuffer + offset);
      uint32_t length_model_name = strlen(this->model_name);
      varToArr(outbuffer + offset, length_model_name);
      offset += 4;
      memcpy(outbuffer + offset, this->model_name, length_model_name);
      offset += length_model_name;
      *(outbuffer + offset + 0) = (this->model_number >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->model_number >> (8 * 1)) & 0xFF;
      offset += sizeof(this->model_number);
      *(outbuffer + offset + 0) = (this->model_id >> (8 * 0)) & 0xFF;
      offset += sizeof(this->model_id);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->load_info.deserialize(inbuffer + offset);
      uint32_t length_model_name;
      arrToVar(length_model_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_model_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_model_name-1]=0;
      this->model_name = (char *)(inbuffer + offset-1);
      offset += length_model_name;
      this->model_number =  ((uint16_t) (*(inbuffer + offset)));
      this->model_number |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->model_number);
      this->model_id =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->model_id);
     return offset;
    }

    virtual const char * getType() override { return "dynamixel_workbench_msgs/DynamixelInfo"; };
    virtual const char * getMD5() override { return "8c02a7626d88db402a8af7bc5e0880d4"; };

  };

}
#endif
