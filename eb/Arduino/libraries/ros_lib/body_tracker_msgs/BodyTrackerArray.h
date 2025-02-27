#ifndef _ROS_body_tracker_msgs_BodyTrackerArray_h
#define _ROS_body_tracker_msgs_BodyTrackerArray_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "std_msgs/Header.h"
#include "body_tracker_msgs/BodyTracker.h"

namespace body_tracker_msgs
{

  class BodyTrackerArray : public ros::Msg
  {
    public:
      typedef std_msgs::Header _header_type;
      _header_type header;
      uint32_t detected_list_length;
      typedef body_tracker_msgs::BodyTracker _detected_list_type;
      _detected_list_type st_detected_list;
      _detected_list_type * detected_list;

    BodyTrackerArray():
      header(),
      detected_list_length(0), st_detected_list(), detected_list(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += this->header.serialize(outbuffer + offset);
      *(outbuffer + offset + 0) = (this->detected_list_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->detected_list_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->detected_list_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->detected_list_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->detected_list_length);
      for( uint32_t i = 0; i < detected_list_length; i++){
      offset += this->detected_list[i].serialize(outbuffer + offset);
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += this->header.deserialize(inbuffer + offset);
      uint32_t detected_list_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      detected_list_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      detected_list_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      detected_list_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->detected_list_length);
      if(detected_list_lengthT > detected_list_length)
        this->detected_list = (body_tracker_msgs::BodyTracker*)realloc(this->detected_list, detected_list_lengthT * sizeof(body_tracker_msgs::BodyTracker));
      detected_list_length = detected_list_lengthT;
      for( uint32_t i = 0; i < detected_list_length; i++){
      offset += this->st_detected_list.deserialize(inbuffer + offset);
        memcpy( &(this->detected_list[i]), &(this->st_detected_list), sizeof(body_tracker_msgs::BodyTracker));
      }
     return offset;
    }

    virtual const char * getType() override { return "body_tracker_msgs/BodyTrackerArray"; };
    virtual const char * getMD5() override { return "6d7bf9d3e9e82b51ffb599a4fb81f5dd"; };

  };

}
#endif
