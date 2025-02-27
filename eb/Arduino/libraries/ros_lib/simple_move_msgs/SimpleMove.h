#ifndef _ROS_simple_move_msgs_SimpleMove_h
#define _ROS_simple_move_msgs_SimpleMove_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace simple_move_msgs
{

  class SimpleMove : public ros::Msg
  {
    public:
      typedef float _move_amount_type;
      _move_amount_type move_amount;
      typedef float _move_speed_type;
      _move_speed_type move_speed;
      typedef float _turn_amount_type;
      _turn_amount_type turn_amount;
      typedef float _turn_speed_type;
      _turn_speed_type turn_speed;

    SimpleMove():
      move_amount(0),
      move_speed(0),
      turn_amount(0),
      turn_speed(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      offset += serializeAvrFloat64(outbuffer + offset, this->move_amount);
      offset += serializeAvrFloat64(outbuffer + offset, this->move_speed);
      offset += serializeAvrFloat64(outbuffer + offset, this->turn_amount);
      offset += serializeAvrFloat64(outbuffer + offset, this->turn_speed);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->move_amount));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->move_speed));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->turn_amount));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->turn_speed));
     return offset;
    }

    virtual const char * getType() override { return "simple_move_msgs/SimpleMove"; };
    virtual const char * getMD5() override { return "6527725e4f5afda5752815d0fecbb483"; };

  };

}
#endif
