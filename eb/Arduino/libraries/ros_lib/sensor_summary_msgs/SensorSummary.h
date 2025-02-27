#ifndef _ROS_sensor_summary_msgs_SensorSummary_h
#define _ROS_sensor_summary_msgs_SensorSummary_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace sensor_summary_msgs
{

  class SensorSummary : public ros::Msg
  {
    public:
      typedef float _nearest_object_front_type;
      _nearest_object_front_type nearest_object_front;
      typedef float _nearest_object_front_right_type;
      _nearest_object_front_right_type nearest_object_front_right;
      typedef float _nearest_object_front_left_type;
      _nearest_object_front_left_type nearest_object_front_left;
      typedef float _nearest_object_side_right_type;
      _nearest_object_side_right_type nearest_object_side_right;
      typedef float _nearest_object_side_left_type;
      _nearest_object_side_left_type nearest_object_side_left;
      typedef float _nearest_object_rear_type;
      _nearest_object_rear_type nearest_object_rear;
      typedef float _clear_path_gap_direction_type;
      _clear_path_gap_direction_type clear_path_gap_direction;
      typedef float _clear_path_gap_width_type;
      _clear_path_gap_width_type clear_path_gap_width;
      typedef float _avoidance_range_used_type;
      _avoidance_range_used_type avoidance_range_used;

    SensorSummary():
      nearest_object_front(0),
      nearest_object_front_right(0),
      nearest_object_front_left(0),
      nearest_object_side_right(0),
      nearest_object_side_left(0),
      nearest_object_rear(0),
      clear_path_gap_direction(0),
      clear_path_gap_width(0),
      avoidance_range_used(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_nearest_object_front;
      u_nearest_object_front.real = this->nearest_object_front;
      *(outbuffer + offset + 0) = (u_nearest_object_front.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_nearest_object_front.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_nearest_object_front.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_nearest_object_front.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->nearest_object_front);
      union {
        float real;
        uint32_t base;
      } u_nearest_object_front_right;
      u_nearest_object_front_right.real = this->nearest_object_front_right;
      *(outbuffer + offset + 0) = (u_nearest_object_front_right.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_nearest_object_front_right.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_nearest_object_front_right.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_nearest_object_front_right.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->nearest_object_front_right);
      union {
        float real;
        uint32_t base;
      } u_nearest_object_front_left;
      u_nearest_object_front_left.real = this->nearest_object_front_left;
      *(outbuffer + offset + 0) = (u_nearest_object_front_left.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_nearest_object_front_left.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_nearest_object_front_left.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_nearest_object_front_left.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->nearest_object_front_left);
      union {
        float real;
        uint32_t base;
      } u_nearest_object_side_right;
      u_nearest_object_side_right.real = this->nearest_object_side_right;
      *(outbuffer + offset + 0) = (u_nearest_object_side_right.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_nearest_object_side_right.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_nearest_object_side_right.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_nearest_object_side_right.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->nearest_object_side_right);
      union {
        float real;
        uint32_t base;
      } u_nearest_object_side_left;
      u_nearest_object_side_left.real = this->nearest_object_side_left;
      *(outbuffer + offset + 0) = (u_nearest_object_side_left.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_nearest_object_side_left.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_nearest_object_side_left.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_nearest_object_side_left.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->nearest_object_side_left);
      union {
        float real;
        uint32_t base;
      } u_nearest_object_rear;
      u_nearest_object_rear.real = this->nearest_object_rear;
      *(outbuffer + offset + 0) = (u_nearest_object_rear.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_nearest_object_rear.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_nearest_object_rear.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_nearest_object_rear.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->nearest_object_rear);
      union {
        float real;
        uint32_t base;
      } u_clear_path_gap_direction;
      u_clear_path_gap_direction.real = this->clear_path_gap_direction;
      *(outbuffer + offset + 0) = (u_clear_path_gap_direction.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_clear_path_gap_direction.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_clear_path_gap_direction.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_clear_path_gap_direction.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->clear_path_gap_direction);
      union {
        float real;
        uint32_t base;
      } u_clear_path_gap_width;
      u_clear_path_gap_width.real = this->clear_path_gap_width;
      *(outbuffer + offset + 0) = (u_clear_path_gap_width.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_clear_path_gap_width.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_clear_path_gap_width.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_clear_path_gap_width.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->clear_path_gap_width);
      union {
        float real;
        uint32_t base;
      } u_avoidance_range_used;
      u_avoidance_range_used.real = this->avoidance_range_used;
      *(outbuffer + offset + 0) = (u_avoidance_range_used.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_avoidance_range_used.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_avoidance_range_used.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_avoidance_range_used.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->avoidance_range_used);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        float real;
        uint32_t base;
      } u_nearest_object_front;
      u_nearest_object_front.base = 0;
      u_nearest_object_front.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_nearest_object_front.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_nearest_object_front.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_nearest_object_front.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->nearest_object_front = u_nearest_object_front.real;
      offset += sizeof(this->nearest_object_front);
      union {
        float real;
        uint32_t base;
      } u_nearest_object_front_right;
      u_nearest_object_front_right.base = 0;
      u_nearest_object_front_right.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_nearest_object_front_right.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_nearest_object_front_right.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_nearest_object_front_right.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->nearest_object_front_right = u_nearest_object_front_right.real;
      offset += sizeof(this->nearest_object_front_right);
      union {
        float real;
        uint32_t base;
      } u_nearest_object_front_left;
      u_nearest_object_front_left.base = 0;
      u_nearest_object_front_left.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_nearest_object_front_left.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_nearest_object_front_left.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_nearest_object_front_left.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->nearest_object_front_left = u_nearest_object_front_left.real;
      offset += sizeof(this->nearest_object_front_left);
      union {
        float real;
        uint32_t base;
      } u_nearest_object_side_right;
      u_nearest_object_side_right.base = 0;
      u_nearest_object_side_right.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_nearest_object_side_right.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_nearest_object_side_right.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_nearest_object_side_right.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->nearest_object_side_right = u_nearest_object_side_right.real;
      offset += sizeof(this->nearest_object_side_right);
      union {
        float real;
        uint32_t base;
      } u_nearest_object_side_left;
      u_nearest_object_side_left.base = 0;
      u_nearest_object_side_left.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_nearest_object_side_left.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_nearest_object_side_left.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_nearest_object_side_left.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->nearest_object_side_left = u_nearest_object_side_left.real;
      offset += sizeof(this->nearest_object_side_left);
      union {
        float real;
        uint32_t base;
      } u_nearest_object_rear;
      u_nearest_object_rear.base = 0;
      u_nearest_object_rear.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_nearest_object_rear.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_nearest_object_rear.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_nearest_object_rear.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->nearest_object_rear = u_nearest_object_rear.real;
      offset += sizeof(this->nearest_object_rear);
      union {
        float real;
        uint32_t base;
      } u_clear_path_gap_direction;
      u_clear_path_gap_direction.base = 0;
      u_clear_path_gap_direction.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_clear_path_gap_direction.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_clear_path_gap_direction.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_clear_path_gap_direction.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->clear_path_gap_direction = u_clear_path_gap_direction.real;
      offset += sizeof(this->clear_path_gap_direction);
      union {
        float real;
        uint32_t base;
      } u_clear_path_gap_width;
      u_clear_path_gap_width.base = 0;
      u_clear_path_gap_width.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_clear_path_gap_width.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_clear_path_gap_width.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_clear_path_gap_width.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->clear_path_gap_width = u_clear_path_gap_width.real;
      offset += sizeof(this->clear_path_gap_width);
      union {
        float real;
        uint32_t base;
      } u_avoidance_range_used;
      u_avoidance_range_used.base = 0;
      u_avoidance_range_used.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_avoidance_range_used.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_avoidance_range_used.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_avoidance_range_used.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->avoidance_range_used = u_avoidance_range_used.real;
      offset += sizeof(this->avoidance_range_used);
     return offset;
    }

    virtual const char * getType() override { return "sensor_summary_msgs/SensorSummary"; };
    virtual const char * getMD5() override { return "f016447f8c5cf207abc4e70c9ff8112d"; };

  };

}
#endif
