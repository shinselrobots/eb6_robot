#ifndef _ROS_body_tracker_msgs_BodyTracker_h
#define _ROS_body_tracker_msgs_BodyTracker_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Point32.h"

namespace body_tracker_msgs
{

  class BodyTracker : public ros::Msg
  {
    public:
      typedef int32_t _body_id_type;
      _body_id_type body_id;
      typedef int32_t _tracking_status_type;
      _tracking_status_type tracking_status;
      typedef int32_t _gesture_type;
      _gesture_type gesture;
      typedef bool _face_found_type;
      _face_found_type face_found;
      typedef int32_t _face_left_type;
      _face_left_type face_left;
      typedef int32_t _face_top_type;
      _face_top_type face_top;
      typedef int32_t _face_width_type;
      _face_width_type face_width;
      typedef int32_t _face_height_type;
      _face_height_type face_height;
      typedef int32_t _age_type;
      _age_type age;
      typedef int32_t _gender_type;
      _gender_type gender;
      typedef const char* _name_type;
      _name_type name;
      typedef float _servo_pan_type;
      _servo_pan_type servo_pan;
      typedef float _servo_tilt_type;
      _servo_tilt_type servo_tilt;
      typedef geometry_msgs::Point32 _position2d_type;
      _position2d_type position2d;
      typedef geometry_msgs::Point32 _position3d_type;
      _position3d_type position3d;
      typedef geometry_msgs::Point32 _face_center_type;
      _face_center_type face_center;

    BodyTracker():
      body_id(0),
      tracking_status(0),
      gesture(0),
      face_found(0),
      face_left(0),
      face_top(0),
      face_width(0),
      face_height(0),
      age(0),
      gender(0),
      name(""),
      servo_pan(0),
      servo_tilt(0),
      position2d(),
      position3d(),
      face_center()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_body_id;
      u_body_id.real = this->body_id;
      *(outbuffer + offset + 0) = (u_body_id.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_body_id.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_body_id.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_body_id.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->body_id);
      union {
        int32_t real;
        uint32_t base;
      } u_tracking_status;
      u_tracking_status.real = this->tracking_status;
      *(outbuffer + offset + 0) = (u_tracking_status.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_tracking_status.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_tracking_status.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_tracking_status.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->tracking_status);
      union {
        int32_t real;
        uint32_t base;
      } u_gesture;
      u_gesture.real = this->gesture;
      *(outbuffer + offset + 0) = (u_gesture.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gesture.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gesture.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gesture.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gesture);
      union {
        bool real;
        uint8_t base;
      } u_face_found;
      u_face_found.real = this->face_found;
      *(outbuffer + offset + 0) = (u_face_found.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->face_found);
      union {
        int32_t real;
        uint32_t base;
      } u_face_left;
      u_face_left.real = this->face_left;
      *(outbuffer + offset + 0) = (u_face_left.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_face_left.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_face_left.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_face_left.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->face_left);
      union {
        int32_t real;
        uint32_t base;
      } u_face_top;
      u_face_top.real = this->face_top;
      *(outbuffer + offset + 0) = (u_face_top.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_face_top.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_face_top.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_face_top.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->face_top);
      union {
        int32_t real;
        uint32_t base;
      } u_face_width;
      u_face_width.real = this->face_width;
      *(outbuffer + offset + 0) = (u_face_width.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_face_width.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_face_width.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_face_width.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->face_width);
      union {
        int32_t real;
        uint32_t base;
      } u_face_height;
      u_face_height.real = this->face_height;
      *(outbuffer + offset + 0) = (u_face_height.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_face_height.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_face_height.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_face_height.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->face_height);
      union {
        int32_t real;
        uint32_t base;
      } u_age;
      u_age.real = this->age;
      *(outbuffer + offset + 0) = (u_age.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_age.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_age.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_age.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->age);
      union {
        int32_t real;
        uint32_t base;
      } u_gender;
      u_gender.real = this->gender;
      *(outbuffer + offset + 0) = (u_gender.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_gender.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_gender.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_gender.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->gender);
      uint32_t length_name = strlen(this->name);
      varToArr(outbuffer + offset, length_name);
      offset += 4;
      memcpy(outbuffer + offset, this->name, length_name);
      offset += length_name;
      offset += serializeAvrFloat64(outbuffer + offset, this->servo_pan);
      offset += serializeAvrFloat64(outbuffer + offset, this->servo_tilt);
      offset += this->position2d.serialize(outbuffer + offset);
      offset += this->position3d.serialize(outbuffer + offset);
      offset += this->face_center.serialize(outbuffer + offset);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        int32_t real;
        uint32_t base;
      } u_body_id;
      u_body_id.base = 0;
      u_body_id.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_body_id.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_body_id.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_body_id.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->body_id = u_body_id.real;
      offset += sizeof(this->body_id);
      union {
        int32_t real;
        uint32_t base;
      } u_tracking_status;
      u_tracking_status.base = 0;
      u_tracking_status.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_tracking_status.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_tracking_status.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_tracking_status.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->tracking_status = u_tracking_status.real;
      offset += sizeof(this->tracking_status);
      union {
        int32_t real;
        uint32_t base;
      } u_gesture;
      u_gesture.base = 0;
      u_gesture.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gesture.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gesture.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gesture.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->gesture = u_gesture.real;
      offset += sizeof(this->gesture);
      union {
        bool real;
        uint8_t base;
      } u_face_found;
      u_face_found.base = 0;
      u_face_found.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->face_found = u_face_found.real;
      offset += sizeof(this->face_found);
      union {
        int32_t real;
        uint32_t base;
      } u_face_left;
      u_face_left.base = 0;
      u_face_left.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_face_left.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_face_left.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_face_left.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->face_left = u_face_left.real;
      offset += sizeof(this->face_left);
      union {
        int32_t real;
        uint32_t base;
      } u_face_top;
      u_face_top.base = 0;
      u_face_top.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_face_top.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_face_top.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_face_top.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->face_top = u_face_top.real;
      offset += sizeof(this->face_top);
      union {
        int32_t real;
        uint32_t base;
      } u_face_width;
      u_face_width.base = 0;
      u_face_width.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_face_width.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_face_width.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_face_width.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->face_width = u_face_width.real;
      offset += sizeof(this->face_width);
      union {
        int32_t real;
        uint32_t base;
      } u_face_height;
      u_face_height.base = 0;
      u_face_height.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_face_height.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_face_height.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_face_height.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->face_height = u_face_height.real;
      offset += sizeof(this->face_height);
      union {
        int32_t real;
        uint32_t base;
      } u_age;
      u_age.base = 0;
      u_age.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_age.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_age.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_age.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->age = u_age.real;
      offset += sizeof(this->age);
      union {
        int32_t real;
        uint32_t base;
      } u_gender;
      u_gender.base = 0;
      u_gender.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_gender.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_gender.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_gender.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->gender = u_gender.real;
      offset += sizeof(this->gender);
      uint32_t length_name;
      arrToVar(length_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_name-1]=0;
      this->name = (char *)(inbuffer + offset-1);
      offset += length_name;
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->servo_pan));
      offset += deserializeAvrFloat64(inbuffer + offset, &(this->servo_tilt));
      offset += this->position2d.deserialize(inbuffer + offset);
      offset += this->position3d.deserialize(inbuffer + offset);
      offset += this->face_center.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "body_tracker_msgs/BodyTracker"; };
    virtual const char * getMD5() override { return "a83c997463c9e099d05d4cafcef562f7"; };

  };

}
#endif
