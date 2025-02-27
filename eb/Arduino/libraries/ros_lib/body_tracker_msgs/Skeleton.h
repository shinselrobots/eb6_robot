#ifndef _ROS_body_tracker_msgs_Skeleton_h
#define _ROS_body_tracker_msgs_Skeleton_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "geometry_msgs/Point32.h"

namespace body_tracker_msgs
{

  class Skeleton : public ros::Msg
  {
    public:
      typedef int32_t _body_id_type;
      _body_id_type body_id;
      typedef int32_t _tracking_status_type;
      _tracking_status_type tracking_status;
      typedef int32_t _gesture_type;
      _gesture_type gesture;
      typedef geometry_msgs::Point32 _position2D_type;
      _position2D_type position2D;
      typedef geometry_msgs::Point32 _centerOfMass_type;
      _centerOfMass_type centerOfMass;
      typedef geometry_msgs::Point32 _joint_position_head_type;
      _joint_position_head_type joint_position_head;
      typedef geometry_msgs::Point32 _joint_position_neck_type;
      _joint_position_neck_type joint_position_neck;
      typedef geometry_msgs::Point32 _joint_position_shoulder_type;
      _joint_position_shoulder_type joint_position_shoulder;
      typedef geometry_msgs::Point32 _joint_position_spine_top_type;
      _joint_position_spine_top_type joint_position_spine_top;
      typedef geometry_msgs::Point32 _joint_position_spine_mid_type;
      _joint_position_spine_mid_type joint_position_spine_mid;
      typedef geometry_msgs::Point32 _joint_position_spine_bottom_type;
      _joint_position_spine_bottom_type joint_position_spine_bottom;
      typedef geometry_msgs::Point32 _joint_position_left_shoulder_type;
      _joint_position_left_shoulder_type joint_position_left_shoulder;
      typedef geometry_msgs::Point32 _joint_position_left_elbow_type;
      _joint_position_left_elbow_type joint_position_left_elbow;
      typedef geometry_msgs::Point32 _joint_position_left_hand_type;
      _joint_position_left_hand_type joint_position_left_hand;
      typedef geometry_msgs::Point32 _joint_position_right_shoulder_type;
      _joint_position_right_shoulder_type joint_position_right_shoulder;
      typedef geometry_msgs::Point32 _joint_position_right_elbow_type;
      _joint_position_right_elbow_type joint_position_right_elbow;
      typedef geometry_msgs::Point32 _joint_position_right_hand_type;
      _joint_position_right_hand_type joint_position_right_hand;

    Skeleton():
      body_id(0),
      tracking_status(0),
      gesture(0),
      position2D(),
      centerOfMass(),
      joint_position_head(),
      joint_position_neck(),
      joint_position_shoulder(),
      joint_position_spine_top(),
      joint_position_spine_mid(),
      joint_position_spine_bottom(),
      joint_position_left_shoulder(),
      joint_position_left_elbow(),
      joint_position_left_hand(),
      joint_position_right_shoulder(),
      joint_position_right_elbow(),
      joint_position_right_hand()
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
      offset += this->position2D.serialize(outbuffer + offset);
      offset += this->centerOfMass.serialize(outbuffer + offset);
      offset += this->joint_position_head.serialize(outbuffer + offset);
      offset += this->joint_position_neck.serialize(outbuffer + offset);
      offset += this->joint_position_shoulder.serialize(outbuffer + offset);
      offset += this->joint_position_spine_top.serialize(outbuffer + offset);
      offset += this->joint_position_spine_mid.serialize(outbuffer + offset);
      offset += this->joint_position_spine_bottom.serialize(outbuffer + offset);
      offset += this->joint_position_left_shoulder.serialize(outbuffer + offset);
      offset += this->joint_position_left_elbow.serialize(outbuffer + offset);
      offset += this->joint_position_left_hand.serialize(outbuffer + offset);
      offset += this->joint_position_right_shoulder.serialize(outbuffer + offset);
      offset += this->joint_position_right_elbow.serialize(outbuffer + offset);
      offset += this->joint_position_right_hand.serialize(outbuffer + offset);
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
      offset += this->position2D.deserialize(inbuffer + offset);
      offset += this->centerOfMass.deserialize(inbuffer + offset);
      offset += this->joint_position_head.deserialize(inbuffer + offset);
      offset += this->joint_position_neck.deserialize(inbuffer + offset);
      offset += this->joint_position_shoulder.deserialize(inbuffer + offset);
      offset += this->joint_position_spine_top.deserialize(inbuffer + offset);
      offset += this->joint_position_spine_mid.deserialize(inbuffer + offset);
      offset += this->joint_position_spine_bottom.deserialize(inbuffer + offset);
      offset += this->joint_position_left_shoulder.deserialize(inbuffer + offset);
      offset += this->joint_position_left_elbow.deserialize(inbuffer + offset);
      offset += this->joint_position_left_hand.deserialize(inbuffer + offset);
      offset += this->joint_position_right_shoulder.deserialize(inbuffer + offset);
      offset += this->joint_position_right_elbow.deserialize(inbuffer + offset);
      offset += this->joint_position_right_hand.deserialize(inbuffer + offset);
     return offset;
    }

    virtual const char * getType() override { return "body_tracker_msgs/Skeleton"; };
    virtual const char * getMD5() override { return "3ccf81ce06b8e4b357ba011ca33898c6"; };

  };

}
#endif
