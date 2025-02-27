#ifndef _ROS_dynamixel_workbench_msgs_PROExt_h
#define _ROS_dynamixel_workbench_msgs_PROExt_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dynamixel_workbench_msgs
{

  class PROExt : public ros::Msg
  {
    public:
      typedef uint16_t _Model_Number_type;
      _Model_Number_type Model_Number;
      typedef uint8_t _Firmware_Version_type;
      _Firmware_Version_type Firmware_Version;
      typedef uint8_t _ID_type;
      _ID_type ID;
      typedef uint8_t _Baud_Rate_type;
      _Baud_Rate_type Baud_Rate;
      typedef uint8_t _Return_Delay_Time_type;
      _Return_Delay_Time_type Return_Delay_Time;
      typedef uint8_t _Operating_Mode_type;
      _Operating_Mode_type Operating_Mode;
      typedef int32_t _Homing_Offset_type;
      _Homing_Offset_type Homing_Offset;
      typedef uint32_t _Moving_Threshold_type;
      _Moving_Threshold_type Moving_Threshold;
      typedef uint8_t _Temperature_Limit_type;
      _Temperature_Limit_type Temperature_Limit;
      typedef uint16_t _Max_Voltage_Limit_type;
      _Max_Voltage_Limit_type Max_Voltage_Limit;
      typedef uint16_t _Min_Voltage_Limit_type;
      _Min_Voltage_Limit_type Min_Voltage_Limit;
      typedef uint32_t _Acceleration_Limit_type;
      _Acceleration_Limit_type Acceleration_Limit;
      typedef uint16_t _Torque_Limit_type;
      _Torque_Limit_type Torque_Limit;
      typedef uint32_t _Velocity_Limit_type;
      _Velocity_Limit_type Velocity_Limit;
      typedef int32_t _Max_Position_Limit_type;
      _Max_Position_Limit_type Max_Position_Limit;
      typedef int32_t _Min_Position_Limit_type;
      _Min_Position_Limit_type Min_Position_Limit;
      typedef uint8_t _External_Port_Mode_1_type;
      _External_Port_Mode_1_type External_Port_Mode_1;
      typedef uint8_t _External_Port_Mode_2_type;
      _External_Port_Mode_2_type External_Port_Mode_2;
      typedef uint8_t _External_Port_Mode_3_type;
      _External_Port_Mode_3_type External_Port_Mode_3;
      typedef uint8_t _External_Port_Mode_4_type;
      _External_Port_Mode_4_type External_Port_Mode_4;
      typedef uint8_t _Shutdown_type;
      _Shutdown_type Shutdown;
      typedef uint8_t _Torque_Enable_type;
      _Torque_Enable_type Torque_Enable;
      typedef uint8_t _LED_RED_type;
      _LED_RED_type LED_RED;
      typedef uint8_t _LED_GREEN_type;
      _LED_GREEN_type LED_GREEN;
      typedef uint8_t _LED_BLUE_type;
      _LED_BLUE_type LED_BLUE;
      typedef uint16_t _Velocity_I_Gain_type;
      _Velocity_I_Gain_type Velocity_I_Gain;
      typedef uint16_t _Velocity_P_Gain_type;
      _Velocity_P_Gain_type Velocity_P_Gain;
      typedef uint16_t _Position_P_Gain_type;
      _Position_P_Gain_type Position_P_Gain;
      typedef int32_t _Goal_Position_type;
      _Goal_Position_type Goal_Position;
      typedef uint32_t _Goal_Velocity_type;
      _Goal_Velocity_type Goal_Velocity;
      typedef uint16_t _Goal_Torque_type;
      _Goal_Torque_type Goal_Torque;
      typedef int32_t _Goal_Acceleration_type;
      _Goal_Acceleration_type Goal_Acceleration;
      typedef uint8_t _Moving_type;
      _Moving_type Moving;
      typedef int32_t _Present_Position_type;
      _Present_Position_type Present_Position;
      typedef uint32_t _Present_Velocity_type;
      _Present_Velocity_type Present_Velocity;
      typedef uint16_t _Present_Current_type;
      _Present_Current_type Present_Current;
      typedef uint16_t _Present_Input_Voltage_type;
      _Present_Input_Voltage_type Present_Input_Voltage;
      typedef uint8_t _Present_Temperature_type;
      _Present_Temperature_type Present_Temperature;
      typedef uint8_t _Registered_Instruction_type;
      _Registered_Instruction_type Registered_Instruction;
      typedef uint8_t _Status_Return_Level_type;
      _Status_Return_Level_type Status_Return_Level;
      typedef uint8_t _Hardware_Error_Status_type;
      _Hardware_Error_Status_type Hardware_Error_Status;

    PROExt():
      Model_Number(0),
      Firmware_Version(0),
      ID(0),
      Baud_Rate(0),
      Return_Delay_Time(0),
      Operating_Mode(0),
      Homing_Offset(0),
      Moving_Threshold(0),
      Temperature_Limit(0),
      Max_Voltage_Limit(0),
      Min_Voltage_Limit(0),
      Acceleration_Limit(0),
      Torque_Limit(0),
      Velocity_Limit(0),
      Max_Position_Limit(0),
      Min_Position_Limit(0),
      External_Port_Mode_1(0),
      External_Port_Mode_2(0),
      External_Port_Mode_3(0),
      External_Port_Mode_4(0),
      Shutdown(0),
      Torque_Enable(0),
      LED_RED(0),
      LED_GREEN(0),
      LED_BLUE(0),
      Velocity_I_Gain(0),
      Velocity_P_Gain(0),
      Position_P_Gain(0),
      Goal_Position(0),
      Goal_Velocity(0),
      Goal_Torque(0),
      Goal_Acceleration(0),
      Moving(0),
      Present_Position(0),
      Present_Velocity(0),
      Present_Current(0),
      Present_Input_Voltage(0),
      Present_Temperature(0),
      Registered_Instruction(0),
      Status_Return_Level(0),
      Hardware_Error_Status(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->Model_Number >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Model_Number >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Model_Number);
      *(outbuffer + offset + 0) = (this->Firmware_Version >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Firmware_Version);
      *(outbuffer + offset + 0) = (this->ID >> (8 * 0)) & 0xFF;
      offset += sizeof(this->ID);
      *(outbuffer + offset + 0) = (this->Baud_Rate >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Baud_Rate);
      *(outbuffer + offset + 0) = (this->Return_Delay_Time >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Return_Delay_Time);
      *(outbuffer + offset + 0) = (this->Operating_Mode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Operating_Mode);
      union {
        int32_t real;
        uint32_t base;
      } u_Homing_Offset;
      u_Homing_Offset.real = this->Homing_Offset;
      *(outbuffer + offset + 0) = (u_Homing_Offset.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Homing_Offset.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Homing_Offset.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Homing_Offset.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Homing_Offset);
      *(outbuffer + offset + 0) = (this->Moving_Threshold >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Moving_Threshold >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->Moving_Threshold >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->Moving_Threshold >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Moving_Threshold);
      *(outbuffer + offset + 0) = (this->Temperature_Limit >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Temperature_Limit);
      *(outbuffer + offset + 0) = (this->Max_Voltage_Limit >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Max_Voltage_Limit >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Max_Voltage_Limit);
      *(outbuffer + offset + 0) = (this->Min_Voltage_Limit >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Min_Voltage_Limit >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Min_Voltage_Limit);
      *(outbuffer + offset + 0) = (this->Acceleration_Limit >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Acceleration_Limit >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->Acceleration_Limit >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->Acceleration_Limit >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Acceleration_Limit);
      *(outbuffer + offset + 0) = (this->Torque_Limit >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Torque_Limit >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Torque_Limit);
      *(outbuffer + offset + 0) = (this->Velocity_Limit >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Velocity_Limit >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->Velocity_Limit >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->Velocity_Limit >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Velocity_Limit);
      union {
        int32_t real;
        uint32_t base;
      } u_Max_Position_Limit;
      u_Max_Position_Limit.real = this->Max_Position_Limit;
      *(outbuffer + offset + 0) = (u_Max_Position_Limit.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Max_Position_Limit.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Max_Position_Limit.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Max_Position_Limit.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Max_Position_Limit);
      union {
        int32_t real;
        uint32_t base;
      } u_Min_Position_Limit;
      u_Min_Position_Limit.real = this->Min_Position_Limit;
      *(outbuffer + offset + 0) = (u_Min_Position_Limit.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Min_Position_Limit.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Min_Position_Limit.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Min_Position_Limit.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Min_Position_Limit);
      *(outbuffer + offset + 0) = (this->External_Port_Mode_1 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->External_Port_Mode_1);
      *(outbuffer + offset + 0) = (this->External_Port_Mode_2 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->External_Port_Mode_2);
      *(outbuffer + offset + 0) = (this->External_Port_Mode_3 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->External_Port_Mode_3);
      *(outbuffer + offset + 0) = (this->External_Port_Mode_4 >> (8 * 0)) & 0xFF;
      offset += sizeof(this->External_Port_Mode_4);
      *(outbuffer + offset + 0) = (this->Shutdown >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Shutdown);
      *(outbuffer + offset + 0) = (this->Torque_Enable >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Torque_Enable);
      *(outbuffer + offset + 0) = (this->LED_RED >> (8 * 0)) & 0xFF;
      offset += sizeof(this->LED_RED);
      *(outbuffer + offset + 0) = (this->LED_GREEN >> (8 * 0)) & 0xFF;
      offset += sizeof(this->LED_GREEN);
      *(outbuffer + offset + 0) = (this->LED_BLUE >> (8 * 0)) & 0xFF;
      offset += sizeof(this->LED_BLUE);
      *(outbuffer + offset + 0) = (this->Velocity_I_Gain >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Velocity_I_Gain >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Velocity_I_Gain);
      *(outbuffer + offset + 0) = (this->Velocity_P_Gain >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Velocity_P_Gain >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Velocity_P_Gain);
      *(outbuffer + offset + 0) = (this->Position_P_Gain >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Position_P_Gain >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Position_P_Gain);
      union {
        int32_t real;
        uint32_t base;
      } u_Goal_Position;
      u_Goal_Position.real = this->Goal_Position;
      *(outbuffer + offset + 0) = (u_Goal_Position.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Goal_Position.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Goal_Position.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Goal_Position.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Goal_Position);
      *(outbuffer + offset + 0) = (this->Goal_Velocity >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Goal_Velocity >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->Goal_Velocity >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->Goal_Velocity >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Goal_Velocity);
      *(outbuffer + offset + 0) = (this->Goal_Torque >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Goal_Torque >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Goal_Torque);
      union {
        int32_t real;
        uint32_t base;
      } u_Goal_Acceleration;
      u_Goal_Acceleration.real = this->Goal_Acceleration;
      *(outbuffer + offset + 0) = (u_Goal_Acceleration.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Goal_Acceleration.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Goal_Acceleration.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Goal_Acceleration.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Goal_Acceleration);
      *(outbuffer + offset + 0) = (this->Moving >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Moving);
      union {
        int32_t real;
        uint32_t base;
      } u_Present_Position;
      u_Present_Position.real = this->Present_Position;
      *(outbuffer + offset + 0) = (u_Present_Position.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Present_Position.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Present_Position.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Present_Position.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Present_Position);
      *(outbuffer + offset + 0) = (this->Present_Velocity >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Present_Velocity >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->Present_Velocity >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->Present_Velocity >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Present_Velocity);
      *(outbuffer + offset + 0) = (this->Present_Current >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Present_Current >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Present_Current);
      *(outbuffer + offset + 0) = (this->Present_Input_Voltage >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Present_Input_Voltage >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Present_Input_Voltage);
      *(outbuffer + offset + 0) = (this->Present_Temperature >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Present_Temperature);
      *(outbuffer + offset + 0) = (this->Registered_Instruction >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Registered_Instruction);
      *(outbuffer + offset + 0) = (this->Status_Return_Level >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Status_Return_Level);
      *(outbuffer + offset + 0) = (this->Hardware_Error_Status >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Hardware_Error_Status);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->Model_Number =  ((uint16_t) (*(inbuffer + offset)));
      this->Model_Number |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->Model_Number);
      this->Firmware_Version =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->Firmware_Version);
      this->ID =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->ID);
      this->Baud_Rate =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->Baud_Rate);
      this->Return_Delay_Time =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->Return_Delay_Time);
      this->Operating_Mode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->Operating_Mode);
      union {
        int32_t real;
        uint32_t base;
      } u_Homing_Offset;
      u_Homing_Offset.base = 0;
      u_Homing_Offset.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Homing_Offset.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Homing_Offset.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Homing_Offset.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Homing_Offset = u_Homing_Offset.real;
      offset += sizeof(this->Homing_Offset);
      this->Moving_Threshold =  ((uint32_t) (*(inbuffer + offset)));
      this->Moving_Threshold |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->Moving_Threshold |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->Moving_Threshold |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->Moving_Threshold);
      this->Temperature_Limit =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->Temperature_Limit);
      this->Max_Voltage_Limit =  ((uint16_t) (*(inbuffer + offset)));
      this->Max_Voltage_Limit |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->Max_Voltage_Limit);
      this->Min_Voltage_Limit =  ((uint16_t) (*(inbuffer + offset)));
      this->Min_Voltage_Limit |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->Min_Voltage_Limit);
      this->Acceleration_Limit =  ((uint32_t) (*(inbuffer + offset)));
      this->Acceleration_Limit |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->Acceleration_Limit |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->Acceleration_Limit |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->Acceleration_Limit);
      this->Torque_Limit =  ((uint16_t) (*(inbuffer + offset)));
      this->Torque_Limit |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->Torque_Limit);
      this->Velocity_Limit =  ((uint32_t) (*(inbuffer + offset)));
      this->Velocity_Limit |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->Velocity_Limit |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->Velocity_Limit |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->Velocity_Limit);
      union {
        int32_t real;
        uint32_t base;
      } u_Max_Position_Limit;
      u_Max_Position_Limit.base = 0;
      u_Max_Position_Limit.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Max_Position_Limit.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Max_Position_Limit.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Max_Position_Limit.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Max_Position_Limit = u_Max_Position_Limit.real;
      offset += sizeof(this->Max_Position_Limit);
      union {
        int32_t real;
        uint32_t base;
      } u_Min_Position_Limit;
      u_Min_Position_Limit.base = 0;
      u_Min_Position_Limit.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Min_Position_Limit.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Min_Position_Limit.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Min_Position_Limit.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Min_Position_Limit = u_Min_Position_Limit.real;
      offset += sizeof(this->Min_Position_Limit);
      this->External_Port_Mode_1 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->External_Port_Mode_1);
      this->External_Port_Mode_2 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->External_Port_Mode_2);
      this->External_Port_Mode_3 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->External_Port_Mode_3);
      this->External_Port_Mode_4 =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->External_Port_Mode_4);
      this->Shutdown =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->Shutdown);
      this->Torque_Enable =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->Torque_Enable);
      this->LED_RED =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->LED_RED);
      this->LED_GREEN =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->LED_GREEN);
      this->LED_BLUE =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->LED_BLUE);
      this->Velocity_I_Gain =  ((uint16_t) (*(inbuffer + offset)));
      this->Velocity_I_Gain |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->Velocity_I_Gain);
      this->Velocity_P_Gain =  ((uint16_t) (*(inbuffer + offset)));
      this->Velocity_P_Gain |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->Velocity_P_Gain);
      this->Position_P_Gain =  ((uint16_t) (*(inbuffer + offset)));
      this->Position_P_Gain |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->Position_P_Gain);
      union {
        int32_t real;
        uint32_t base;
      } u_Goal_Position;
      u_Goal_Position.base = 0;
      u_Goal_Position.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Goal_Position.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Goal_Position.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Goal_Position.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Goal_Position = u_Goal_Position.real;
      offset += sizeof(this->Goal_Position);
      this->Goal_Velocity =  ((uint32_t) (*(inbuffer + offset)));
      this->Goal_Velocity |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->Goal_Velocity |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->Goal_Velocity |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->Goal_Velocity);
      this->Goal_Torque =  ((uint16_t) (*(inbuffer + offset)));
      this->Goal_Torque |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->Goal_Torque);
      union {
        int32_t real;
        uint32_t base;
      } u_Goal_Acceleration;
      u_Goal_Acceleration.base = 0;
      u_Goal_Acceleration.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Goal_Acceleration.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Goal_Acceleration.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Goal_Acceleration.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Goal_Acceleration = u_Goal_Acceleration.real;
      offset += sizeof(this->Goal_Acceleration);
      this->Moving =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->Moving);
      union {
        int32_t real;
        uint32_t base;
      } u_Present_Position;
      u_Present_Position.base = 0;
      u_Present_Position.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Present_Position.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Present_Position.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Present_Position.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Present_Position = u_Present_Position.real;
      offset += sizeof(this->Present_Position);
      this->Present_Velocity =  ((uint32_t) (*(inbuffer + offset)));
      this->Present_Velocity |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->Present_Velocity |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->Present_Velocity |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->Present_Velocity);
      this->Present_Current =  ((uint16_t) (*(inbuffer + offset)));
      this->Present_Current |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->Present_Current);
      this->Present_Input_Voltage =  ((uint16_t) (*(inbuffer + offset)));
      this->Present_Input_Voltage |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->Present_Input_Voltage);
      this->Present_Temperature =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->Present_Temperature);
      this->Registered_Instruction =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->Registered_Instruction);
      this->Status_Return_Level =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->Status_Return_Level);
      this->Hardware_Error_Status =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->Hardware_Error_Status);
     return offset;
    }

    virtual const char * getType() override { return "dynamixel_workbench_msgs/PROExt"; };
    virtual const char * getMD5() override { return "7e5c32f4e1d1adb95d72d66d5dcec77e"; };

  };

}
#endif
