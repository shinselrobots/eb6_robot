#ifndef _ROS_dynamixel_workbench_msgs_MX2_h
#define _ROS_dynamixel_workbench_msgs_MX2_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dynamixel_workbench_msgs
{

  class MX2 : public ros::Msg
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
      typedef uint8_t _Drive_Mode_type;
      _Drive_Mode_type Drive_Mode;
      typedef uint8_t _Operating_Mode_type;
      _Operating_Mode_type Operating_Mode;
      typedef uint8_t _Secondary_ID_type;
      _Secondary_ID_type Secondary_ID;
      typedef uint8_t _Protocol_Version_type;
      _Protocol_Version_type Protocol_Version;
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
      typedef uint16_t _PWM_Limit_type;
      _PWM_Limit_type PWM_Limit;
      typedef uint32_t _Acceleration_Limit_type;
      _Acceleration_Limit_type Acceleration_Limit;
      typedef uint32_t _Velocity_Limit_type;
      _Velocity_Limit_type Velocity_Limit;
      typedef uint32_t _Max_Position_Limit_type;
      _Max_Position_Limit_type Max_Position_Limit;
      typedef uint32_t _Min_Position_Limit_type;
      _Min_Position_Limit_type Min_Position_Limit;
      typedef uint8_t _Shutdown_type;
      _Shutdown_type Shutdown;
      typedef uint8_t _Torque_Enable_type;
      _Torque_Enable_type Torque_Enable;
      typedef uint8_t _LED_type;
      _LED_type LED;
      typedef uint8_t _Status_Return_Level_type;
      _Status_Return_Level_type Status_Return_Level;
      typedef uint8_t _Registered_Instruction_type;
      _Registered_Instruction_type Registered_Instruction;
      typedef uint8_t _Hardware_Error_Status_type;
      _Hardware_Error_Status_type Hardware_Error_Status;
      typedef uint16_t _Velocity_I_Gain_type;
      _Velocity_I_Gain_type Velocity_I_Gain;
      typedef uint16_t _Velocity_P_Gain_type;
      _Velocity_P_Gain_type Velocity_P_Gain;
      typedef uint16_t _Position_D_Gain_type;
      _Position_D_Gain_type Position_D_Gain;
      typedef uint16_t _Position_I_Gain_type;
      _Position_I_Gain_type Position_I_Gain;
      typedef uint16_t _Position_P_Gain_type;
      _Position_P_Gain_type Position_P_Gain;
      typedef uint16_t _Feedforward_2nd_Gain_type;
      _Feedforward_2nd_Gain_type Feedforward_2nd_Gain;
      typedef uint16_t _Feedforward_1st_Gain_type;
      _Feedforward_1st_Gain_type Feedforward_1st_Gain;
      typedef uint8_t _Bus_Watchdog_type;
      _Bus_Watchdog_type Bus_Watchdog;
      typedef int16_t _Goal_PWM_type;
      _Goal_PWM_type Goal_PWM;
      typedef int32_t _Goal_Velocity_type;
      _Goal_Velocity_type Goal_Velocity;
      typedef uint32_t _Profile_Acceleration_type;
      _Profile_Acceleration_type Profile_Acceleration;
      typedef uint32_t _Profile_Velocity_type;
      _Profile_Velocity_type Profile_Velocity;
      typedef uint32_t _Goal_Position_type;
      _Goal_Position_type Goal_Position;
      typedef uint16_t _Realtime_Tick_type;
      _Realtime_Tick_type Realtime_Tick;
      typedef uint8_t _Moving_type;
      _Moving_type Moving;
      typedef uint8_t _Moving_Status_type;
      _Moving_Status_type Moving_Status;
      typedef int16_t _Present_PWM_type;
      _Present_PWM_type Present_PWM;
      typedef int16_t _Present_Load_type;
      _Present_Load_type Present_Load;
      typedef int32_t _Present_Velocity_type;
      _Present_Velocity_type Present_Velocity;
      typedef int32_t _Present_Position_type;
      _Present_Position_type Present_Position;
      typedef uint32_t _Velocity_Trajectory_type;
      _Velocity_Trajectory_type Velocity_Trajectory;
      typedef uint32_t _Position_Trajectory_type;
      _Position_Trajectory_type Position_Trajectory;
      typedef uint16_t _Present_Input_Voltage_type;
      _Present_Input_Voltage_type Present_Input_Voltage;
      typedef uint8_t _Present_Temperature_type;
      _Present_Temperature_type Present_Temperature;

    MX2():
      Model_Number(0),
      Firmware_Version(0),
      ID(0),
      Baud_Rate(0),
      Return_Delay_Time(0),
      Drive_Mode(0),
      Operating_Mode(0),
      Secondary_ID(0),
      Protocol_Version(0),
      Homing_Offset(0),
      Moving_Threshold(0),
      Temperature_Limit(0),
      Max_Voltage_Limit(0),
      Min_Voltage_Limit(0),
      PWM_Limit(0),
      Acceleration_Limit(0),
      Velocity_Limit(0),
      Max_Position_Limit(0),
      Min_Position_Limit(0),
      Shutdown(0),
      Torque_Enable(0),
      LED(0),
      Status_Return_Level(0),
      Registered_Instruction(0),
      Hardware_Error_Status(0),
      Velocity_I_Gain(0),
      Velocity_P_Gain(0),
      Position_D_Gain(0),
      Position_I_Gain(0),
      Position_P_Gain(0),
      Feedforward_2nd_Gain(0),
      Feedforward_1st_Gain(0),
      Bus_Watchdog(0),
      Goal_PWM(0),
      Goal_Velocity(0),
      Profile_Acceleration(0),
      Profile_Velocity(0),
      Goal_Position(0),
      Realtime_Tick(0),
      Moving(0),
      Moving_Status(0),
      Present_PWM(0),
      Present_Load(0),
      Present_Velocity(0),
      Present_Position(0),
      Velocity_Trajectory(0),
      Position_Trajectory(0),
      Present_Input_Voltage(0),
      Present_Temperature(0)
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
      *(outbuffer + offset + 0) = (this->Drive_Mode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Drive_Mode);
      *(outbuffer + offset + 0) = (this->Operating_Mode >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Operating_Mode);
      *(outbuffer + offset + 0) = (this->Secondary_ID >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Secondary_ID);
      *(outbuffer + offset + 0) = (this->Protocol_Version >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Protocol_Version);
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
      *(outbuffer + offset + 0) = (this->PWM_Limit >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->PWM_Limit >> (8 * 1)) & 0xFF;
      offset += sizeof(this->PWM_Limit);
      *(outbuffer + offset + 0) = (this->Acceleration_Limit >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Acceleration_Limit >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->Acceleration_Limit >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->Acceleration_Limit >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Acceleration_Limit);
      *(outbuffer + offset + 0) = (this->Velocity_Limit >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Velocity_Limit >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->Velocity_Limit >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->Velocity_Limit >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Velocity_Limit);
      *(outbuffer + offset + 0) = (this->Max_Position_Limit >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Max_Position_Limit >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->Max_Position_Limit >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->Max_Position_Limit >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Max_Position_Limit);
      *(outbuffer + offset + 0) = (this->Min_Position_Limit >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Min_Position_Limit >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->Min_Position_Limit >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->Min_Position_Limit >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Min_Position_Limit);
      *(outbuffer + offset + 0) = (this->Shutdown >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Shutdown);
      *(outbuffer + offset + 0) = (this->Torque_Enable >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Torque_Enable);
      *(outbuffer + offset + 0) = (this->LED >> (8 * 0)) & 0xFF;
      offset += sizeof(this->LED);
      *(outbuffer + offset + 0) = (this->Status_Return_Level >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Status_Return_Level);
      *(outbuffer + offset + 0) = (this->Registered_Instruction >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Registered_Instruction);
      *(outbuffer + offset + 0) = (this->Hardware_Error_Status >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Hardware_Error_Status);
      *(outbuffer + offset + 0) = (this->Velocity_I_Gain >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Velocity_I_Gain >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Velocity_I_Gain);
      *(outbuffer + offset + 0) = (this->Velocity_P_Gain >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Velocity_P_Gain >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Velocity_P_Gain);
      *(outbuffer + offset + 0) = (this->Position_D_Gain >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Position_D_Gain >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Position_D_Gain);
      *(outbuffer + offset + 0) = (this->Position_I_Gain >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Position_I_Gain >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Position_I_Gain);
      *(outbuffer + offset + 0) = (this->Position_P_Gain >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Position_P_Gain >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Position_P_Gain);
      *(outbuffer + offset + 0) = (this->Feedforward_2nd_Gain >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Feedforward_2nd_Gain >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Feedforward_2nd_Gain);
      *(outbuffer + offset + 0) = (this->Feedforward_1st_Gain >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Feedforward_1st_Gain >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Feedforward_1st_Gain);
      *(outbuffer + offset + 0) = (this->Bus_Watchdog >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Bus_Watchdog);
      union {
        int16_t real;
        uint16_t base;
      } u_Goal_PWM;
      u_Goal_PWM.real = this->Goal_PWM;
      *(outbuffer + offset + 0) = (u_Goal_PWM.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Goal_PWM.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Goal_PWM);
      union {
        int32_t real;
        uint32_t base;
      } u_Goal_Velocity;
      u_Goal_Velocity.real = this->Goal_Velocity;
      *(outbuffer + offset + 0) = (u_Goal_Velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Goal_Velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Goal_Velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Goal_Velocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Goal_Velocity);
      *(outbuffer + offset + 0) = (this->Profile_Acceleration >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Profile_Acceleration >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->Profile_Acceleration >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->Profile_Acceleration >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Profile_Acceleration);
      *(outbuffer + offset + 0) = (this->Profile_Velocity >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Profile_Velocity >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->Profile_Velocity >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->Profile_Velocity >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Profile_Velocity);
      *(outbuffer + offset + 0) = (this->Goal_Position >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Goal_Position >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->Goal_Position >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->Goal_Position >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Goal_Position);
      *(outbuffer + offset + 0) = (this->Realtime_Tick >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Realtime_Tick >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Realtime_Tick);
      *(outbuffer + offset + 0) = (this->Moving >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Moving);
      *(outbuffer + offset + 0) = (this->Moving_Status >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Moving_Status);
      union {
        int16_t real;
        uint16_t base;
      } u_Present_PWM;
      u_Present_PWM.real = this->Present_PWM;
      *(outbuffer + offset + 0) = (u_Present_PWM.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Present_PWM.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Present_PWM);
      union {
        int16_t real;
        uint16_t base;
      } u_Present_Load;
      u_Present_Load.real = this->Present_Load;
      *(outbuffer + offset + 0) = (u_Present_Load.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Present_Load.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Present_Load);
      union {
        int32_t real;
        uint32_t base;
      } u_Present_Velocity;
      u_Present_Velocity.real = this->Present_Velocity;
      *(outbuffer + offset + 0) = (u_Present_Velocity.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_Present_Velocity.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_Present_Velocity.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_Present_Velocity.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Present_Velocity);
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
      *(outbuffer + offset + 0) = (this->Velocity_Trajectory >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Velocity_Trajectory >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->Velocity_Trajectory >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->Velocity_Trajectory >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Velocity_Trajectory);
      *(outbuffer + offset + 0) = (this->Position_Trajectory >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Position_Trajectory >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->Position_Trajectory >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->Position_Trajectory >> (8 * 3)) & 0xFF;
      offset += sizeof(this->Position_Trajectory);
      *(outbuffer + offset + 0) = (this->Present_Input_Voltage >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->Present_Input_Voltage >> (8 * 1)) & 0xFF;
      offset += sizeof(this->Present_Input_Voltage);
      *(outbuffer + offset + 0) = (this->Present_Temperature >> (8 * 0)) & 0xFF;
      offset += sizeof(this->Present_Temperature);
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
      this->Drive_Mode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->Drive_Mode);
      this->Operating_Mode =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->Operating_Mode);
      this->Secondary_ID =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->Secondary_ID);
      this->Protocol_Version =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->Protocol_Version);
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
      this->PWM_Limit =  ((uint16_t) (*(inbuffer + offset)));
      this->PWM_Limit |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->PWM_Limit);
      this->Acceleration_Limit =  ((uint32_t) (*(inbuffer + offset)));
      this->Acceleration_Limit |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->Acceleration_Limit |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->Acceleration_Limit |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->Acceleration_Limit);
      this->Velocity_Limit =  ((uint32_t) (*(inbuffer + offset)));
      this->Velocity_Limit |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->Velocity_Limit |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->Velocity_Limit |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->Velocity_Limit);
      this->Max_Position_Limit =  ((uint32_t) (*(inbuffer + offset)));
      this->Max_Position_Limit |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->Max_Position_Limit |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->Max_Position_Limit |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->Max_Position_Limit);
      this->Min_Position_Limit =  ((uint32_t) (*(inbuffer + offset)));
      this->Min_Position_Limit |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->Min_Position_Limit |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->Min_Position_Limit |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->Min_Position_Limit);
      this->Shutdown =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->Shutdown);
      this->Torque_Enable =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->Torque_Enable);
      this->LED =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->LED);
      this->Status_Return_Level =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->Status_Return_Level);
      this->Registered_Instruction =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->Registered_Instruction);
      this->Hardware_Error_Status =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->Hardware_Error_Status);
      this->Velocity_I_Gain =  ((uint16_t) (*(inbuffer + offset)));
      this->Velocity_I_Gain |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->Velocity_I_Gain);
      this->Velocity_P_Gain =  ((uint16_t) (*(inbuffer + offset)));
      this->Velocity_P_Gain |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->Velocity_P_Gain);
      this->Position_D_Gain =  ((uint16_t) (*(inbuffer + offset)));
      this->Position_D_Gain |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->Position_D_Gain);
      this->Position_I_Gain =  ((uint16_t) (*(inbuffer + offset)));
      this->Position_I_Gain |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->Position_I_Gain);
      this->Position_P_Gain =  ((uint16_t) (*(inbuffer + offset)));
      this->Position_P_Gain |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->Position_P_Gain);
      this->Feedforward_2nd_Gain =  ((uint16_t) (*(inbuffer + offset)));
      this->Feedforward_2nd_Gain |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->Feedforward_2nd_Gain);
      this->Feedforward_1st_Gain =  ((uint16_t) (*(inbuffer + offset)));
      this->Feedforward_1st_Gain |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->Feedforward_1st_Gain);
      this->Bus_Watchdog =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->Bus_Watchdog);
      union {
        int16_t real;
        uint16_t base;
      } u_Goal_PWM;
      u_Goal_PWM.base = 0;
      u_Goal_PWM.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Goal_PWM.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->Goal_PWM = u_Goal_PWM.real;
      offset += sizeof(this->Goal_PWM);
      union {
        int32_t real;
        uint32_t base;
      } u_Goal_Velocity;
      u_Goal_Velocity.base = 0;
      u_Goal_Velocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Goal_Velocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Goal_Velocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Goal_Velocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Goal_Velocity = u_Goal_Velocity.real;
      offset += sizeof(this->Goal_Velocity);
      this->Profile_Acceleration =  ((uint32_t) (*(inbuffer + offset)));
      this->Profile_Acceleration |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->Profile_Acceleration |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->Profile_Acceleration |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->Profile_Acceleration);
      this->Profile_Velocity =  ((uint32_t) (*(inbuffer + offset)));
      this->Profile_Velocity |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->Profile_Velocity |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->Profile_Velocity |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->Profile_Velocity);
      this->Goal_Position =  ((uint32_t) (*(inbuffer + offset)));
      this->Goal_Position |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->Goal_Position |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->Goal_Position |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->Goal_Position);
      this->Realtime_Tick =  ((uint16_t) (*(inbuffer + offset)));
      this->Realtime_Tick |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->Realtime_Tick);
      this->Moving =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->Moving);
      this->Moving_Status =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->Moving_Status);
      union {
        int16_t real;
        uint16_t base;
      } u_Present_PWM;
      u_Present_PWM.base = 0;
      u_Present_PWM.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Present_PWM.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->Present_PWM = u_Present_PWM.real;
      offset += sizeof(this->Present_PWM);
      union {
        int16_t real;
        uint16_t base;
      } u_Present_Load;
      u_Present_Load.base = 0;
      u_Present_Load.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Present_Load.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->Present_Load = u_Present_Load.real;
      offset += sizeof(this->Present_Load);
      union {
        int32_t real;
        uint32_t base;
      } u_Present_Velocity;
      u_Present_Velocity.base = 0;
      u_Present_Velocity.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_Present_Velocity.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_Present_Velocity.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_Present_Velocity.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->Present_Velocity = u_Present_Velocity.real;
      offset += sizeof(this->Present_Velocity);
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
      this->Velocity_Trajectory =  ((uint32_t) (*(inbuffer + offset)));
      this->Velocity_Trajectory |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->Velocity_Trajectory |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->Velocity_Trajectory |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->Velocity_Trajectory);
      this->Position_Trajectory =  ((uint32_t) (*(inbuffer + offset)));
      this->Position_Trajectory |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->Position_Trajectory |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->Position_Trajectory |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->Position_Trajectory);
      this->Present_Input_Voltage =  ((uint16_t) (*(inbuffer + offset)));
      this->Present_Input_Voltage |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->Present_Input_Voltage);
      this->Present_Temperature =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->Present_Temperature);
     return offset;
    }

    virtual const char * getType() override { return "dynamixel_workbench_msgs/MX2"; };
    virtual const char * getMD5() override { return "ea49a2701ed85afc888c2c84aca6af1f"; };

  };

}
#endif
