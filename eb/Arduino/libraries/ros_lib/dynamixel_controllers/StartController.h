#ifndef _ROS_SERVICE_StartController_h
#define _ROS_SERVICE_StartController_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace dynamixel_controllers
{

static const char STARTCONTROLLER[] = "dynamixel_controllers/StartController";

  class StartControllerRequest : public ros::Msg
  {
    public:
      typedef const char* _port_name_type;
      _port_name_type port_name;
      typedef const char* _package_path_type;
      _package_path_type package_path;
      typedef const char* _module_name_type;
      _module_name_type module_name;
      typedef const char* _class_name_type;
      _class_name_type class_name;
      typedef const char* _controller_name_type;
      _controller_name_type controller_name;
      uint32_t dependencies_length;
      typedef char* _dependencies_type;
      _dependencies_type st_dependencies;
      _dependencies_type * dependencies;

    StartControllerRequest():
      port_name(""),
      package_path(""),
      module_name(""),
      class_name(""),
      controller_name(""),
      dependencies_length(0), st_dependencies(), dependencies(nullptr)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_port_name = strlen(this->port_name);
      varToArr(outbuffer + offset, length_port_name);
      offset += 4;
      memcpy(outbuffer + offset, this->port_name, length_port_name);
      offset += length_port_name;
      uint32_t length_package_path = strlen(this->package_path);
      varToArr(outbuffer + offset, length_package_path);
      offset += 4;
      memcpy(outbuffer + offset, this->package_path, length_package_path);
      offset += length_package_path;
      uint32_t length_module_name = strlen(this->module_name);
      varToArr(outbuffer + offset, length_module_name);
      offset += 4;
      memcpy(outbuffer + offset, this->module_name, length_module_name);
      offset += length_module_name;
      uint32_t length_class_name = strlen(this->class_name);
      varToArr(outbuffer + offset, length_class_name);
      offset += 4;
      memcpy(outbuffer + offset, this->class_name, length_class_name);
      offset += length_class_name;
      uint32_t length_controller_name = strlen(this->controller_name);
      varToArr(outbuffer + offset, length_controller_name);
      offset += 4;
      memcpy(outbuffer + offset, this->controller_name, length_controller_name);
      offset += length_controller_name;
      *(outbuffer + offset + 0) = (this->dependencies_length >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->dependencies_length >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->dependencies_length >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->dependencies_length >> (8 * 3)) & 0xFF;
      offset += sizeof(this->dependencies_length);
      for( uint32_t i = 0; i < dependencies_length; i++){
      uint32_t length_dependenciesi = strlen(this->dependencies[i]);
      varToArr(outbuffer + offset, length_dependenciesi);
      offset += 4;
      memcpy(outbuffer + offset, this->dependencies[i], length_dependenciesi);
      offset += length_dependenciesi;
      }
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_port_name;
      arrToVar(length_port_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_port_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_port_name-1]=0;
      this->port_name = (char *)(inbuffer + offset-1);
      offset += length_port_name;
      uint32_t length_package_path;
      arrToVar(length_package_path, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_package_path; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_package_path-1]=0;
      this->package_path = (char *)(inbuffer + offset-1);
      offset += length_package_path;
      uint32_t length_module_name;
      arrToVar(length_module_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_module_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_module_name-1]=0;
      this->module_name = (char *)(inbuffer + offset-1);
      offset += length_module_name;
      uint32_t length_class_name;
      arrToVar(length_class_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_class_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_class_name-1]=0;
      this->class_name = (char *)(inbuffer + offset-1);
      offset += length_class_name;
      uint32_t length_controller_name;
      arrToVar(length_controller_name, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_controller_name; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_controller_name-1]=0;
      this->controller_name = (char *)(inbuffer + offset-1);
      offset += length_controller_name;
      uint32_t dependencies_lengthT = ((uint32_t) (*(inbuffer + offset))); 
      dependencies_lengthT |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1); 
      dependencies_lengthT |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2); 
      dependencies_lengthT |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3); 
      offset += sizeof(this->dependencies_length);
      if(dependencies_lengthT > dependencies_length)
        this->dependencies = (char**)realloc(this->dependencies, dependencies_lengthT * sizeof(char*));
      dependencies_length = dependencies_lengthT;
      for( uint32_t i = 0; i < dependencies_length; i++){
      uint32_t length_st_dependencies;
      arrToVar(length_st_dependencies, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_st_dependencies; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_st_dependencies-1]=0;
      this->st_dependencies = (char *)(inbuffer + offset-1);
      offset += length_st_dependencies;
        memcpy( &(this->dependencies[i]), &(this->st_dependencies), sizeof(char*));
      }
     return offset;
    }

    virtual const char * getType() override { return STARTCONTROLLER; };
    virtual const char * getMD5() override { return "7785d708c83a180befd2fe3450dd9d41"; };

  };

  class StartControllerResponse : public ros::Msg
  {
    public:
      typedef bool _success_type;
      _success_type success;
      typedef const char* _reason_type;
      _reason_type reason;

    StartControllerResponse():
      success(0),
      reason("")
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.real = this->success;
      *(outbuffer + offset + 0) = (u_success.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->success);
      uint32_t length_reason = strlen(this->reason);
      varToArr(outbuffer + offset, length_reason);
      offset += 4;
      memcpy(outbuffer + offset, this->reason, length_reason);
      offset += length_reason;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_success;
      u_success.base = 0;
      u_success.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->success = u_success.real;
      offset += sizeof(this->success);
      uint32_t length_reason;
      arrToVar(length_reason, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_reason; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_reason-1]=0;
      this->reason = (char *)(inbuffer + offset-1);
      offset += length_reason;
     return offset;
    }

    virtual const char * getType() override { return STARTCONTROLLER; };
    virtual const char * getMD5() override { return "a4d50a34d34f18de48e2436ff1472594"; };

  };

  class StartController {
    public:
    typedef StartControllerRequest Request;
    typedef StartControllerResponse Response;
  };

}
#endif
