#ifndef _ROS_SERVICE_speech_recognition_intent_h
#define _ROS_SERVICE_speech_recognition_intent_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace speech_recognition_common
{

static const char SPEECH_RECOGNITION_INTENT[] = "speech_recognition_common/speech_recognition_intent";

  class speech_recognition_intentRequest : public ros::Msg
  {
    public:
      typedef const char* _phrase_heard_type;
      _phrase_heard_type phrase_heard;
      typedef const char* _suggested_response_type;
      _suggested_response_type suggested_response;
      typedef bool _partial_phrase_type;
      _partial_phrase_type partial_phrase;

    speech_recognition_intentRequest():
      phrase_heard(""),
      suggested_response(""),
      partial_phrase(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      uint32_t length_phrase_heard = strlen(this->phrase_heard);
      varToArr(outbuffer + offset, length_phrase_heard);
      offset += 4;
      memcpy(outbuffer + offset, this->phrase_heard, length_phrase_heard);
      offset += length_phrase_heard;
      uint32_t length_suggested_response = strlen(this->suggested_response);
      varToArr(outbuffer + offset, length_suggested_response);
      offset += 4;
      memcpy(outbuffer + offset, this->suggested_response, length_suggested_response);
      offset += length_suggested_response;
      union {
        bool real;
        uint8_t base;
      } u_partial_phrase;
      u_partial_phrase.real = this->partial_phrase;
      *(outbuffer + offset + 0) = (u_partial_phrase.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->partial_phrase);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      uint32_t length_phrase_heard;
      arrToVar(length_phrase_heard, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_phrase_heard; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_phrase_heard-1]=0;
      this->phrase_heard = (char *)(inbuffer + offset-1);
      offset += length_phrase_heard;
      uint32_t length_suggested_response;
      arrToVar(length_suggested_response, (inbuffer + offset));
      offset += 4;
      for(unsigned int k= offset; k< offset+length_suggested_response; ++k){
          inbuffer[k-1]=inbuffer[k];
      }
      inbuffer[offset+length_suggested_response-1]=0;
      this->suggested_response = (char *)(inbuffer + offset-1);
      offset += length_suggested_response;
      union {
        bool real;
        uint8_t base;
      } u_partial_phrase;
      u_partial_phrase.base = 0;
      u_partial_phrase.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->partial_phrase = u_partial_phrase.real;
      offset += sizeof(this->partial_phrase);
     return offset;
    }

    virtual const char * getType() override { return SPEECH_RECOGNITION_INTENT; };
    virtual const char * getMD5() override { return "8175a3c358ecad3007e9945f90fffce4"; };

  };

  class speech_recognition_intentResponse : public ros::Msg
  {
    public:
      typedef bool _result_type;
      _result_type result;

    speech_recognition_intentResponse():
      result(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_result;
      u_result.real = this->result;
      *(outbuffer + offset + 0) = (u_result.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->result);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      union {
        bool real;
        uint8_t base;
      } u_result;
      u_result.base = 0;
      u_result.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->result = u_result.real;
      offset += sizeof(this->result);
     return offset;
    }

    virtual const char * getType() override { return SPEECH_RECOGNITION_INTENT; };
    virtual const char * getMD5() override { return "eb13ac1f1354ccecb7941ee8fa2192e8"; };

  };

  class speech_recognition_intent {
    public:
    typedef speech_recognition_intentRequest Request;
    typedef speech_recognition_intentResponse Response;
  };

}
#endif
