#ifndef _ROS_SERVICE_BusInt32Set_h
#define _ROS_SERVICE_BusInt32Set_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace loki_base_node
{

static const char BUSINT32SET[] = "loki_base_node/BusInt32Set";

  class BusInt32SetRequest : public ros::Msg
  {
    public:
      typedef int16_t _address_type;
      _address_type address;
      typedef int8_t _command_type;
      _command_type command;
      typedef int32_t _value_type;
      _value_type value;
      typedef int8_t _retries_type;
      _retries_type retries;
      typedef int8_t _priority_type;
      _priority_type priority;

    BusInt32SetRequest():
      address(0),
      command(0),
      value(0),
      retries(0),
      priority(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_address;
      u_address.real = this->address;
      *(outbuffer + offset + 0) = (u_address.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_address.base >> (8 * 1)) & 0xFF;
      offset += sizeof(this->address);
      union {
        int8_t real;
        uint8_t base;
      } u_command;
      u_command.real = this->command;
      *(outbuffer + offset + 0) = (u_command.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->command);
      union {
        int32_t real;
        uint32_t base;
      } u_value;
      u_value.real = this->value;
      *(outbuffer + offset + 0) = (u_value.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_value.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_value.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_value.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->value);
      union {
        int8_t real;
        uint8_t base;
      } u_retries;
      u_retries.real = this->retries;
      *(outbuffer + offset + 0) = (u_retries.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->retries);
      union {
        int8_t real;
        uint8_t base;
      } u_priority;
      u_priority.real = this->priority;
      *(outbuffer + offset + 0) = (u_priority.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->priority);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int16_t real;
        uint16_t base;
      } u_address;
      u_address.base = 0;
      u_address.base |= ((uint16_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_address.base |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->address = u_address.real;
      offset += sizeof(this->address);
      union {
        int8_t real;
        uint8_t base;
      } u_command;
      u_command.base = 0;
      u_command.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->command = u_command.real;
      offset += sizeof(this->command);
      union {
        int32_t real;
        uint32_t base;
      } u_value;
      u_value.base = 0;
      u_value.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_value.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_value.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_value.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->value = u_value.real;
      offset += sizeof(this->value);
      union {
        int8_t real;
        uint8_t base;
      } u_retries;
      u_retries.base = 0;
      u_retries.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->retries = u_retries.real;
      offset += sizeof(this->retries);
      union {
        int8_t real;
        uint8_t base;
      } u_priority;
      u_priority.base = 0;
      u_priority.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->priority = u_priority.real;
      offset += sizeof(this->priority);
     return offset;
    }

    const char * getType(){ return BUSINT32SET; };
    const char * getMD5(){ return "b1dadd76ff0a038121a9b5a97053932f"; };

  };

  class BusInt32SetResponse : public ros::Msg
  {
    public:
      typedef int8_t _status_type;
      _status_type status;

    BusInt32SetResponse():
      status(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_status;
      u_status.real = this->status;
      *(outbuffer + offset + 0) = (u_status.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->status);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer)
    {
      int offset = 0;
      union {
        int8_t real;
        uint8_t base;
      } u_status;
      u_status.base = 0;
      u_status.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->status = u_status.real;
      offset += sizeof(this->status);
     return offset;
    }

    const char * getType(){ return BUSINT32SET; };
    const char * getMD5(){ return "581cc55c12abfc219e446416014f6d0e"; };

  };

  class BusInt32Set {
    public:
    typedef BusInt32SetRequest Request;
    typedef BusInt32SetResponse Response;
  };

}
#endif
