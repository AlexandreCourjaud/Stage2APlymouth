#ifndef _ROS_SERVICE_BusUInt64Get_h
#define _ROS_SERVICE_BusUInt64Get_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace loki_base_node
{

static const char BUSUINT64GET[] = "loki_base_node/BusUInt64Get";

  class BusUInt64GetRequest : public ros::Msg
  {
    public:
      typedef int16_t _address_type;
      _address_type address;
      typedef int8_t _command_type;
      _command_type command;
      typedef int8_t _retries_type;
      _retries_type retries;
      typedef int8_t _priority_type;
      _priority_type priority;

    BusUInt64GetRequest():
      address(0),
      command(0),
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

    const char * getType(){ return BUSUINT64GET; };
    const char * getMD5(){ return "8f34dd8065292bfe7f0cd82f981e1007"; };

  };

  class BusUInt64GetResponse : public ros::Msg
  {
    public:
      typedef uint64_t _value_type;
      _value_type value;
      typedef int8_t _status_type;
      _status_type status;

    BusUInt64GetResponse():
      value(0),
      status(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->value >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->value >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->value >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->value >> (8 * 3)) & 0xFF;
      *(outbuffer + offset + 4) = (this->value >> (8 * 4)) & 0xFF;
      *(outbuffer + offset + 5) = (this->value >> (8 * 5)) & 0xFF;
      *(outbuffer + offset + 6) = (this->value >> (8 * 6)) & 0xFF;
      *(outbuffer + offset + 7) = (this->value >> (8 * 7)) & 0xFF;
      offset += sizeof(this->value);
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
      this->value =  ((uint64_t) (*(inbuffer + offset)));
      this->value |= ((uint64_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->value |= ((uint64_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->value |= ((uint64_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->value |= ((uint64_t) (*(inbuffer + offset + 4))) << (8 * 4);
      this->value |= ((uint64_t) (*(inbuffer + offset + 5))) << (8 * 5);
      this->value |= ((uint64_t) (*(inbuffer + offset + 6))) << (8 * 6);
      this->value |= ((uint64_t) (*(inbuffer + offset + 7))) << (8 * 7);
      offset += sizeof(this->value);
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

    const char * getType(){ return BUSUINT64GET; };
    const char * getMD5(){ return "7ef052270f7aff1b46a97cd453f759e2"; };

  };

  class BusUInt64Get {
    public:
    typedef BusUInt64GetRequest Request;
    typedef BusUInt64GetResponse Response;
  };

}
#endif
