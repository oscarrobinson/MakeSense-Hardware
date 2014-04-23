/*
  SendOurCode.d - Library for sending the data recieved from 
  sensor and code for type of sensor
*/  


#ifndef SendOurCode_h
#define SendOurCode_h

#include "Arduino.h"
#include <string.h>


class SendOurCode
{
   public:
     SendOurCode(std::string data, std::string code);
     void send_data();
   private:
     int _data;
     std::string _code;
     void oneString();
};

#endif
