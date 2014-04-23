/*
  SendOurCode.d - Library for sending the data recieved from 
  sensor and code for type of sensor
*/  

#include<string>
#ifndef SendOurCode_h
#define SendOurCode_h

#include "string.h"
#include "Arduino.h"

using namespace std;


class SendOurCode
{

   public:
     SendOurCode(String data, String code);
     void send_data();
   private:
     String data;
     String code;
     String oneString(String data, String code);
};

#endif