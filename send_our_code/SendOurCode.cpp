#include "Arduino.h"
#include "SendOurCode.h"
#include <string.h>

SendOurCode::SendOurCode(int data, std::string code)
{
	_data = data;
	_code = code;
}
std::string SendOurCode::oneString(int data, std::string code)
{
std::string one_string;
sprintf(one_string,"%d X %s", data, code);
return one_string;
}

void SendOurCode::send_data()
{
	std::string one_string = oneString(_data, _code);
	Serial.println(one_string);
	
}
