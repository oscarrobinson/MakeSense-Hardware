#include "SendOurCode.h"
#include "Arduino.h"

#include <iostream>  
#include <Serial.h>
#include <EngduinoLEDs.h>
#include <EngduinoLight.h>
#include <EngduinoThermistor.h>


SendOurCode::SendOurCode(String data, String code)
{
	this->data = data;
	this->code = code;
}
String SendOurCode::oneString(String data, String code)
{
String one_string = ("%s X %s",data, code);
return one_string;
}

void SendOurCode::send_data() 
{
    Serial.begin(115200);
    EngduinoLEDs.begin();
    EngduinoLight.begin();
    EngduinoThermistor.begin();
	String one_string = oneString(data, code);
	Serial.println(one_string);

}