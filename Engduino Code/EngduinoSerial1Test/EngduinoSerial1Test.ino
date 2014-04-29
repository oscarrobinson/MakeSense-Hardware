// Light sensor demo
// There is no include file for this sensor. The value is read
// directly from an analog pin
//
#include <EngduinoLEDs.h>
#include <EngduinoLight.h>
//#include <EngduinoThermistor.h>

void setup() {
  Serial.begin(115200);
  EngduinoLEDs.begin();
  EngduinoLight.begin();
  //EngduinoThermistor.begin();
}

void loop()
{ 
  int l;
  float t;
 
  l = EngduinoLight.lightLevel();
  //t = EngduinoThermistor.temperature();
   
  //Serial1.print(t);
  //Serial1.print(" ");
  Serial1.print(l);

  
  delay(2000);
}
